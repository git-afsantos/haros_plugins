###
# standard packages
import csv
import os
import xml.etree.ElementTree as ET

###
# third-party packages
import rosgraph.names

###
# internal packages
import launch_analyser.substitution_args as sub_args


def _resolve_base_name(name, ns = "/"):
    assert name[0].isalpha() and not "/" in name
    if ns != "/":
        return ns + "/" + name
    return ns + name

def _resolve_non_private_name(name, ns = "/"):
    assert name[0].isalpha() or name[0] == "/"
    if name[0] == "/":
        return name
    if ns != "/":
        return ns + "/" + name
    return ns + name
    


###
# exceptions

class IncludeCycleError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class InvalidTagError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class InvalidAttributeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


###
# unknown results

UNKNOWN = "<?>"

class Condition(object):
    def __init__(self, value, negate = False):
        self.value = value
        self.negate = negate

    _TRUTH = ("true", "1")
    _FALSE = ("false", "0")

    @classmethod
    def from_value(cls, value, negate = False, tolerate = False):
        v = value.lower()
        if negate:
            t = Condition._FALSE
            f = Condition._TRUTH
        else:
            t = Condition._TRUTH
            f = Condition._FALSE
        if v == f[0] or v == f[1]:
            return False
        if v == t[0] or v == t[1]:
            return True
        if tolerate:
            return cls(value, negate = negate)
        raise InvalidAttributeError(value)

    @classmethod
    def from_attribute(cls, attribute, negate = False):
        if attribute.unknown:
            return cls(attribute.value, negate = negate)
        if negate:
            t = Condition._FALSE
            f = Condition._TRUTH
        else:
            t = Condition._TRUTH
            f = Condition._FALSE
        arg = attribute.value.lower()
        if arg in f:
            return False
        if not arg in t:
            raise InvalidAttributeError(arg)
        return True

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        if self.negate:
            return "not " + self.value
        return self.value

###
# launch file model

# http://wiki.ros.org/roslaunch/XML

# The only possible scopes are <launch> and <group>.
# These can have children (<group>, <node>), and can <include> other files.
# <arg> apply to the <launch> within which they are declared.
# <node> are leaves of the parsing tree.
# They may have children (<remap>, <rosparam>, <param>), but the children
# apply the effects globally. Nesting only provides convenience
# for namespace resolution.
# <param> can be put inside of a <node>,
# in which case the parameter is treated like a private parameter.

class LaunchFile(object):
    def __init__(self, name, args = None, ns = "/"):
        self.name = name
        self.arguments = args or {} # all evaluated <arg>
        self.nodes = []             # all evaluated <node>
        self.includes = []          # all evaluated <include>
        self.unknown = []           # unknown references

        self.stats = Stats(name)



class Resource(object):
    def __init__(self, name, ns = "/"):
        self.name = _resolve_non_private_name(name, ns)
        self.original_name = self.name


class Node(Resource):
    def __init__(self, launch, name, pkg, ntype, args = None, ns = "/",
                 remaps = None):
        Resource.__init__(self, name, ns = ns)
        self.launch = launch
        self.package = pkg
        self.node_type = ntype      # executable name
        self.argv = args if not args is None else []
        self.remaps = remaps if not remaps is None else {}

class ConditionalNode(Node):
    def __init__(self, launch, name, pkg, ntype, conditions,
                 args = None, ns = "/", remaps = None):
        Node.__init__(self, launch, name, pkg, ntype, args = args, ns = ns,
                      remaps = remaps)
        self.conditions = conditions    # if/unless conditions to launch


class Topic(Resource):
    def __init__(self, name, ns = ""):
        Resource.__init__(self, name, ns = ns)
        self.subscribers = []
        self.publishers = []

class Service(Resource):
    def __init__(self, name, ns = ""):
        Resource.__init__(self, name, ns = ns)
        self.server = None
        self.clients = []

class Parameter(Resource):
    def __init__(self, name, ptype, value, ns = "/"):
        Resource.__init__(self, name, ns = ns)
        self.type = ptype
        self.value = value

class ConditionalParameter(Parameter):
    def __init__(self, name, ptype, value, conditions, ns = "/"):
        Parameter.__init__(self, name, ptype, value, ns = ns)
        self.conditions = conditions    # if/unless conditions to launch


class ResourceGraph(object):
    def __init__(self, resources = None, remaps = None, collisions = None):
        self.resources = resources if not resources is None else {}
        self.remaps = remaps if not remaps is None else {}
        self.collisions = collisions if not collisions is None else {}

    @property
    def n_collisions(self):
        n = 0
        for name, values in self.collisions.iteritems():
            n += len(values)
        return n

    def get_node(self, name):
        return self._get(name, Node)

    def get_param(self, name):
        return self._get(name, Parameter)

    def _get(self, name, cls):
        r = self.resources.get(name)
        if r is None or isinstance(r, cls):
            return r
        rs = self.collisions.get(name)
        if rs:
            for r in rs:
                if isinstance(r, cls):
                    return r
        return None

    def register(self, resource):
        resource.name = self.remaps.get(resource.name, resource.name)
        if resource.name in self.resources:
            self._collision(resource.name, resource)
        else:
            self.resources[resource.name] = resource

    def remap(self, source, target):
        self.remaps[source] = target

    def child(self):
        return ResourceGraph(resources = self.resources,
                             remaps = dict(self.remaps),
                             collisions = self.collisions)

    def _collision(self, name, resource):
        if not name in self.collisions:
            self.collisions[name] = []
        self.collisions[name].append(resource)

class ResourceManager(object):
    def __init__(self, enabled = None, disabled = None, conditional = None):
        self.enabled = ResourceGraph() if enabled is None else enabled
        self.disabled = ResourceGraph() if disabled is None else disabled
        self.conditional = ResourceGraph() if conditional is None \
                                           else conditional

    def child(self):
        return ResourceManager(enabled = self.enabled.child(),
                              disabled = self.disabled.child(),
                              conditional = self.conditional.child())


###
# launch file analyser

class LaunchFileAnalyser(object):
    def __init__(self, environment, finder):
    # public:
        self.visited_files = {}
        self.top_launch = None
        self.resources = ResourceManager()
    # private:
        self._base_scope = LaunchScope(None, None, environment, finder,
                                       self.resources)
        self._scope = None

    _ALL_TAGS = ("node", "machine", "include", "remap", "env", "param",
                 "rosparam", "group", "test", "arg")

    def analyse(self, launch_file, args = None, ns = "/"):
        if launch_file in self.visited_files:
            # TODO detect cycles but allow multiple inclusions
            print "WARNING:", launch_file, "is included multiple times"
            # return self.visited_files[launch_file]
        launch = LaunchFile(launch_file, args = args, ns = ns)
        self._scope = self._base_scope.child_scope(ns = ns)
        self._scope.launch = launch
        self.visited_files[launch_file] = launch
        self.top_launch = self.top_launch or launch
        if os.path.isfile(launch_file):
            xml_root = ET.parse(launch_file).getroot()
            if not xml_root.tag == "launch":
                raise InvalidTagError(xml_root.tag)
            for tag in xml_root:
                self._analyse_tag(tag)
        else:
            self.top_launch.stats.n_unk_includes += 1
        return launch

    def _analyse_tag(self, tag):
        name = tag.tag
        if not name in LaunchFileAnalyser._ALL_TAGS:
            raise InvalidTagError(name)
        if name == "node":
            self._node_tag(tag)
        elif name == "include":
            self._include_tag(tag)
        elif name == "remap":
            self._remap_tag(tag)
        elif name == "arg":
            self._arg_tag(tag)
        elif name == "param":
            self._param_tag(tag)
        elif name == "rosparam":
            self._rosparam_tag(tag)
        elif name == "group":
            self._group_tag(tag)
        else:
            if name == "env":
                self._scope.launch.stats.n_env += 1
            elif name == "test":
                self._scope.launch.stats.n_test += 1
            elif name == "machine":
                self._scope.launch.stats.n_machines += 1

    _NODE_ATTRS = ("pkg", "type", "name", "args", "machine", "respawn",
                   "respawn_delay", "required", "ns", "clear_params",
                   "output", "cwd", "launch-prefix")

    def _node_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._NODE_ATTRS)
        name = attrib["name"]
        pkg = attrib["pkg"]
        ntype = attrib["type"]
        argv = attrib.get("args")
        ns = attrib.get("ns")
        # create a scope for the private namespace of the node
        self._scope = self._scope.node_scope(name, pkg, ntype,
                                             argv, ns, attrib["if"])
        for child in tag:
            name = child.tag
            if name == "remap":
                self._remap_tag(child)
            elif name == "param":
                self._param_tag(child)
            elif name == "rosparam":
                self._rosparam_tag(child)
            elif name == "env":
                self._scope.launch.stats.n_env += 1
            else:
                raise InvalidTagError(name)
        self._scope = self._scope.scope

        if attrib.get("machine"):
            self._scope.launch.stats.n_node_machines += 1
        if attrib.get("required"):
            self._scope.launch.stats.n_req_nodes += 1
        if attrib.get("respawn"):
            self._scope.launch.stats.n_respawn += 1
        if argv:
            self._scope.launch.stats.n_argv += 1

    _INCLUDE_ATTRS = ("file", "ns", "clear_params", "pass_all_args")

    def _include_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._INCLUDE_ATTRS)
        launch_file = attrib["file"]
        ns = attrib.get("ns")
        ns = _resolve_non_private_name(ns, self._scope.namespace) \
                if ns else self._scope.namespace
        args = dict(self._scope.launch.arguments) \
                if attrib.get("pass_all_args", False) else {}
        self._scope.launch.stats.n_pass_args += len(args)
        for child in tag:
            if child.tag == "arg":
                self._include_arg_tag(child, args)
            elif child.tag == "env":
                self._scope.launch.stats.n_env += 1
            else:
                raise InvalidTagError(child.name)
        if launch_file in self.visited_files:
            self.top_launch.stats.n_repeat_includes += 1
        scope = self._scope
        launch = self.analyse(launch_file, args = args, ns = ns)
        scope.launch.includes.append(launch)
        self._scope = scope
        self._scope.launch.stats.n_includes += 1

    def _include_arg_tag(self, tag, args):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ARG_ATTRS)
        name = attrib["name"]
        value = attrib.get("value", attrib.get("default"))
        condition = attrib["if"]
        if condition is True \
                and not (False in self._scope.conditions) \
                and not value is None:
            args[name] = value
        self._scope.launch.stats.n_pass_args += 1

    _REMAP_ATTRS = ("from", "to")

    def _remap_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._REMAP_ATTRS)
        self._scope.set_remap(attrib["from"], attrib["to"], attrib["if"])
        self._scope.launch.stats.n_remaps += 1

    _ARG_ATTRS = ("name", "default", "value", "doc")

    def _arg_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ARG_ATTRS)
        name = attrib["name"]
        if "value" in attrib and "default" in attrib:
            raise InvalidAttributeError("default")
        value = attrib.get("value", attrib.get("default"))
        self._scope.create_arg(name, value, "default" in attrib, attrib["if"])
        # stats below

    _PARAM_ATTRS = ("name", "value", "type", "textfile", "binfile", "command")

    def _param_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._PARAM_ATTRS)
        self._scope.launch.stats.n_params += 1

    _ROSPARAM_ATTRS = ("command", "file", "param", "ns", "subst_value")

    def _rosparam_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ROSPARAM_ATTRS)
        self._scope.launch.stats.n_rosparams += 1

    _GROUP_ATTRS = ("ns", "clear_params")

    def _group_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._GROUP_ATTRS)
        self._scope = self._scope.child_scope(attrib.get("ns"), attrib["if"])
        for child in tag:
            self._analyse_tag(child)
        self._scope = self._scope.scope

    def _attributes(self, attrib, possibilities):
        res = {"if": True}
        for key, text in attrib.iteritems():
            if key in possibilities:
                res[key] = sub_args.parse(text, self._scope, UNKNOWN)
            elif key == "if" or key == "unless":
                self._scope.launch.stats.n_if += 1
                value = sub_args.parse(text, self._scope, UNKNOWN)
                if value == UNKNOWN:
                    res["if"] = Condition.from_value(text, key == "unless", True)
                else:
                    res["if"] = Condition.from_value(value, key == "unless")
            else:
                raise InvalidAttributeError(key)
        return res


###############################################################################

class LaunchScope(object):
    def __init__(self, launch, scope, environment, finder, resources,
                 anonymous = None, ns = "/"):
        self.namespace = ns
        self.launch = launch    # launch file to which this belongs
        self.scope = scope      # parent scope
        self.children = []      # children scopes
        self.environment        = environment
        self.resource_finder    = finder
        self.resources          = resources
        self.anonymous          = anonymous or {}
        self.node = None        # associated node
        self.conditions = []    # unknown values on which this depends

    def get_arg(self, arg, default = None):
        value = self.launch.arguments.get(arg)
        if value is None:
            value = default
            self.launch.unknown.append(("arg", arg))
        return value

    def get_pkg(self, pkg, default = None):
        value = self.resource_finder.find_package(pkg)
        if value is None:
            value = default
            self.launch.unknown.append(("pkg", pkg))
        else:
            value = value.path
        return value

    def get_env(self, var, default = None):
        value = self.environment.get(var)
        if value is None:
            value = default
            self.launch.unknown.append(("env", var))
        return value

    def get_anon(self, name):
        if name in self.anonymous:
            return self.anonymous[name]
        else:
            value = rosgraph.names.anonymous_name(name)
            self.anonymous[name] = value
            return value

    def child_scope(self, ns = None, condition = True):
        ns = self._namespace(ns)
        scope = LaunchScope(self.launch, self, self.environment,
                            self.resource_finder, self.resources.child(),
                            anonymous = self.anonymous, ns = ns)
        self.children.append(scope)
        scope.conditions.extend(self.conditions)
        if not condition is True:
            scope.conditions.append(condition)
        return scope

    def node_scope(self, name, pkg, ntype, args, ns, condition):
        # creates a node under the current scope and also the
        # associated node scope.
        # we need to share the remap dict between node and scope
        ns = self._namespace(ns)
        remaps = dict(self.resources.enabled.remaps)
        scope = self.child_scope(condition = condition)
        if condition is False or False in self.conditions:
            node = Node(self.launch, name, pkg, ntype, args = args,
                        ns = ns, remaps = remaps)
            self.resources.disabled.register(node)
            self.launch.stats.n_not_nodes += 1
        elif self.conditions or isinstance(condition, Condition):
            node = ConditionalNode(self.launch, name, pkg, ntype,
                                   scope.conditions, args = args,
                                   ns = ns, remaps = remaps)
            self.resources.conditional.register(node)
            self.launch.stats.n_cond_nodes += 1
            self.launch.stats.cond_pkgs.add(pkg)
        else:
            node = Node(self.launch, name, pkg, ntype, args = args,
                        ns = ns, remaps = remaps)
            self.resources.enabled.register(node)
            self.launch.nodes.append(node)
            self.launch.stats.n_nodes += 1
            self.launch.stats.nodes.add(pkg + "/" + ntype)
            self.launch.stats.pkgs.add(pkg)
            if pkg == "nodelet":
                self.launch.stats.n_nodelets += 1
        scope.node = node
        scope.namespace = node.name
        return scope

    def create_arg(self, name, value, is_default, condition):
        if self.conditions or not condition is True:
            return
        args = self.launch.arguments
        prev = args.get(name)
        if value is None:
            args[name] = prev
        else:
            if not is_default or prev is None:
                args[name] = value
            self.launch.stats.n_set_args += 1
        self.launch.stats.n_args += 1

    def create_param(self, name, value, condition):
        pass

    def set_remap(self, source, target, condition):
        source = _resolve_non_private_name(source, self.namespace)
        target = _resolve_non_private_name(target, self.namespace)
        if condition is False:
            self.resources.disabled.remap(source, target)
        elif isinstance(condition, Condition):
            self.resources.conditional.remap(source, target)
        else:
            if False in self.conditions:
                self.resources.disabled.remap(source, target)
            elif self.conditions:
                self.resources.conditional.remap(source, target)
            else:
                self.resources.enabled.remap(source, target)
            if self.node:
                self.node.remaps[source] = target

    def _namespace(self, ns):
        if ns is None:
            return self.namespace
        return _resolve_non_private_name(ns, self.namespace)


###############################################################################
# HAROS plugin
###############################################################################

class InternalState(object):
    def __init__(self):
        self.gstats = Stats("global")
        self.fstats = {}

def pre_analysis():
    return InternalState()

def package_analysis(iface, package):
    for sf in package.source_files:
        if sf.language == "launch":
            launch_file = sf.get_path()
            env = {}
            analyser = LaunchFileAnalyser(env, iface)
            analyser.analyse(launch_file)

            if not launch_file in iface.state.fstats:
                stats = analyser.top_launch.stats
                iface.state.fstats[launch_file] = stats
                for ref in analyser.top_launch.unknown:
                    if ref[0] == "env":
                        stats.n_unk_env += 1
                    elif ref[0] == "arg":
                        stats.n_unk_args += 1
                    elif ref[0] == "pkg":
                        stats.unk_pkgs.add(ref[1])
                        print "[LAUNCH] unknown package", ref[1]

            launches = analyser.top_launch.includes
            while launches:
                launch = launches[0]
                iface.state.fstats[launch.name] = None
                launches.extend(launch.includes)
                del launches[0]

def post_analysis(iface):
    depends = {}
    rows = [Stats._CSV_HEADERS]
    for lf, stats in iface.state.fstats.iteritems():
        if stats:
            iface.state.gstats.merge(stats)
            rows.append(stats.to_csv())
            depends[lf] = stats.all_pkg_depends
    with open("launch_stats.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for row in rows:
            out.writerow(row)
    iface.export_file("launch_stats.csv")
    with open("launch_pkg_depends.txt", "w") as f:
        for lf, deps in depends.iteritems():
            f.write(lf + ";" + ";".join(deps) + "\n")
    iface.export_file("launch_pkg_depends.txt")
    print "[LAUNCH] considered", len(iface.state.fstats), "launch files"



###############################################################################

class Stats(object):
    def __init__(self, name = ""):
        self.name           = name
        self.n_args         = 0
        self.n_set_args     = 0
        self.n_unk_args     = 0
        self.nodes          = set()
        self.n_nodes        = 0
        self.n_cond_nodes   = 0
        self.n_not_nodes    = 0
        self.n_nodelets     = 0
        self.n_params       = 0
        self.n_rosparams    = 0
        self.n_machines     = 0
        self.n_node_machines    = 0
        self.n_req_nodes    = 0
        self.n_respawn      = 0
        self.n_argv         = 0
        self.n_includes     = 0
        self.n_repeat_includes  = 0
        self.n_reset_param  = 0 # TODO
        self.n_unknown      = 0
        self.n_unk_env      = 0
        self.pkgs           = set()
        self.cond_pkgs      = set()
        self.unk_pkgs       = set()
        self.n_remaps       = 0
        self.n_pass_args    = 0
        self.n_env          = 0
        self.n_if           = 0
        self.n_test         = 0
        self.n_unk_includes = 0

    @property
    def n_packages(self):
        return len(self.pkgs)

    @property
    def n_cond_packages(self):
        return len(self.cond_pkgs)

    @property
    def all_pkg_depends(self):
        return self.pkgs | self.cond_pkgs

    def merge(self, other):
        self.n_args += other.n_args
        self.n_set_args += other.n_set_args
        self.n_unk_args += other.n_unk_args
        self.nodes.update(other.nodes)
        self.n_nodes += other.n_nodes
        self.n_cond_nodes += other.n_cond_nodes
        self.n_not_nodes += other.n_not_nodes
        self.n_nodelets += other.n_nodelets
        self.n_params += other.n_params
        self.n_rosparams += other.n_rosparams
        self.n_machines += other.n_machines
        self.n_node_machines += other.n_node_machines
        self.n_req_nodes += other.n_req_nodes
        self.n_respawn += other.n_respawn
        self.n_argv += other.n_argv
        self.n_includes += other.n_includes
        self.n_repeat_includes += other.n_repeat_includes
        self.n_reset_param += other.n_reset_param
        self.n_unknown += other.n_unknown
        self.n_unk_env += other.n_unk_env
        self.pkgs.update(other.pkgs)
        self.cond_pkgs.update(other.cond_pkgs)
        self.unk_pkgs.update(other.unk_pkgs)
        self.n_remaps += other.n_remaps
        self.n_pass_args += other.n_pass_args
        self.n_env += other.n_env
        self.n_if += other.n_if
        self.n_test += other.n_test
        self.n_unk_includes += other.n_unk_includes

    _CSV_HEADERS = ["Name", "Arg", "Arg Value", "Param", "ROSParam", "Nodes",
                    "Unique Nodes", "Nodelets", "Conditional Nodes",
                    "Disabled Nodes", "Required Nodes", "Respawn Nodes",
                    "Remote Nodes", "Node Args", "Machines", "Tests", "Packages",
                    "Conditional Packages", "Remaps", "Conditionals", "Includes",
                    "Repeated Includes", "Unknown Includes", "Include Args",
                    "Env. Sets", "Unknown References", "Unknown Arg", "Unknown Env"]

    def to_csv(self):
        return [self.name, self.n_args, self.n_set_args, self.n_params,
                self.n_rosparams, self.n_nodes, len(self.nodes), self.n_nodelets,
                self.n_cond_nodes, self.n_not_nodes, self.n_req_nodes,
                self.n_respawn, self.n_node_machines, self.n_argv,
                self.n_machines, self.n_test, len(self.pkgs), len(self.cond_pkgs),
                self.n_remaps, self.n_if, self.n_includes, self.n_repeat_includes,
                self.n_unk_includes, self.n_pass_args, self.n_env, self.unknown,
                self.n_unk_args, self.n_unk_env]
