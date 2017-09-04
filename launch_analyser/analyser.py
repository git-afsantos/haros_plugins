
###
# standard packages
import os
import xml.etree.ElementTree as ET

###
# internal packages
from haros_util.events import Event
import haros_util.ros_model as ROS
import launch_analyser.substitution_args as sub_args
from launch_analyser.launch_model import LaunchFile, LaunchScope, \
                                         LaunchResourceGraph, Condition


UNKNOWN = "<?>"


###############################################################################
# Launch File Analysis
###############################################################################

class LaunchFileAnalyser(object):
    def __init__(self, environment, finder, resources = None):
    # public:
        self.visited_files = {}
        self.top_launch = None
        self.resources = LaunchResourceGraph(enabled = resources)
        self.stats = Statistics()
        self.onNode = Event()
    # private:
        self._base_scope = LaunchScope(None, None, environment, finder,
                                       self.resources)
        self._scope = None

    _ALL_TAGS = ("node", "machine", "include", "remap", "env", "param",
                 "rosparam", "group", "test", "arg", "master")

    def analyse(self, launch_file, args = None, ns = "/"):
        self.stats.name = launch_file
        launch = self._analyse(launch_file, args = args, ns = ns)
        launch = launch.merge()
        self.stats.gather(launch, self.resources)
        return launch

    def _analyse(self, launch_file, args = None, ns = "/"):
        if launch_file in self.visited_files:
            # TODO detect cycles but allow multiple inclusions
            #print "WARNING:", launch_file, "is included multiple times"
            # return self.visited_files[launch_file]
            pass
        launch = LaunchFile(launch_file, args = args, ns = ns)
        self._scope = self._base_scope.child_scope(ns = ns)
        self._scope.launch = launch
        self.visited_files[launch_file] = launch
        self.top_launch = self.top_launch or launch
        if os.path.isfile(launch_file):
            try:
                xml_root = ET.parse(launch_file).getroot()
                if not xml_root.tag == "launch":
                    raise InvalidTagError(xml_root.tag)
                for tag in xml_root:
                    self._analyse_tag(tag)
            except (ET.ParseError, InvalidTagError, InvalidAttributeError) as e:
                launch.valid = False
        else:
            self.stats.unknown_includes += 1
            launch.valid = False
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
                self.stats.env_tags += 1
            elif name == "test":
                self.stats.test_tags += 1
            elif name == "machine":
                self.stats.machine_tags += 1

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
        nodelet = False

        if argv:
            argv = argv.split()
        if pkg == "nodelet":
            if argv[0] == "load":
                nodelet = True
                ss = argv[1].split("/")
                pkg = ss[0]
                ntype = ss[1]
                argv = argv[2:]

        # create a scope for the private namespace of the node
        self._scope = self._scope.node_scope(name, pkg, ntype, nodelet,
                                             argv, ns, attrib["if"])
        for child in tag:
            name = child.tag
            if name == "remap":
                self._remap_tag(child)
            elif name == "param":
                self._param_tag(child, private = True)
            elif name == "rosparam":
                self._rosparam_tag(child, private = True)
            elif name == "env":
                self.stats.env_tags += 1
            else:
                raise InvalidTagError(name)

        self.onNode(self._scope.node)
        self._scope = self._scope.scope

        self.stats.node_tags += 1
        if nodelet:
            self.stats.nodelets += 1
        if attrib.get("machine"):
            self.stats.remote_nodes += 1
        if attrib.get("required"):
            self.stats.required_nodes += 1
        if attrib.get("respawn"):
            self.stats.respawn_nodes += 1
        if argv:
            self.stats.argv_nodes += 1

    _INCLUDE_ATTRS = ("file", "ns", "clear_params", "pass_all_args")

    def _include_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._INCLUDE_ATTRS)
        launch_file = attrib["file"]
        if launch_file == UNKNOWN:
            return
        ns = attrib.get("ns")
        ns = ROS.resolve_name(ns, self._scope.namespace) \
                if ns else self._scope.namespace
        args = dict(self._scope.launch.arguments) \
                if attrib.get("pass_all_args", False) else {}
        for child in tag:
            if child.tag == "arg":
                self._include_arg_tag(child, args)
            elif child.tag == "env":
                self.stats.env_tags += 1
            else:
                raise InvalidTagError(child.name)
        self.stats.include_tags += 1
        self.stats.passed_args += len(args)
        if launch_file in self.visited_files:
            self.stats.repeat_includes += 1
        scope = self._scope
        launch = self._analyse(launch_file, args = args, ns = ns)
        scope.launch.includes.append(launch)
        self._scope = scope

    def _include_arg_tag(self, tag, args):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ARG_ATTRS)
        name = attrib["name"]
        value = attrib.get("value", attrib.get("default"))
        condition = attrib["if"]
        if condition is True \
                and not (False in self._scope.conditions) \
                and not value is None:
            args[name] = value

    _REMAP_ATTRS = ("from", "to")

    def _remap_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._REMAP_ATTRS)
        self._scope.set_remap(attrib["from"], attrib["to"], attrib["if"])
        self.stats.remap_tags += 1

    _ARG_ATTRS = ("name", "default", "value", "doc")

    def _arg_tag(self, tag):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ARG_ATTRS)
        name = attrib["name"]
        if "value" in attrib and "default" in attrib:
            raise InvalidAttributeError("default")
        value = attrib.get("value", attrib.get("default"))
        self._scope.create_arg(name, value, "default" in attrib, attrib["if"])
        self.stats.arg_tags += 1
        if value is None:
            self.stats.unbound_args += 1

    _PARAM_ATTRS = ("name", "value", "type", "textfile", "binfile", "command")

    def _param_tag(self, tag, private = False):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._PARAM_ATTRS)
        self.stats.param_tags += 1
        name = attrib["name"]
        ptype = attrib.get("type")
        if attrib.get("command"):
            value = attrib.get("command") # TODO
            self.stats.cmd_params += 1
        elif attrib.get("textfile") or attrib.get("binfile"):
            value = attrib.get("textfile", attrib.get("binfile"))
            self.stats.file_params += 1
        else:
            value = attrib["value"]
        self._scope.create_param(name, ptype, value, attrib["if"])

    _ROSPARAM_ATTRS = ("command", "file", "param", "ns", "subst_value")

    def _rosparam_tag(self, tag, private = False):
        attrib = self._attributes(tag.attrib, LaunchFileAnalyser._ROSPARAM_ATTRS)
        self.stats.rosparam_tags += 1
        cmd = attrib.get("command", "load")
        if cmd == "load" and attrib.get("file"):
            self.stats.file_params += 1

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
                self.stats.if_attributes += 1
                value = sub_args.parse(text, self._scope, UNKNOWN)
                if value == UNKNOWN:
                    res["if"] = Condition.from_value(text, key == "unless", True)
                else:
                    res["if"] = Condition.from_value(value, key == "unless")
            else:
                raise InvalidAttributeError(key)
        return res


###############################################################################
# Launch File Statistics
###############################################################################

class Statistics(object):
    def __init__(self, name = ""):
        self.name               = name
    # <arg> related
        self.arg_tags           = 0
        self.unbound_args       = 0
    # <node> related
        self.node_tags          = 0
        self.unique_nodes       = 0
        self.conditional_nodes  = 0
        self.disabled_nodes     = 0
        self.nodelets           = 0
        self.remote_nodes       = 0
        self.required_nodes     = 0
        self.respawn_nodes      = 0
        self.argv_nodes         = 0
    # <param> related
        self.param_tags         = 0
        self.rosparam_tags      = 0
        self.file_params        = 0
        self.cmd_params         = 0
        self.reset_params       = 0 # TODO
    # <include> related
        self.include_tags       = 0
        self.repeat_includes    = 0
        self.passed_args        = 0
    # <tag> related
        self.remap_tags         = 0
        self.machine_tags       = 0
        self.env_tags           = 0
        self.test_tags          = 0
    # unknown references
        self.unknown_refs       = 0
        self.unknown_args       = 0
        self.unknown_envs       = 0
        self.unknown_pkgs       = 0
        self.unknown_includes   = 0
    # other statistics
        self.pkg_depends        = 0
        self.conditional_pkgs   = 0 # TODO
        self.if_attributes      = 0
        self.env_variables      = 0

    def gather(self, launch, resources):
    # <node> related
        self.unique_nodes       = len(set([n.package + "/" + n.node_type \
                                           for n in launch.nodes]))
        self.conditional_nodes  = resources.conditional_nodes(count = True)
        self.disabled_nodes     = resources.disabled_nodes(count = True)
    # unknown references
        for ur in launch.unknown:
            self.unknown_refs   += 1
            if ur[0] == "arg":
                self.unknown_args += 1
            elif ur[0] == "env":
                self.unknown_envs += 1
            elif ur[0] == "pkg":
                self.unknown_pkgs += 1
    # other statistics
        self.pkg_depends        = len(launch.pkg_depends)
        self.env_variables      = len(launch.env_depends)

    CSV_HEADERS = ["Launch File", "Arg Tags", "Unbound Args",
                    "Node Tags", "Unique Nodes", "Nodelets",
                    "Conditional Nodes", "Disabled Nodes", "Required Nodes",
                    "Respawning Nodes", "Remote Nodes", "Nodes With Args"
                    "Param Tags", "ROSParam Tags", "File Params",
                    "Command Params", "Include Tags", "Repeated Includes",
                    "Passed Include Args", "Remap Tags", "Machine Tags",
                    "Env Tags", "Test Tags", "Unknown References",
                    "Unknown Args", "Unknown Envs", "Unknown Packages",
                    "Unknown Includes", "Package Dependencies",
                    "Conditional Attributes", "Environment Variables"]

    def to_csv(self):
        return [self.name, self.arg_tags, self.unbound_args,
                self.node_tags, self.unique_nodes, self.nodelets,
                self.conditional_nodes, self.disabled_nodes,
                self.required_nodes, self.respawn_nodes,
                self.remote_nodes, self.argv_nodes,
                self.param_tags, self.rosparam_tags, self.file_params,
                self.cmd_params, self.include_tags, self.repeat_includes,
                self.passed_args, self.remap_tags, self.machine_tags,
                self.env_tags, self.test_tags, self.unknown_refs,
                self.unknown_args, self.unknown_envs, self.unknown_pkgs,
                self.unknown_includes, self.pkg_depends,
                self.if_attributes, self.env_variables]

    def aggregate(self, other):
    # <arg> related
        self.arg_tags           += other.arg_tags
        self.unbound_args       += other.unbound_args
    # <node> related
        self.node_tags          += other.node_tags
        self.unique_nodes       += other.unique_nodes
        self.conditional_nodes  += other.conditional_nodes
        self.disabled_nodes     += other.disabled_nodes
        self.nodelets           += other.nodelets
        self.remote_nodes       += other.remote_nodes
        self.required_nodes     += other.required_nodes
        self.respawn_nodes      += other.respawn_nodes
        self.argv_nodes         += other.argv_nodes
    # <param> related
        self.param_tags         += other.param_tags
        self.rosparam_tags      += other.rosparam_tags
        self.file_params        += other.file_params
        self.cmd_params         += other.cmd_params
        self.reset_params       += other.reset_params
    # <include> related
        self.include_tags       += other.include_tags
        self.repeat_includes    += other.repeat_includes
        self.passed_args        += other.passed_args
    # <tag> related
        self.remap_tags         += other.remap_tags
        self.machine_tags       += other.machine_tags
        self.env_tags           += other.env_tags
        self.test_tags          += other.test_tags
    # unknown references
        self.unknown_refs       += other.unknown_refs
        self.unknown_args       += other.unknown_args
        self.unknown_envs       += other.unknown_envs
        self.unknown_pkgs       += other.unknown_pkgs
        self.unknown_includes   += other.unknown_includes
    # other statistics
        self.pkg_depends        += other.pkg_depends
        self.conditional_pkgs   += other.conditional_pkgs
        self.if_attributes      += other.if_attributes
        self.env_variables      += other.env_variables


###############################################################################
# Exceptions
###############################################################################

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
