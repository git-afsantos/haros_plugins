
###
# standard packages
import yaml

###
# third-party packages
import rosgraph.names

###
# internal packages
import haros_util.ros_model as ROS


###
# unknown results

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

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        if self.negate:
            return "not " + self.value
        return self.value


###############################################################################
# ROS Launch - Concepts
###############################################################################

class LaunchFile(object):
    def __init__(self, name, args = None, ns = "/"):
        self.name = name
        self.arguments = args or {} # all evaluated <arg>
        self.nodes = []             # all evaluated <node>
        self.includes = []          # all evaluated <include>
        self.unknown = []           # unknown references
        self.valid = True
        self.pkg_depends = set()
        self.env_depends = set()

    def merge(self):
        merged = LaunchFile(self.name)
        launches = [self]
        while launches:
            launch = launches[0]
            launches.extend(launch.includes)
            merged.valid = merged.valid and launch.valid
            merged.nodes.extend(launch.nodes)
            merged.includes.extend(launch.includes)
            merged.unknown.extend(launch.unknown)
            merged.pkg_depends.update(launch.pkg_depends)
            merged.env_depends.update(launch.env_depends)
            for name, value in launch.arguments.iteritems():
                while name in merged.arguments:
                    name += "$"
                merged.arguments[name] = value
            del launches[0]
        return merged


class LaunchNode(ROS.Node):
    def __init__(self, launch, name, pkg, ntype, nodelet = False,
                 args = None, ns = "/", remaps = None, conditions = None):
        ROS.Node.__init__(self, name, pkg, ntype, nodelet = nodelet,
                          args = args, remaps = remaps, ns = ns)
        self.launch = launch
        self.conditions = conditions if not conditions is None else ()
                          # if/unless conditions to launch


class LaunchParameter(ROS.Parameter):
    def __init__(self, name, ptype, value, ns = "/", private_ns = "",
                 node_param = False, scope_private = False, conditions = None):
        given_name = name
        if node_param and name[0] != "~" and name[0] != "/":
            name = "~" + name
        if isinstance(value, basestring):
            value = self._convert_value(value, ptype)
        ROS.Parameter.__init__(self, name, ptype, value, ns, private_ns)
        self.given_name = given_name
        self.node_param = node_param
        self.scope_private = scope_private # declared with "~" outside of <node>
        self.conditions = conditions if not conditions is None else ()
                          # if/unless conditions to launch


    # as seen in roslaunch code, sans a few details
    def _convert_value(value, ptype):
        if ptype is None:
            # attempt numeric conversion
            try:
                if "." in value:
                    return float(value)
                else:
                    return int(value)
            except ValueError as e:
                pass
            # bool
            lval = value.lower()
            if lval == "true" or lval == "false":
                return self._convert_value(value, "bool")
            # string
            return value
        elif ptype == "str" or ptype == "string":
            return value
        elif ptype == "int":
            return int(value)
        elif ptype == "double":
            return float(value)
        elif ptype == "bool" or ptype == "boolean":
            value = value.lower().strip()
            if value == "true" or value == "1":
                return True
            elif value == "false" or value == "0":
                return False
            raise ValueError("{} is not a '{}' type".format(value, ptype))
        elif ptype == "yaml":
            try:
                return yaml.load(value)
            except yaml.parser.ParserError as e:
                raise ValueError(e)
        else:
            raise ValueError("Unknown type '{}'".format(ptype))


class LaunchResourceGraph(object):
    def __init__(self, enabled = None, disabled = None, conditional = None):
        self.enabled = ROS.ResourceGraph() if enabled is None else enabled
        self.disabled = ROS.ResourceGraph() if disabled is None else disabled
        self.conditional = [] if conditional is None else conditional

    def enabled_nodes(self, count = False):
        if count:
            c = 0
            for _, n in self.enabled.resources.iteritems():
                if isinstance(n, LaunchNode):
                    c += 1
            return c
        return [n for _, n in self.enabled.resources.iteritems() \
                  if isinstance(n, LaunchNode)]

    def disabled_nodes(self, count = False):
        if count:
            c = 0
            for _, n in self.disabled.resources.iteritems():
                if isinstance(n, LaunchNode):
                    c += 1
            return c
        return [n for _, n in self.disabled.resources.iteritems() \
                  if isinstance(n, LaunchNode)]

    def conditional_nodes(self, count = False):
        if count:
            c = 0
            for n in self.conditional:
                if isinstance(n, LaunchNode):
                    c += 1
            return c
        return [n for n in self.conditional if isinstance(n, LaunchNode)]

    def child(self):
        return LaunchResourceGraph(enabled = self.enabled.child(),
                                   disabled = self.disabled.child(),
                                   conditional = list(self.conditional))


###############################################################################
# ROS Launch - Launch Scope
###############################################################################

class LaunchScope(object):
    def __init__(self, launch, parent, environment, finder, resources,
                 anonymous = None, ns = "/"):
        self.namespace = ns
        self.launch = launch    # launch file to which this belongs
        self.scope = parent     # parent scope
        self.children = []      # children scopes
        self.environment        = environment
        self.resource_finder    = finder
        self.resources          = resources
        self.anonymous          = anonymous or {}
        self.node = None        # associated node
        self.conditions = []    # unknown values on which this depends
        self.parameters = []    # private, to be passed on to nodes

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
        self.launch.pkg_depends.add(pkg)
        return value

    def get_env(self, var, default = None):
        value = self.environment.get(var)
        if value is None:
            value = default
            self.launch.unknown.append(("env", var))
        self.launch.env_depends.add(var)
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
        scope.parameters.extend(self.parameters)
        if not condition is True:
            scope.conditions.append(condition)
        return scope

    def node_scope(self, name, pkg, ntype, args, nodelet, ns, condition):
        # creates a node under the current scope and also the
        # associated node scope.
        # TODO check if remaps fict really needs to be a copy
        #       instead of using the dict from child scope
        scope = self.child_scope(condition = condition)
        node = LaunchNode(self.launch, name, pkg, ntype, nodelet = nodelet,
                          args = args, ns = self._namespace(ns),
                          remaps = dict(self.resources.enabled.remaps),
                          conditions = scope.conditions)
        scope.node = node
        if condition is False or False in self.conditions:
            self.resources.disabled.register(node)
        elif self.conditions or isinstance(condition, Condition):
            self.resources.conditional.append(node)
        else:
            self.resources.enabled.register(node)
            self.launch.nodes.append(node)
            for param in self.parameters:
                conditions = list(param.conditions)
                conditions.extend(self.conditions)
                param = LaunchParameter(param.given_name, param.type,
                                        param.value, ns = scope.namespace,
                                        private_ns = scope.private_ns,
                                        node_param = True,
                                        scope_private = True,
                                        conditions = conditions)
                if False in conditions:
                    self.resources.disabled.register(param)
                elif conditions:
                    self.resources.conditional.append(param)
                else:
                    self.resources.enabled.register(param)
        self.launch.pkg_depends.add(pkg)
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

    def create_param(self, name, ptype, value, condition):
        conditions = list(self.conditions)
        if not condition is True:
            conditions.append(condition)
        param = LaunchParameter(name, ptype, value, ns = self.namespace,
                                private_ns = self.private_ns,
                                node_param = not self.node is None,
                                conditions = conditions)
        if not self.node and name[0] == "~":
            self.parameters.append(param)
        else:
            if False in conditions:
                self.resources.disabled.register(param)
            elif conditions:
                self.resources.conditional.append(param)
            else:
                self.resources.enabled.register(param)

    def set_remap(self, source, target, condition):
        source = ROS.resolve_name(source, self.namespace, self.private_ns)
        target = ROS.resolve_name(target, self.namespace, self.private_ns)
        if condition is False:
            self.resources.disabled.remap(source, target)
        elif isinstance(condition, Condition):
            self.resources.conditional.append((source, target))
        else:
            if False in self.conditions:
                self.resources.disabled.remap(source, target)
            elif self.conditions:
                self.resources.conditional.append((source, target))
            else:
                self.resources.enabled.remap(source, target)
                if self.node:
                    self.node.remaps[source] = target

    @property
    def private_ns(self):
        if self.node:
            return self.node.full_name
        return self.namespace

    def _namespace(self, ns):
        if ns is None:
            return self.namespace
        return ROS.resolve_name(ns, self.namespace)
