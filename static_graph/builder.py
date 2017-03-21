
###
# standard packages
import os
import sys

###
# third-party packages
import clang.cindex as clang

###
# internal packages
import haros_util.ros_model as ROS
from clang_plugin.cpp_model import CppGlobalScope, CppFunctionCall
from cmake_analyser.analyser import CMakeAnalyser
from launch_analyser.analyser import LaunchFileAnalyser

###
# constants
LIB_PATH        = "/usr/lib/llvm-3.8/lib"
CWS             = "/home/andre/catkin_ws"
CWS_PREFIX      = "/home/andre/catkin_ws/"
DB_PATH         = "/home/andre/catkin_ws/build"
STD_INCLUDES    = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
CATKIN_INCLUDE  = "/home/andre/catkin_ws/devel/include"
USR_INCLUDE     = "/usr/include/"
EIGEN_INCLUDE   = "/usr/include/eigen3"
MAGICK_INCLUDE  = "/usr/include/ImageMagick"

###############################################################################
# AST Analysis
###############################################################################

def setup():
    sys.setrecursionlimit(2000)
    clang.Config.set_library_path(LIB_PATH)


class ConfigurationBuilder(object):
    def __init__(self):
    # public:
        self.environment = dict(os.environ)
        self.launch_files = set()
        self.unknown_packages = set()
    # private:
        self._config = None
        self._node = None
        self._db = clang.CompilationDatabase.fromDirectory(DB_PATH)
        self._exe = {}  # package -> (name -> [file])
        self._gen = {}  # cache to generate topics and services for nodes
        CppFunctionCall.onInstance.sub(self._onFunctionCall)

    def with_package(self, package, finder):
        """Register executables from a package's CMakeLists.
            This has to be called *before* 'from_launch'.
        """
        executables = {}
        self._exe[package.name] = executables
        parser = CMakeAnalyser({
            "catkin_INCLUDE_DIRS":      CATKIN_INCLUDE,
            "Boost_INCLUDE_DIRS":       USR_INCLUDE,
            "Eigen_INCLUDE_DIRS":       EIGEN_INCLUDE,
            "ImageMagick_INCLUDE_DIRS": MAGICK_INCLUDE,
            "PROJECT_SOURCE_DIR":       package.path
        }, self.environment, finder)
        parser.analyse(os.path.join(package.path, "CMakeLists.txt"))
        for exe in parser.data["executables"] + parser.data["libraries"]:
            executables[exe[0]] = exe[1]

    def from_launch(self, iface, launch_file):
        """Build a Configuration, given a launch file.
            Control starts with the launch analyser.
            Whenever a node is parsed, control reverts to the builder.
            If the node is not cached, control is given to the clang parser.
            Whenever a function call is parsed, control reverts to the builder.
            If a primitive is detected, register the respective resource.
            Control reverts to the clang parser until the end.
            Control reverts to the launch analyser until the end.
        """
        path = launch_file.get_path()
        if path in self.launch_files:
            return None
        self.launch_files.add(path)
        config = ROS.Configuration(path.replace(CWS_PREFIX, ""),
                                   self.environment)
        self._config = config
        analyser = LaunchFileAnalyser(self.environment, iface,
                                      resources = config.resources)
        analyser.onNode.sub(self._onNode)
    # ----- analyse will eventually call the event callbacks ------------------
        merged_launch = analyser.analyse(path)
    # -------------------------------------------------------------------------
        if not merged_launch.valid:
            return None
        for ref in merged_launch.unknown:
            if ref[0] == "pkg":
                self.unknown_packages.add(ref[1])
        config.pkg_depends.update(merged_launch.pkg_depends)
        config.valid = not bool(merged_launch.unknown)
        self._config = None
        return config


    def _onNode(self, node):
        if not node.reference in self._gen:
            return
        else:
            files = self._exe.get(node.package, {}).get(node.node_type, [])
            for f in files:
                if _clang_node(iface.state.compile_db, f):
                    pass
                    # if ast in cache, use it
                    # if not, analyse source
        for gen in self._gen.get(node.reference, []):
            pass

    def _onFunctionCall(self, call):
        return



class TopicGenerator(object):
    def __init__(self, name, namespace):
        self.name = ROS.resolve_name(name, namespace)

    def generate(self, node, resources):
        topic = resources.get_topic(self.name, remap = True)
        if topic is None:
            topic = ROS.Topic(name)


class ServiceGenerator(object):
    def __init__(self, name):
        self.name = name








def node_analysis(unit, package, state):
    global_scope = CppGlobalScope()
    for node in unit.cursor.get_children():
        if node.location.file and node.location.file.name.startswith(CWS):
            global_scope.add_from_cursor(node)
    if not package.id in state.pkg_metrics:
        state.pkg_metrics[package.id] = collectors.GlobalCollector()
    state.pkg_metrics[package.id].collect_from_global_scope(global_scope)
    return state.metrics.collect_from_global_scope(global_scope, store = True)


###############################################################################
# Function Collector
###############################################################################

class FunctionCollector(object):
    def __init__(self, function):
        assert isinstance(function, CppFunction)
        self.function           = function.name
        self.subscribe          = []
        self.advertise          = []
        self.publish            = []
        self.advertise_service  = []
        self.service_client     = []
        self.spin_rate          = []
        self.sleep              = []
        self.ros_parameters     = []
        self.param_vars         = set()
        self.modified_params    = set()
        self.set_param_count    = 0
        self.function_set       = set()
        self.function_calls     = 0
        self.spinner_vars       = 0
        for statement in function.body.body:
            self._collect(statement)

    def _collect(self, statement, nesting = 0, variable = None):
        if isinstance(statement, LazyCppEntity):
            statement = statement.evaluate()
        if isinstance(statement, CppVariable):
            self._from_variable(statement, nesting)
        elif isinstance(statement, CppFunctionCall):
            self._from_call(statement, nesting, variable)
        elif isinstance(statement, CppControlFlow):
            # TODO reduce nesting on "else if"
            for branch in statement.branches:
                for stmt in branch[1].body:
                    self._collect(stmt, nesting = nesting + 1, variable = variable)
        elif isinstance(statement, CppBlock):
            for stmt in statement.body:
                self._collect(stmt, nesting, variable)
        elif isinstance(statement, CppStatement):
            if statement.value:
                self._collect(statement.value, nesting, variable)
        elif isinstance(statement, CppOperator):
            if statement.name == "=" or statement.name == "[op]":
                for arg in statement.arguments:
                    self._collect(arg, nesting, variable)

    def _from_variable(self, variable, nesting):
        global pubsub_var
        name = variable.name
        if variable.result == "ros::Rate" or variable.result == "ros::WallRate" \
                or variable.result == "ros::Duration" \
                or variable.result == "ros::WallDuration":
            self.spinner_vars += 1
            if isinstance(variable.value, CppFunctionCall):
                value = None
                svar = ()
                if len(variable.value.arguments) > 0:
                    value = variable.value.arguments[0]
                if isinstance(value, CppExpression):
                    svar = value.variables
                o = SpinRateTuple(value, name, self.function, variable.line,
                                  any(v in self.param_vars for v in svar),
                                  variable.result in ("ros::Duration", "ros::WallDuration"),
                                  variable.result in ("ros::WallRate", "ros::WallDuration"))
                self.spin_rate.append(o)
        elif variable.result == "ros::Publisher" \
                or variable.result == "ros::Subscriber" \
                or variable.result == "ros::ServiceServer" \
                or variable.result == "ros::ServiceClient":
            if variable.result == "ros::Publisher" or variable.result == "ros::Subscriber":
                pubsub_var += 1
            self._collect(variable.value, nesting, variable = name)
        elif isinstance(variable.value, CppFunctionCall) \
                and variable.value.name == "param":
            self._from_call(variable.value, nesting, variable = name)

    def _from_call(self, call, nesting, variable = None):
        global advertise_count
        global subscribe_count
        # TODO a function map seems better by now...
        name = call.name
        if name == "operator=":
            if len(call.arguments) >= 2 and isinstance(call.arguments[1], CppFunctionCall):
                self._from_call(call.arguments[1], nesting,
                                variable = call.arguments[0].name)
            elif len(call.arguments) < 2:
                print "[CLANG]", name, call.arguments
            return
        self.function_set.add(name)
        self.function_calls += 1
        if name == "advertise" and call.result == "ros::Publisher":
            advertise_count += 1
            self._advertise_call(call, nesting, variable = variable)
        elif name == "subscribe" and call.result == "ros::Subscriber":
            subscribe_count += 1
            self._subscribe_call(call, nesting, variable = variable)
        elif name == "publish" and call.method_of \
                and call.method_of.result == "ros::Publisher":
            o = PublishTuple(call.method_of.name, nesting,
                             self.function, call.line, call.scope)
            self.publish.append(o)
        elif name == "advertiseService" and call.result == "ros::ServiceServer":
            self._advertise_service_call(call, nesting, variable = variable)
        elif name == "serviceClient" and call.result == "ros::ServiceClient":
            self._service_client_call(call, nesting, variable = variable)
        elif name == "sleep" and call.method_of \
                and (call.method_of.result == "ros::Rate" \
                or call.method_of.result == "ros::Duration" \
                or call.method_of.result == "ros::WallDuration" \
                or call.method_of.result == "ros::WallRate"):
            self.sleep.append(call)
        elif name == "param":
            self._param_call(call, nesting, variable = variable)
        elif name == "getParam" or name == "getParamCached":
            self._get_param_call(call, nesting, variable = variable)
        elif name == "deleteParam" or name == "setParam":
            self.set_param_count += 1
            if call.arguments and isinstance(call.arguments[0], basestring):
                self.modified_params.add(call.arguments[0])
        else:
            for arg in call.arguments:
                self._collect(arg)


    def _advertise_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        topic       = None
        queue_size  = None
        latch       = False
        tvar        = ()
        qvar        = ()
        if nargs == 1:
            atype = ADV_TYPE_3
        else:
            assert nargs == 3 or nargs == 6
            atype = ADV_TYPE_1 if nargs == 3 else ADV_TYPE_2
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            latch       = call.arguments[-1]
            latch       = latch if isinstance(latch, bool) else None
            if isinstance(topic, CppExpression):
                tvar    = topic.variables
            if isinstance(queue_size, CppExpression):
                qvar    = queue_size.variables
        self.advertise.append(AdvertiseTuple(topic, queue_size,
                call.template, nesting, variable, self.function, call.line,
                atype, latch, any(v in self.param_vars for v in tvar),
                any(v in self.param_vars for v in qvar)))

    def _subscribe_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        topic       = None
        queue_size  = None
        callback    = None
        msg_type    = None
        hints       = False
        tvar        = ()
        qvar        = ()
        if nargs == 1:
            # assert call.arguments[0].result == "ros::SubscribeOptions"
            stype = SUB_TYPE_4
        else:
            # nargs == 4 (function) or nargs == 5 (method/boost)
            assert nargs == 4 or nargs == 5
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            cb          = call.arguments[2]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            if nargs == 4:
                stype = SUB_TYPE_2
            else:
                if isinstance(call.arguments[3], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    stype = SUB_TYPE_3
                else:
                    stype = SUB_TYPE_1
            hints = not isinstance(call.arguments[-1], CppDefaultArgument)
            if isinstance(topic, CppExpression):
                tvar = topic.variables
            if isinstance(queue_size, CppExpression):
                qvar = queue_size.variables
        self.subscribe.append(SubscribeTuple(topic, queue_size, callback,
                msg_type, nesting, variable, self.function,
                call.line, stype, hints,
                any(v in self.param_vars for v in tvar),
                any(v in self.param_vars for v in qvar)))

    def _advertise_service_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        service     = None
        callback    = None
        msg_type    = None
        events      = False
        var         = ()
        if nargs == 1:
            atype = ADV_SRV_TYPE_4
        else:
            # nargs == 2 (function) or nargs == 3 (method/boost)
            assert nargs == 2 or nargs == 3
            service = call.arguments[0]
            cb = call.arguments[1]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            events = "ros::ServiceEvent" in cb.result
            if nargs == 2:
                atype = ADV_SRV_TYPE_2
            else:
                if isinstance(call.arguments[2], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    atype = ADV_SRV_TYPE_3
                else:
                    atype = ADV_SRV_TYPE_1
            if isinstance(service, CppExpression):
                var = service.variables
        self.advertise_service.append(AdvertiseServiceTuple(service,
                callback, msg_type, nesting, variable, self.function, call.line,
                atype, events, any(v in self.param_vars for v in var)))

    def _service_client_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        service = None
        ctype = SRV_CLI_TYPE_1
        var = ()
        if nargs == 1:
            if "ros::ServiceClientOptions" in call.arguments[0].result:
                ctype = SRV_CLI_TYPE_2
        else:
            service = call.arguments[0]
            if isinstance(service, CppExpression):
                var = service.variables
        self.service_client.append(ServiceClientTuple(service,
                call.template, nesting, variable, self.function,
                call.line, ctype, any(v in self.param_vars for v in var)))

    def _param_call(self, call, nesting, variable = None):
        n = len(call.arguments)
        if n == 2:      # T param(std::string &key, T &default_val)
            result = call.result
        elif n == 3:    # void param(std::string &key, T &val, T &default_val)
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
        self.ros_parameters.append(ParamTuple(call.arguments[0],
                variable, True, result, self.function, call.line))
        if variable:
            self.param_vars.add(variable)

    def _get_param_call(self, call, nesting, variable = None):
        if len(call.arguments) >= 2:
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
            self.ros_parameters.append(ParamTuple(call.arguments[0],
                    variable, False, result, self.function, call.line))
            if variable:
                self.param_vars.add(variable)


    def _msg_type_from_callback(self, callback):
        type_string = callback.result
        type_string = type_string.split(None, 1)[1]
        if not (type_string[0] == "(" and type_string[-1] == ")"):
            print "[CLANG] Unknown message type:", type_string
            return type_string
        type_string = type_string[1:-1]
        is_const = type_string.startswith("const ")
        if is_const:
            type_string = type_string[6:]
        is_ref = type_string.endswith(" &")
        if is_ref:
            type_string = type_string[:-2]
        is_ptr = type_string.endswith("::ConstPtr")
        if is_ptr:
            type_string = type_string[:-10]
        else:
            is_ptr = type_string.endswith("ConstPtr")
            if is_ptr:
                type_string = type_string[:-8]
        return type_string



###############################################################################
# Function Collector
###############################################################################

def collect_functions(scope = None):
    global_scope = scope or CppGlobalScope.INSTANCE
    all_functions = list(global_scope.functions)
    for c in global_scope.classes:
        all_functions.extend(c.functions)
    s = list(global_scope.namespaces)
    while s:
        ns = s
        s = []
        for n in ns:
            all_functions.extend(n.functions)
            for c in n.classes:
                all_functions.extend(c.functions)
            s.extend(n.namespaces)
    return all_functions

def collect_from_global_scope(self, global_scope, store = False):
    assert isinstance(global_scope, CppGlobalScope)
    data = [FunctionCollector(f) for f in collect_functions(global_scope)]
    for collector in data:
        self.collect(collector, store = store)
    return data
