
###
# standard packages
import os

###
# internal packages
import haros_util.ros_model as ROS
from clang_plugin.new_cpp_model import CppAstParser, CppQuery, \
                                        CppFunctionCall, CppDefaultArgument, \
                                        CppOperator, resolve_reference, \
                                        resolve_expression, get_control_depth
from cmake_analyser.analyser import CMakeAnalyser
from launch_analyser.analyser import LaunchFileAnalyser

###
# constants
CWS             = "/home/andre/catkin_ws/"
DB_PATH         = "/home/andre/catkin_ws/build"
CATKIN_INCLUDE  = "/home/andre/catkin_ws/devel/include"
USR_INCLUDE     = "/usr/include/"
EIGEN_INCLUDE   = "/usr/include/eigen3"
MAGICK_INCLUDE  = "/usr/include/ImageMagick"


###############################################################################
# Resource Generators
###############################################################################

class BaseGenerator(object):
    def __init__(self, namespace, call):
        name = self._extract_name(call)
        if namespace:
            if namespace[-1].isalpha():
                name = namespace + "/" + name
            else:
                name = namespace + name
        self.name           = name
        self.namespace      = namespace
        self.message_type = self._extract_message_type(call)
        self.nesting = get_control_depth(call, recursive = True)
        self.file = call.file
        self.line = call.line
        self.is_global = ROS.is_global_name(name)

    def _extract_name(self, call):
        name = resolve_expression(call.arguments[0])
        if not isinstance(name, basestring):
            name = "?"
        return name

    def _extract_message_type(self, call):
        if call.template:
            return call.template[0]
        if call.name != "subscribe" and call.name != "advertiseService":
            return "?"
        callback = call.arguments[2] if call.name == "subscribe" \
                                     else call.arguments[1]
        while isinstance(callback, CppOperator):
            callback = callback.arguments[0]
        type_string = callback.result
        type_string = type_string.split(None, 1)[1]
        if type_string[0] == "(" and type_string[-1] == ")":
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
        if type_string.startswith("boost::function"):
            type_string = type_string[52:-25]
        return type_string


class TopicGenerator(BaseGenerator):
    def __init__(self, namespace, call):
        BaseGenerator.__init__(self, namespace, call)
        self.publisher = call.name == "advertise"
        self.queue_size = self._extract_queue_size(call)
        self.latch = self._extract_latching(call)

    def generate(self, node, resources):
        name = ROS.transform_name(self.name, ns = node.namespace,
                                  private_ns = node.full_name,
                                  remaps = node.remaps)
        topic = resources.get_topic(name)
        if topic is None:
            topic = ROS.Topic(self.name, ns = node.namespace,
                              private_ns = node.full_name,
                              message_type = self.message_type)
            resources.register(topic, remaps = node.remaps)
        if self.publisher:
            node.add_publisher(topic, self.message_type,
                               queue_size = self.queue_size,
                               nesting = self.nesting,
                               file = self.file, line = self.line,
                               global_ref = self.is_global)
            topic.add_publisher(node, self.message_type,
                                queue_size = self.queue_size,
                                nesting = self.nesting,
                                file = self.file, line = self.line,
                                global_ref = self.is_global)
        else:
            node.add_subscriber(topic, self.message_type,
                                queue_size = self.queue_size,
                                nesting = self.nesting,
                                file = self.file, line = self.line,
                                global_ref = self.is_global)
            topic.add_subscriber(node, self.message_type,
                                 queue_size = self.queue_size,
                                 nesting = self.nesting,
                                 file = self.file, line = self.line,
                                 global_ref = self.is_global)
        return topic

    def _extract_queue_size(self, call):
        queue_size = resolve_expression(call.arguments[1])
        if isinstance(queue_size, (int, long, float)):
            return queue_size
        return None

    def _extract_latching(self, call):
        if self.publisher:
            latch = call.arguments[-1]
            if isinstance(latch, CppDefaultArgument):
                latch = False
            elif not isinstance(latch, bool):
                latch = resolve_expression(latch)
            return latch
        return None


class ServiceGenerator(BaseGenerator):
    def __init__(self, namespace, call):
        BaseGenerator.__init__(self, namespace, call)
        self.server = call.name == "advertiseService"

    def generate(self, node, resources):
        name = ROS.transform_name(self.name, ns = node.namespace,
                                  private_ns = node.full_name,
                                  remaps = node.remaps)
        service = resources.get_service(name)
        if service is None:
            service = ROS.Service(self.name, ns = node.namespace,
                                  private_ns = node.full_name,
                                  message_type = self.message_type)
            resources.register(service, remaps = node.remaps)
        if self.server:
            node.add_server(service, self.message_type, nesting = self.nesting,
                            file = self.file, line = self.line,
                            global_ref = self.is_global)
            service.set_server(node, self.message_type, nesting = self.nesting,
                               file = self.file, line = self.line,
                               global_ref = self.is_global)
        else:
            node.add_client(service, self.message_type, nesting = self.nesting,
                            file = self.file, line = self.line,
                            global_ref = self.is_global)
            service.add_client(node, self.message_type, nesting = self.nesting,
                               file = self.file, line = self.line,
                               global_ref = self.is_global)
        return service


class ErrorGenerator(object):
    def __init__(self, message):
        self.message = message

    def generate(self, node, resources):
        node._error = self.message
        return self.message

###############################################################################
# ROS Configuration Builder
#       - extracts executables from CMakeLists
#       - parses launch files
#       - analyses C++ AST only when necessary
#       - caches communication primitives for each node type
###############################################################################

def setup():
    CppAstParser.set_library_path()
    CppAstParser.set_database(DB_PATH)


class ConfigurationBuilder(object):
    def __init__(self):
    # public:
        self.environment = dict(os.environ)
        self.launch_files = set()
        self.unknown_packages = set()
    # private:
        self._config = None
        self._exe = {}  # package -> (name -> [file])
        self._gen = {}  # cache to generate topics and services for nodes
        self._gen_entry = None

    def with_package(self, package, finder):
        """Register executables from a package's CMakeLists.
            This has to be called *before* 'from_launch'.
        """
        srcdir = package.path[len(CWS):]
        srcdir = CWS + srcdir.split(os.sep, 1)[0]
        executables = {}
        variables = {
            "catkin_INCLUDE_DIRS":      CATKIN_INCLUDE,
            "Boost_INCLUDE_DIRS":       USR_INCLUDE,
            "Eigen_INCLUDE_DIRS":       EIGEN_INCLUDE,
            "ImageMagick_INCLUDE_DIRS": MAGICK_INCLUDE,
            "PROJECT_SOURCE_DIR":       package.path
        }
        self._exe[package.name] = executables
        parser = CMakeAnalyser(self.environment, finder,
                               srcdir, DB_PATH,
                               variables = variables)
        if not os.path.isfile(os.path.join(package.path, "CMakeLists.txt")):
            return
        parser.analyse(os.path.join(package.path, "CMakeLists.txt"))
        for exe in parser.data["libraries"].itervalues():
            fs = list(exe.files)
            for link in exe.links:
                fs.extend(link.files)
            executables[exe.prefixed_name] = fs
        for exe in parser.data["executables"].itervalues():
            fs = list(exe.files)
            for link in exe.links:
                fs.extend(link.files)
            executables[exe.output_name] = fs
        for lib, name in package.nodelets:
            pkg = name.split("/")[0]
            name = name.split("/")[1]
            exemap = self._exe.get(pkg, {})
            if lib in exemap:
                exemap[name] = exemap[lib]
                del exemap[lib]
            

    def from_launch(self, launch_file, finder):
        """Build a Configuration, given a launch file.
            Control starts with the launch analyser.
            Whenever a node is parsed, control reverts to the builder.
            If the node is not cached, control is given to the clang parser.
            Whenever a primitive is detected, register the respective resource.
            Control reverts to the launch analyser until the end.
        """
        path = launch_file.get_path()
        if path in self.launch_files:
            return None
        self.launch_files.add(path)
        config = ROS.Configuration(launch_file.name.replace(".launch", ""),
                                   launch_file.package.id,
                                   self.environment)
        self._config = config
        analyser = LaunchFileAnalyser(self.environment, finder,
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
        config.env_depends.update(merged_launch.env_depends)
        config.valid = not bool(merged_launch.unknown)
        self._config = None
        return config


    def _onNode(self, node):
        """Called right after a <node> is parsed from the launch file."""
        node._error = None
        node._analysed = False
        if node.conditions:
            node._error = "conditional node ignored [{}/{}]".format(node.package, node.node_type) 
            return False
        ref = node.reference
        if not ref in self._gen:
            self._gen_entry = []
            self._clang_analysis(node)
            self._gen[ref]  = self._gen_entry
            self._gen_entry = None
        assert ref in self._gen
        for generator in self._gen[ref]:
            generator.generate(node, self._config.resources)
            node._analysed = True
        if not self._gen[ref]:
            node._error = "no ROS primitives"


    def _clang_analysis(self, node):
        files = self._exe.get(node.package, {}).get(node.node_type, ())
        if not files:
            self._gen_entry.append(ErrorGenerator("no C++ source"))
        parser = CppAstParser(workspace = CWS)
        for f in files:
            gs = parser.parse(f)
            if gs is None:
                self._gen_entry.append(ErrorGenerator("no compile commands for file " + f))
    # ----- queries after parsing, since global scope is reused ---------------
        for call in (CppQuery(parser.global_scope).all_calls
                       .where_name("advertise")
                       .where_result("ros::Publisher").get()):
            self._onRosPrimitive(call, TopicGenerator)
        for call in (CppQuery(parser.global_scope).all_calls
                       .where_name("subscribe")
                       .where_result("ros::Subscriber").get()):
            self._onRosPrimitive(call, TopicGenerator)
        for call in (CppQuery(parser.global_scope).all_calls
                       .where_name("advertiseService")
                       .where_result("ros::ServiceServer").get()):
            self._onRosPrimitive(call, ServiceGenerator)
        for call in (CppQuery(parser.global_scope).all_calls
                       .where_name("serviceClient")
                       .where_result("ros::ServiceClient").get()):
            self._onRosPrimitive(call, ServiceGenerator)


    def _onRosPrimitive(self, call, generator):
        """Called after a primitive call is detected in the C++ AST."""
        if len(call.arguments) > 1:
            ns = self._resolve_node_handle(call)
            self._gen_entry.append(generator(ns, call))


    def _resolve_node_handle(self, call):
        ns = "?/"
        value = resolve_reference(call.method_of) if call.method_of else None
        if not value is None:
            if isinstance(value, CppFunctionCall):
                if value.name == "NodeHandle":
                    if len(value.arguments) == 2:
                        value = value.arguments[0]
                        if isinstance(value, basestring):
                            ns = value
                        elif isinstance(value, CppDefaultArgument):
                            ns = ""
                    elif len(value.arguments) == 1:
                        value = value.arguments[0]
                        if isinstance(value, CppFunctionCall):
                            if value.name == "getNodeHandle":
                                ns = ""
                            elif value.name == "getPrivateNodeHandle":
                                ns = "~"
                elif value.name == "getNodeHandle":
                    ns = ""
                elif value.name == "getPrivateNodeHandle":
                    ns = "~"
        return ns
