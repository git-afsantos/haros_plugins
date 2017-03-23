
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
CWS             = "/home/andre/catkin_ws/"
DB_PATH         = "/home/andre/catkin_ws/build"
STD_INCLUDES    = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
CATKIN_INCLUDE  = "/home/andre/catkin_ws/devel/include"
USR_INCLUDE     = "/usr/include/"
EIGEN_INCLUDE   = "/usr/include/eigen3"
MAGICK_INCLUDE  = "/usr/include/ImageMagick"


###############################################################################
# Resource Generators
###############################################################################

class TopicGenerator(object):
    def __init__(self, name, method):
        self.name       = name
        self.publisher  = method == "advertise"

    def generate(self, node, resources):
        name = ROS.resolve_name(self.name, ns = node.namespace)
        topic = resources.get_topic(name, remap = True)
        if topic is None:
            topic = ROS.Topic(name)
            resources.register(topic)
        if self.publisher:
            node.publishers.append(topic)
            topic.publishers.append(node)
        else:
            node.subscribers.append(topic)
            topic.subscribers.append(node)
        return topic


class ServiceGenerator(object):
    def __init__(self, name, method):
        self.name   = name
        self.server = method == "advertiseService"

    def generate(self, node, resources):
        name = ROS.resolve_name(self.name, ns = node.namespace)
        service = resources.get_service(name, remap = True)
        if service is None:
            service = ROS.Service(name)
            resources.register(service)
        if self.server:
            node.servers.append(service)
            service.server = node
        else:
            node.clients.append(service)
            service.clients.append(node)
        return service

###############################################################################
# ROS Configuration Builder
#       - extracts executables from CMakeLists
#       - parses launch files
#       - analyses C++ AST only when necessary
#       - caches communication primitives for each node type
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
        self._db = clang.CompilationDatabase.fromDirectory(DB_PATH)
        self._exe = {}  # package -> (name -> [file])
        self._gen = {}  # cache to generate topics and services for nodes
        self._gen_entry = None
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

    def from_launch(self, launch_file, finder):
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
        config = ROS.Configuration(path.replace(CWS, ""),
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
        config.valid = not bool(merged_launch.unknown)
        self._config = None
        return config


    def _onNode(self, node):
        """Called right after a <node> is parsed from the launch file."""
        ref = node.reference
        if not ref in self._gen:
            self._gen_entry = []
            self._gen[ref] = self._gen_entry
            self._clang_analysis(node)
            self._gen_entry = None
        assert ref in self._gen
        for generator in self._gen[ref]:
            generator.generate(node, self._config.resources)


    _CALLS = {
        ("advertise",        "ros::Publisher"):     TopicGenerator,
        ("subscribe",        "ros::Subscriber"):    TopicGenerator,
        ("advertiseService", "ros::ServiceServer"): ServiceGenerator,
        ("serviceClient",    "ros::ServiceClient"): ServiceGenerator
    }

    def _onFunctionCall(self, call):
        """Called right after a function call is parsed from the C++ AST."""
        generator = ConfigurationBuilder._CALLS.get((call.name, call.result))
        if generator and len(call.arguments) > 1:
            topic = call.arguments[0]
            if isinstance(topic, basestring):
                self._gen_entry.append(generator(topic, call.name))


    def _clang_analysis(self, node):
        files = self._exe.get(node.package, {}).get(node.node_type, [])
        for f in files:
            for c in self._db.getCompileCommands(f) or ():
                with cwd(os.path.join(DB_PATH, c.directory)):
                    args = ["-I" + STD_INCLUDES] + list(c.arguments)[1:]
                    index = clang.Index.create()
                    unit = index.parse(None, args)
    # ----- check for compilation problems ------------------------------------
                    if unit.diagnostics:
                        for d in unit.diagnostics:
                            if d.severity >= clang.Diagnostic.Error:
                                print "[CLANG]", d.spelling
    # ----- actual AST analysis -----------------------------------------------
                    global_scope = CppGlobalScope()
                    for child in unit.cursor.get_children():
                        if child.location.file \
                                and child.location.file.name.startswith(CWS):
                            global_scope.add_from_cursor(child)


###############################################################################
# Helper Functions
###############################################################################

class cwd:
    """Run a block of code from a specified working directory"""
    def __init__(self, path):
        self.dir = path

    def __enter__(self):
        self.old_dir = os.getcwd()
        os.chdir(self.dir)

    def __exit__(self, exc_type, exc_value, traceback):
        os.chdir(self.old_dir)
