
# TODO
# - sort packages according to dependencies;
#   cmake might change environment variables, so we must emulate the same
#   behaviour as catkin (and the environment must be shared between analyses).
# - add a register configuration to haros interface;
#   a configuration is more or less a super class of launch files,
#   it should feature nodes, environment, and other resources.
# - implement the proper workflow; do not assume the code has been compiled.
# - analyse nodes of the same type but different argv; memoize the rest.

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
from cmake_analyser.analyser import CMakeAnalyser
from launch_analyser.analyser import LaunchFileAnalyser


LIB_PATH = "/usr/lib/llvm-3.8/lib"
CWS = "/home/andre/catkin_ws"
DB_PATH = "/home/andre/catkin_ws/build"
STD_INCLUDES = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"

###############################################################################
# HAROS plugin
###############################################################################

# Plugin workflow:
# We must parse CMake files first, because these may change the environment...
# This gives us all possible executables. Then we move on to launch files.
# From launch files we extract configurations, nodes and package dependencies.
# Finally we move on to source code if the nodes and executables match.
#
# HACK: since we must compile the code before analysing it, the machine's
# environment will be changed with whatever the CMake files do.
# This way, we can start directly with the launch files and a copy of the
# environment, leaving the CMake files for when we already a list of nodes.
# File analysis runs first, so we analyse launch files there.


class InternalState(object):
    def __init__(self):
        self.environment = dict(os.environ)
        self.configurations = []
        self.executables = {}
        self.node_types = {}
        self.unknown_packages = set()
        self.compile_db = clang.CompilationDatabase.fromDirectory(DB_PATH)


def pre_analysis():
    sys.setrecursionlimit(2000)
    clang.Config.set_library_path(LIB_PATH)
    return InternalState()


def file_analysis(iface, launch_file):
    path = launch_file.get_path()
    if not path in iface.state.configurations:
        config = ROS.Configuration(path.replace("/home/andre/catkin_ws/", ""),
                                   iface.state.environment)
        analyser = LaunchFileAnalyser(iface.state.environment, iface,
                                      resources = config.resources)
        analyser.onNode.sub()
        merged_launch = analyser.analyse(path)
        for launch in merged_launch.includes:
            iface.state.configurations[launch.name] = None
        if analyser.stats.unknown_pkgs or not merged_launch.valid:
            iface.state.configurations[path] = None
        else:
            iface.state.configurations[path] = config
            config.pkg_depends.update(merged_launch.pkg_depends)
            for ref in merged_launch.unknown:
                if ref[0] == "pkg":
                    iface.state.unknown_packages.add(ref[1])
            for node in merged_launch.nodes:
                if not node.package in iface.state.node_types:
                    iface.state.node_types[node.package] = set()
                iface.state.node_types[node.package].add(node.node_type)


def package_analysis(iface, package):
    xs = {}
    iface.state.executables[package.name] = xs
    cmake_vars = {
        "catkin_INCLUDE_DIRS": "/home/andre/catkin_ws/devel/include",
        "Boost_INCLUDE_DIRS": "/usr/include/",
        "Eigen_INCLUDE_DIRS": "/usr/include/eigen3",
        "ImageMagick_INCLUDE_DIRS": "/usr/include/ImageMagick",
        "PROJECT_SOURCE_DIR": package.path
    }
    parser = CMakeAnalyser(cmake_vars, iface.state.environment, iface)
    parser.analyse(os.path.join(package.path, "CMakeLists.txt"))
    for x in parser.data["executables"] + parser.data["libraries"]:
        xs[x[0]] = x[1]



    if package.name in iface.state.node_types:
        analyse = []
        node_types = iface.state.node_types[package.name]
        for exe in _cmake_executables(iface, package):
            if exe[0] in node_types:
                for f in exe[1]:
                    for c in state.database.getCompileCommands(file_path) or []:
                    # if f in compile_commands
                    analyse.append(f)


def post_analysis(iface):
    xs = iface.state.executables
    for config in iface.state.configurations:
        for node in config.nodes():
            files = xs.get(node.package, {}).get(node.node_type, [])
            for f in files:
                if _clang_node(iface.state.compile_db, f):
                    pass
                    # if ast in cache, use it
                    # if not, analyse source


#


def _clang_node(compile_db, file_path):
    data = None
    for c in compile_db.getCompileCommands(file_path) or []:
        with cwd(os.path.join(DB_PATH, c.directory)):
            args = ["-I" + STD_INCLUDES] + list(c.arguments)[1:]
            index = clang.Index.create()
            unit = index.parse(None, args)
            # check for problems
            if unit.diagnostics:
                for d in unit.diagnostics:
                    if d.severity >= clang.Diagnostic.Error:
                        print "[CLANG]", d.spelling
            data = _ast_analysis(unit, scope.package, state)
    return data


def _ast_analysis(unit, package, state):
    cursor = unit.cursor
    # we cannot reuse the same instance of global scope, or else we get stuff
    # from other files mixed in
    global_scope = CppGlobalScope()
    for node in cursor.get_children():
        if node.location.file and node.location.file.name.startswith(CWS):
            global_scope.add_from_cursor(node)
    if not package.id in state.pkg_metrics:
        state.pkg_metrics[package.id] = collectors.GlobalCollector()
    state.pkg_metrics[package.id].collect_from_global_scope(global_scope)
    return state.metrics.collect_from_global_scope(global_scope, store = True)


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
