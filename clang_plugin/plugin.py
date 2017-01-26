
# TODO
# resolve references
# references across files in the same package

# https://github.com/llvm-mirror/clang/blob/master/bindings/python/clang/cindex.py

# might be interesting
# https://github.com/pybee-attic/sealang

###
# standard packages
import os

###
# third-party packages
import clang.cindex as clang

###
# internal packages
from cmake_analyser.analyser import CMakeAnalyser
from clang_plugin.cpp_model import CppGlobalScope
from clang_plugin.collectors import FileCollector


###############################################################################
# HAROS Plugin Interface
###############################################################################

def pre_analysis():
    clang.Config.set_library_path("/usr/lib/llvm-3.8/lib")
    index = clang.Index.create()
    return (index, {})


def file_analysis(iface, scope):
    file_path   = scope.get_path()
    index       = iface.state[0]
    includes    = iface.state[1]
    _find_includes(iface, scope.package, includes)
    assert scope.package.id in includes
    args = ["-I" + path for path in includes[scope.package.id]]
    args.append("-x")
    args.append("c++")
    # args.append("-std=c++11")
    unit = index.parse(file_path, args)
    # check for problems
    if unit.diagnostics:
        for d in unit.diagnostics:
            if d.severity >= clang.Diagnostic.Error:
                print d
    # traverse nodes
    data = _ast_analysis(unit, file_path)
    _report_results(iface, data)


###############################################################################
# CMake Analysis
###############################################################################

_DEFAULT_INCLUDES = ["/usr/lib/llvm-3.8/lib/clang/3.8.0/include",
                     "/usr/include/eigen3", "/usr/include/pcl-1.7/pcl"]

_DEFAULT_VARS = {
    "catkin_INCLUDE_DIRS": "/home/andre/catkin_ws/devel/include",
    "Boost_INCLUDE_DIRS": "/usr/include/",
    "Eigen_INCLUDE_DIRS": "/usr/include/eigen3",
    "ImageMagick_INCLUDE_DIRS": "/usr/include/ImageMagick"
}

def _find_includes(iface, package, includes, var_data = None):
    if package.id in includes:
        return
    if var_data is None:
        var_data = dict(_DEFAULT_VARS)
    includes[package.id] = _read_package_includes(iface, package, var_data)
    for id in package.dependencies:
        dep = iface.find_package(id)
        if dep:
            _find_includes(iface, dep, includes, var_data = var_data)
            includes[package.id].extend(includes[dep.id])
    includes[package.id] = list(set(includes[package.id]))


def _read_package_includes(iface, package, var_data):
    var_data["PROJECT_SOURCE_DIR"] = package.path
    cmake_path = os.path.join(package.path, "CMakeLists.txt")
    parser = CMakeAnalyser(var_data, dict(os.environ), iface)
    parser.analyse(cmake_path)
    includes = list(_DEFAULT_INCLUDES)
    package_includes = parser.data["package_includes"]
    if package_includes:
        includes.extend(package_includes)
        var_data[package.id + "_INCLUDE_DIRS"] = ";".join(package_includes)
    includes.extend(parser.data["include_dirs"])
    return includes



###############################################################################
# Analyser
###############################################################################

def _ast_analysis(unit, file_path):
    cursor = unit.cursor
    # we cannot reuse the same instance of global scope, or else we get stuff
    # from other files mixed in
    global_scope = CppGlobalScope()
    for node in cursor.get_children():
        if node.location.file and node.location.file.name == file_path:
            global_scope.add_from_cursor(node)
    return FileCollector.from_global_scope(global_scope)


def _report_results(iface, data):
    _report_subscribe_data(iface, data)
    _report_advertise_data(iface, data)
    _report_publish_data(iface, data)
    _report_advertise_service_data(iface, data)
    _report_service_client_data(iface, data)
    _report_other_data(iface, data)


def _report_subscribe_data(iface, data):
    hardcoded_topics = 0
    hardcoded_queue_sizes = 0
    infinite_queues = 0
    transport_hints = 0
    global_resource_names = 0
    iface.report_metric("subscribers", data.num_subscribe())
    for topic, datum in data.subscribe.iteritems():
        hardcoded_topics += 1
        iface.report_metric("subscribe_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            hardcoded_queue_sizes += 1
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
            if datum.queue_size == 0:
                infinite_queues += 1
        if datum.transport_hints:
            transport_hints += 1
        if topic.startswith("/"):
            global_resource_names += 1
    for datum in data.subscribe_unknown:
        iface.report_metric("subscribe_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            hardcoded_queue_sizes += 1
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
            if datum.queue_size == 0:
                infinite_queues += 1
        if datum.transport_hints:
            transport_hints += 1
    iface.report_metric("hardcoded_topics", hardcoded_topics)
    iface.report_metric("hardcoded_queue_sizes", hardcoded_queue_sizes)
    iface.report_metric("infinite_queues", infinite_queues)
    iface.report_metric("transport_hints", transport_hints)
    iface.report_metric("global_resource_names", global_resource_names)

def _report_advertise_data(iface, data):
    hardcoded_topics = 0
    infinite_queues = 0
    latching = 0
    global_resource_names = 0
    iface.report_metric("publishers", data.num_advertise())
    for topic, datum in data.advertise.iteritems():
        hardcoded_topics += 1
        iface.report_metric("advertise_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            hardcoded_queue_sizes += 1
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
            if datum.queue_size == 0:
                infinite_queues += 1
        if datum.latch:
            latching += 1
        if topic.startswith("/"):
            global_resource_names += 1
    for datum in data.advertise_unknown:
        iface.report_metric("advertise_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            hardcoded_queue_sizes += 1
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
            if datum.queue_size == 0:
                infinite_queues += 1
        if datum.latch:
            latching += 1
    iface.report_metric("hardcoded_topics", hardcoded_topics)
    iface.report_metric("hardcoded_queue_sizes", hardcoded_queue_sizes)
    iface.report_metric("infinite_queues", infinite_queues)
    iface.report_metric("latching_topics", latching)
    iface.report_metric("global_resource_names", global_resource_names)

def _report_publish_data(iface, data):
    iface.report_metric("active_publishers", len(data.publish))
    for var, d in data.publish.iteritems():
        for datum in d:
            iface.report_metric("publish_nesting", datum.nesting,
                                line = datum.line, function = datum.function)

def _report_advertise_service_data(iface, data):
    hardcoded_services = 0
    iface.report_metric("service_servers", data.num_advertise_service())
    for topic, datum in data.advertise_service.iteritems():
        hardcoded_services += 1
        iface.report_metric("advertise_service_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    for datum in data.advertise_service_unknown:
        iface.report_metric("advertise_service_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    iface.report_metric("hardcoded_services", hardcoded_services)

def _report_service_client_data(iface, data):
    hardcoded_services = 0
    iface.report_metric("service_clients", data.num_service_client())
    for topic, datum in data.service_client.iteritems():
        hardcoded_services += 1
        iface.report_metric("service_client_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    for datum in data.service_client_unknown:
        iface.report_metric("service_client_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    iface.report_metric("hardcoded_services", hardcoded_services)

def _report_other_data(iface, data):
    hardcoded_spin_rates = 0
    for var, datum in data.spin_rate.iteritems():
        if isinstance(datum.rate, (int, long)):
            hardcoded_spin_rates += 1
            iface.report_metric("spin_rate", datum.rate,
                                function = datum.function)
    iface.report_metric("hardcoded_spin_rates", hardcoded_spin_rates)



###############################################################################
# Testing and Helper Functions
###############################################################################

def analysis(file_path, index):
    unit = index.parse(file_path, ["-I" + "/usr/lib/llvm-3.8/lib/clang/3.8.0/include",
                        "-I" + "/usr/include/eigen3",
                        "-x", "c++"]) # "-std=c++11"
    # check for problems
    if unit.diagnostics:
        for d in unit.diagnostics:
            if d.severity >= clang.Diagnostic.Error:
                print d
    # traverse nodes
    cursor = unit.cursor
    global_scope = CppGlobalScope.INSTANCE
    for node in cursor.get_children():
        if node.location.file and node.location.file.name == file_path:
            global_scope.add_from_cursor(node)
    # print global_scope.pretty_str()
    data = FileCollector.from_global_scope(global_scope)
    if not data.publish_rate:
        print "No publish rates were detected."
    for topic, rate in data.publish_rate.iteritems():
        print topic, "publishes at", rate, "Hz"



def print_node(node, prefix):
    line = str(node.location.line)
    print prefix + node.kind.name, node.spelling, \
            "[ln " + line + "," + node.displayname + "]", \
            "[" + node.type.spelling + "]"


def traverse(cursor, prefix):
    child_prefix = prefix + "."
    for node in cursor.get_children():
        if not next(node.get_tokens(), None):
            print "NO TOKENS", node.kind.name, node.location.line
            continue
        # first print, then skip. show that there is something more
        print_node(node, prefix)
        if node.kind == clang.CursorKind.CALL_EXPR and node.spelling:
            args = list(node.get_arguments())
            if not args:
                print child_prefix + "{TOKENS}", [t.spelling for t in node.get_tokens()][:-1]
            for x in node.get_children():
                if x in args:
                    print child_prefix + "{ARGUMENT}", [t.spelling for t in x.get_tokens()][:-1]
                print_node(x, child_prefix)
                traverse(x, child_prefix + ".")
        else:
            traverse(node, child_prefix)

def print_ast(file_path, index):
    unit = index.parse(file_path, ["-I" + "/usr/lib/llvm-3.8/lib/clang/3.8.0/include",
                        "-I" + "/usr/include/eigen3",
                        "-x", "c++"]) # "-std=c++11"
    # check for problems
    if unit.diagnostics:
        for d in unit.diagnostics:
            print d
    # traverse nodes
    cursor = unit.cursor
    global_scope = CppGlobalScope.INSTANCE
    # first skip, then print. avoid includes, focus on given file
    for node in cursor.get_children():
        if node.location.file and node.location.file.name == file_path:
            print_node(node, ".")
            traverse(node, "..")


def main():
    file_path = "/home/andre/ros/tests/talker.cpp"
    # file_path = "/home/andre/kobuki/src/kobuki/kobuki_node/src/library/kobuki_ros.cpp"
    clang.Config.set_library_path("/usr/lib/llvm-3.8/lib")
    index = clang.Index.create()
    # print_ast(file_path, index)
    analysis(file_path, index)


###############################################################################
# Main Runnable
###############################################################################

if __name__ == "__main__":
    main()
