
# TODO
# resolve references
# references across files in the same package

# https://github.com/llvm-mirror/clang/blob/master/bindings/python/clang/cindex.py

# might be interesting
# https://github.com/pybee-attic/sealang

# Install
# sudo pip install clang
# sudo apt-get install libclang-3.8-dev
#   optional, currently hard-coded:
# set up LD_LIBRARY_PATH to point to the libclang.so shared library
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib

###
# standard packages
from collections import namedtuple, Counter
import csv
import os
import sys
import time

###
# third-party packages
import clang.cindex as clang

###
# internal packages
from cmake_analyser.analyser import CMakeAnalyser
from clang_plugin.cpp_model import CppGlobalScope, advertise_list as ADVERTISE, subscribe_list as SUBSCRIBE
import clang_plugin.cpp_model as model
import clang_plugin.collectors as collectors


###############################################################################
# Internal State
###############################################################################

LIB_PATH = "/usr/lib/llvm-3.8/lib"
DB_PATH = "/home/andre/catkin_ws/build"
STD_INCLUDES = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"


InternalState = namedtuple("InternalState",
                           ["database", "start", "metrics", "pkg_metrics"])


###############################################################################
# HAROS Plugin Interface
###############################################################################

def pre_analysis():
    sys.setrecursionlimit(2000)
    clang.Config.set_library_path(LIB_PATH)
    db = clang.CompilationDatabase.fromDirectory(DB_PATH)
    return InternalState(db, time.time(), collectors.GlobalCollector(), {})


def file_analysis(iface, scope):
    file_path   = scope.get_path()
    state       = iface.state
    for c in state.database.getCompileCommands(file_path) or []:
        # print "[CLANG] Analysing", file_path
        with cwd(os.path.join(DB_PATH, c.directory)):
            args = ["-I" + STD_INCLUDES] + list(c.arguments)[1:]
            index = clang.Index.create()
            unit = index.parse(None, args)
            # check for problems
            if unit.diagnostics:
                for d in unit.diagnostics:
                    if d.severity >= clang.Diagnostic.Error:
                        print "[CLANG]", d.spelling
            # traverse nodes
            print "[CLANG] analysing", file_path
            data = _ast_analysis(unit, scope.package, state)


def post_analysis(iface):
    _export_subscriber_csv(iface)
    _export_publisher_csv(iface)
    _export_service_csv(iface)
    _export_message_csv(iface)
    _export_param_csv(iface)
    _export_param_type_csv(iface)
    _export_other(iface)
    print ""
    print "TOTAL ADVERTISE", len(ADVERTISE)
    print "RETURN TYPES", Counter(ADVERTISE)
    print ""
    print "TOTAL SUBSCRIBE", len(SUBSCRIBE)
    print "RETURN TYPES", Counter(SUBSCRIBE)
    print ""
    print "COUNTED ADVERTISE", collectors.advertise_count/2
    print "COUNTED SUBSCRIBE", collectors.subscribe_count/2
    print "COUNTED PUB-SUB VARS", collectors.pubsub_var
    print ""
    print "COLLECTED {}/{} FUNCTIONS".format(collectors.collected_functions/2,
                                             model.function_counter)
    print ""
    print "[CLANG] Analysis took", int(time.time() - iface.state.start), "seconds"



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

def _ast_analysis(unit, package, state):
    allowed = set(f.get_path() for f in package.source_files)
    cursor = unit.cursor
    # we cannot reuse the same instance of global scope, or else we get stuff
    # from other files mixed in
    global_scope = CppGlobalScope()
    for node in cursor.get_children():
        if node.location.file and node.location.file.name.startswith("/home/andre/catkin_ws"):
            global_scope.add_from_cursor(node)
    if not package.id in state.pkg_metrics:
        state.pkg_metrics[package.id] = collectors.GlobalCollector()
    state.pkg_metrics[package.id].collect_from_global_scope(global_scope)
    return state.metrics.collect_from_global_scope(global_scope, store = True)


def _report_results(iface, data):
    for fc in data:
        iface.report_metric("function_calls", fc.function_calls)
        iface.report_metric("unique_function_calls", len(fc.function_set))
    # TODO FIXME now data is a list of FunctionCollector
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
    function_callbacks = 0
    method_callbacks = 0
    boost_callbacks = 0
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
        if datum.overload == collectors.SUB_TYPE_1:
            method_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_2:
            function_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_3:
            boost_callbacks += 1
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
        if datum.overload == collectors.SUB_TYPE_1:
            method_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_2:
            function_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_3:
            boost_callbacks += 1
    iface.report_metric("hardcoded_topics", hardcoded_topics)
    iface.report_metric("hardcoded_queue_sizes", hardcoded_queue_sizes)
    iface.report_metric("infinite_queues", infinite_queues)
    iface.report_metric("transport_hints", transport_hints)
    iface.report_metric("global_resource_names", global_resource_names)
    iface.report_metric("function_callbacks", function_callbacks)
    iface.report_metric("method_callbacks", method_callbacks)
    iface.report_metric("boost_callbacks", boost_callbacks)

def _report_advertise_data(iface, data):
    hardcoded_topics = 0
    hardcoded_queue_sizes = 0
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
    function_callbacks = 0
    method_callbacks = 0
    boost_callbacks = 0
    iface.report_metric("service_servers", data.num_advertise_service())
    for topic, datum in data.advertise_service.iteritems():
        hardcoded_services += 1
        iface.report_metric("advertise_service_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if datum.overload == collectors.SUB_TYPE_1:
            method_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_2:
            function_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_3:
            boost_callbacks += 1
    for datum in data.advertise_service_unknown:
        iface.report_metric("advertise_service_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if datum.overload == collectors.SUB_TYPE_1:
            method_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_2:
            function_callbacks += 1
        if datum.overload == collectors.SUB_TYPE_3:
            boost_callbacks += 1
    iface.report_metric("hardcoded_services", hardcoded_services)
    iface.report_metric("function_callbacks", function_callbacks)
    iface.report_metric("method_callbacks", method_callbacks)
    iface.report_metric("boost_callbacks", boost_callbacks)

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
# Exporting Functions
###############################################################################

def _export_subscriber_csv(iface):
    with open("subscriber.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        rows = iface.state.metrics.sub.csv_subscribe()
        for row in rows:
            out.writerow(row)
    iface.export_file("subscriber.csv")
    headers = rows[0]
    headers.insert(0, "Package")
    with open("subscriber_pkg.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        out.writerow(headers)
        for pkg, metrics in iface.state.pkg_metrics.iteritems():
            rows = metrics.sub.csv_subscribe()[1:]
            for row in rows:
                row.insert(0, pkg)
                out.writerow(row)
    iface.export_file("subscriber_pkg.csv")

def _export_publisher_csv(iface):
    with open("publisher.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        rows = iface.state.metrics.pub.csv_publish()
        for row in rows:
            out.writerow(row)
    iface.export_file("publisher.csv")
    headers = rows[0]
    headers.insert(0, "Package")
    with open("publisher_pkg.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        out.writerow(headers)
        for pkg, metrics in iface.state.pkg_metrics.iteritems():
            rows = metrics.pub.csv_publish()[1:]
            for row in rows:
                row.insert(0, pkg)
                out.writerow(row)
    iface.export_file("publisher_pkg.csv")

def _export_service_csv(iface):
    with open("service.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        rows = iface.state.metrics.rpc.csv_service()
        for row in rows:
            out.writerow(row)
    iface.export_file("service.csv")
    headers = rows[0]
    headers.insert(0, "Package")
    with open("service_pkg.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        out.writerow(headers)
        for pkg, metrics in iface.state.pkg_metrics.iteritems():
            rows = metrics.rpc.csv_service()[1:]
            for row in rows:
                row.insert(0, pkg)
                out.writerow(row)
    iface.export_file("service_pkg.csv")

def _export_message_csv(iface):
    with open("message.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for row in iface.state.metrics.csv_message_types():
            out.writerow(row)
    iface.export_file("message.csv")

def _export_param_csv(iface):
    with open("param.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        rows = iface.state.metrics.param.csv_param()
        for row in rows:
            out.writerow(row)
    iface.export_file("param.csv")
    headers = rows[0]
    headers.insert(0, "Package")
    with open("param_pkg.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        out.writerow(headers)
        for pkg, metrics in iface.state.pkg_metrics.iteritems():
            rows = metrics.param.csv_param()[1:]
            for row in rows:
                row.insert(0, pkg)
                out.writerow(row)
    iface.export_file("param_pkg.csv")

def _export_param_type_csv(iface):
    with open("param_type.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for row in iface.state.metrics.param.csv_param_type():
            out.writerow(row)
    iface.export_file("param_type.csv")

def _export_other(iface):
    with open("other.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for row in iface.state.metrics.csv_other():
            out.writerow(row)
    iface.export_file("other.csv")
    with open("pub_data.txt", "w") as datafile:
        datafile.write(iface.state.metrics.str_pub_data())
    iface.export_file("pub_data.txt")
    with open("sub_data.txt", "w") as datafile:
        datafile.write(iface.state.metrics.str_sub_data())
    iface.export_file("sub_data.txt")
    with open("rpc_data.txt", "w") as datafile:
        datafile.write(iface.state.metrics.str_rpc_data())
    iface.export_file("rpc_data.txt")
    with open("other_data.txt", "w") as datafile:
        datafile.write(iface.state.metrics.str_other_data())
    iface.export_file("other_data.txt")



###############################################################################
# Testing and Helper Functions
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
    data = collectors.FileCollector.from_global_scope(global_scope)
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
    file_path = "/home/andre/catkin_ws/src/beginner_tutorials/src/listener.cpp"
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
