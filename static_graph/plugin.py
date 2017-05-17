
# TODO
# - add a register configuration to haros interface.
# - analyse nodes of the same type but different argv; memoize the rest.

###
# standard packages
import traceback

###
# internal packages
from static_graph.builder import setup, ConfigurationBuilder


###############################################################################
# HAROS plugin
###############################################################################

# Plugin workflow:
# We must parse CMake files first, because these may change the environment...
# This gives us all possible executables. Then we move on to launch files.
# From launch files we extract configurations, nodes and package dependencies.
# Finally we move on to source code if the nodes and executables match.


class InternalState(object):
    def __init__(self):
        self.builder = ConfigurationBuilder()
        self.launch_files = []


def pre_analysis():
    setup()
    return InternalState()


def file_analysis(iface, launch_file):
    iface.state.launch_files.append(launch_file)


def package_analysis(iface, package):
    iface.state.builder.with_package(package, iface)


def post_analysis(iface):
    try:
        for launch_file in iface.state.launch_files:
            config = iface.state.builder.from_launch(launch_file, iface)
            print
            if not config:
                print "Invalid launch file", launch_file.get_path()
                continue
            print config.name
            print "Depends on packages:"
            print "   ", config.pkg_depends
            if config.env_depends:
                print "Depends on environment:"
                print "   ", config.env_depends
            # print "Nodes:"
            # for node in config.nodes():
                # print "  {} [{}/{}]".format(node.full_name, node.package, node.node_type)
                # if not node._analysed or node._error:
                    # print "    [N/A]", node._error
                    # continue
                # ts = map(lambda x: x[0].full_name, node.publishers)
                # if ts:
                    # print "    pub {}".format(ts)
                # ts = map(lambda x: x[0].full_name, node.subscribers)
                # print "    sub {}".format(ts)
                # ts = map(lambda x: x[0].full_name, node.servers)
                # if ts:
                    # print "    srv {}".format(ts)
                # ts = map(lambda x: x[0].full_name, node.clients)
                # if ts:
                    # print "    cli {}".format(ts)
            _type_check_topics(config)
    except Exception as e:
        print traceback.print_exc()
    if iface.state.builder.unknown_packages:
        print
        print "Unknown packages", iface.state.builder.unknown_packages


def _type_check_topics(config):
    for topic in config.resources.get_topics():
        print
        print "TOPIC", topic.full_name
        for pub in topic.publishers:
            print "  [pub]", pub
        for sub in topic.subscribers:
            print "  [sub]", sub
