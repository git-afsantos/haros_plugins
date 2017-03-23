
# TODO
# - add a register configuration to haros interface.
# - analyse nodes of the same type but different argv; memoize the rest.

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
    for launch_file in iface.state.launch_files:
        config = iface.state.builder.from_launch(launch_file, iface)
        print
        print config.name
        print "Depends on packages:"
        print "   ", config.pkg_depends
        print "Depends on environment:"
        print "   ", config.env_depends
        print "Nodes:"
        for node in config.nodes():
            print "  {}[{}/{}]".format(node.full_name, node.package, node.node_type)
            print "    pub {}".format(map(lambda x: x.full_name, node.publishers))
            print "    sub {}".format(map(lambda x: x.full_name, node.subscribers))
            print "    srv {}".format(map(lambda x: x.full_name, node.servers))
            print "    cli {}".format(map(lambda x: x.full_name, node.clients))
