
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
            print "   ", ", ".join(config.pkg_depends)
            if config.env_depends:
                print "Depends on environment:"
                print "   ", ", ".join(config.env_depends)
            print "Nodes:"
            for node in config.nodes():
                print "  {} [{}/{}]".format(node.full_name, node.package, node.node_type)
                if not node._analysed or node._error:
                    print "    [N/A]", node._error
                    continue
                for pub in node.publishers:
                    print "    pub{}[{}] {}".format("*" if pub[3] else "",
                                                    pub[2], pub[0].full_name)
                for sub in node.subscribers:
                    print "    sub{}[{}] {}".format("*" if sub[3] else "",
                                                    sub[2], sub[0].full_name)
                for srv in node.servers:
                    print "    srv{} {}".format("*" if srv[2] else "",
                                                srv[0].full_name)
                for cli in node.clients:
                    print "    cli{} {}".format("*" if cli[2] else "",
                                                cli[0].full_name)
            _type_check_topics(config)
            _check_disconnected_topics(config)
    except Exception as e:
        print traceback.print_exc()
    if iface.state.builder.unknown_packages:
        print
        print "Unknown packages", iface.state.builder.unknown_packages


def _type_check_topics(config):
    for topic in config.resources.get_topics():
        for pub in topic.publishers:
            if pub[1] and pub[1] != topic.message_type:
                print "[WARNING] Topic type mismatch on publisher", pub[0].reference
                print "  expected:", topic.message_type
                print "     found:", pub[1]
        for sub in topic.subscribers:
            if sub[1] and sub[1] != topic.message_type:
                print "[WARNING] Topic type mismatch on subscriber", sub[0].reference
                print "  expected:", topic.message_type
                print "     found:", sub[1]


def _check_disconnected_topics(config):
    topics = config.resources.get_topics()
    for topic in topics:
        if topic.is_disconnected:
            for other in topics:
                if other is topic:
                    continue
                test = other.is_disconnected
                test = test and len(topic.publishers) != len(other.publishers)
                test = test and topic.message_type == other.message_type
                test = test and _names_are_similar(topic, other)
                if test:
                    print "[WARNING] Possible naming mistake."
                    print "  Topics {} and {} are similar and disconnected.".format(
                                topic.full_name, other.full_name)


def _names_are_similar(r1, r2):
    return (r1.full_name.endswith(r2.name)
            or r2.full_name.endswith(r1.name)
            or r1.original_name.endswith(r2.name)
            or r2.original_name.endswith(r1.name)
            or r1.given_name == r2.given_name)