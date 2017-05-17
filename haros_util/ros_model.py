
###############################################################################
# ROS Model - Resources
###############################################################################

class Resource(object):
    def __init__(self, name, ns = "/", private_ns = ""):
        self.given_name = name
        self.full_name = resolve_name(name, ns = ns, private_ns = private_ns)
        self.original_name = self.full_name
        self.name = self.full_name.split("/")[-1]
        self.namespace = self.full_name.rsplit("/", 1)[0]


class Node(Resource):
    def __init__(self, name, pkg, ntype, nodelet = False, args = None,
                 remaps = None, ns = "/", private_ns = ""):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.package = pkg
        self.node_type = ntype      # executable name
        self.nodelet = nodelet      # is this a nodelet?
        self.argv = args if not args is None else []
        self.remaps = remaps if not remaps is None else {}
        self.publishers = []
        self.subscribers = []
        self.servers = []
        self.clients = []

    @property
    def reference(self):
        return self.package + "/" + self.node_type

    def add_publisher(self, topic, message_type, queue_size = 0, nesting = 0):
        self.publishers.append((topic, message_type, queue_size, nesting))

    def add_subscriber(self, topic, message_type, queue_size = 0, nesting = 0):
        self.subscribers.append((topic, message_type, queue_size, nesting))

    def add_server(self, service, message_type, nesting = 0):
        self.servers.append((node, message_type, nesting))

    def add_client(self, service, message_type, nesting = 0):
        self.clients.append((node, message_type, nesting))


class Topic(Resource):
    def __init__(self, name, ns = "/", private_ns = "", message_type = None):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.message_type = message_type
        self.publishers = []
        self.subscribers = []

    def add_publisher(self, node, message_type, queue_size = 0, nesting = 0):
        self.publishers.append((node, message_type, queue_size, nesting))

    def add_subscriber(self, node, message_type, queue_size = 0, nesting = 0):
        self.subscribers.append((node, message_type, queue_size, nesting))


class Service(Resource):
    def __init__(self, name, ns = "/", private_ns = "", message_type = None):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.message_type = message_type
        self.server = None
        self.clients = []

    def set_server(self, node, message_type, nesting = 0):
        self.server = (node, message_type, nesting)

    def add_client(self, node, message_type, nesting = 0):
        self.clients.append((node, message_type, nesting))


class Parameter(Resource):
    def __init__(self, name, ptype, value, ns = "/", private_ns = ""):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.type = ptype
        self.value = value


###############################################################################
# ROS Model - Resource Graph
###############################################################################

class ResourceGraph(object):
    def __init__(self, resources = None, remaps = None, collisions = None):
        self.resources = resources if not resources is None else {}
        self.remaps = remaps if not remaps is None else {}
        self.collisions = collisions if not collisions is None else {}

    @property
    def n_collisions(self):
        n = 0
        for name, values in self.collisions.iteritems():
            n += len(values)
        return n

    def get_node(self, name, remaps = False):
        return self._get(name, Node, remaps)

    def get_nodes(self):
        return self._get_all(Node)

    def get_param(self, name, remaps = False):
        return self._get(name, Parameter, remaps)

    def get_params(self):
        return self._get_all(Parameter)

    def get_topic(self, name, remaps = False):
        return self._get(name, Topic, remaps)

    def get_topics(self):
        return self._get_all(Topic)

    def get_service(self, name, remaps = False):
        return self._get(name, Service, remaps)

    def get_services(self):
        return self._get_all(Service)

    def _get(self, name, cls, remaps):
        if isinstance(remaps, dict):
            name = remaps.get(name, name)
        elif remaps:
            name = self.remaps.get(name, name)
        r = self.resources.get(name)
        if r is None or isinstance(r, cls):
            return r
        rs = self.collisions.get(name)
        if rs:
            for r in rs:
                if isinstance(r, cls):
                    return r
        return None

    def _get_all(self, cls):
        all = []
        for _, r in self.resources.iteritems():
            if isinstance(r, cls):
                all.append(r)
        for _, rs in self.collisions.iteritems():
            for r in rs:
                if isinstance(r, cls):
                    all.append(r)
        return all

    def register(self, resource):
        name = resource.full_name
        name = self.remaps.get(name, name)
        resource.full_name = name
        resource.namespace = name.rsplit("/", 1)[0]
        if name in self.resources:
            if not name in self.collisions:
                self.collisions[name] = []
            self.collisions[name].append(resource)
        else:
            self.resources[name] = resource

    def remap(self, source, target):
        self.remaps[source] = target

    def child(self):
        return ResourceGraph(resources = self.resources,
                             remaps = dict(self.remaps),
                             collisions = self.collisions)


###############################################################################
# ROS Model - Configuration
###############################################################################

# A configuration is more or less equivalent to an application.
# It is the result of a launch file, plus environment, parameters, etc.
# A configuration becomes invalid if there are unknown references.

class Configuration(object):
    def __init__(self, name, env, resources = None):
        self.name = name
        self.environment = env
        self.resources = ResourceGraph() if resources is None else resources
        self.valid = True
        self.pkg_depends = set()
        self.env_depends = set()

    def nodes(self):
        return self.resources.get_nodes()


###############################################################################
# Helper Functions
###############################################################################

def resolve_name(name, ns = "/", private_ns = ""):
    if name[0] == "~":
        return private_ns + "/" + name[1:]
    elif name[0] == "/":
        return name
    elif ns != "/":
        return ns + "/" + name
    else:
        return ns + name


def apply_remaps(name, remaps):
    """NOTE: It seems remappings do not work this way after all.
        My tests show only one level of transformations.
    """
    remap = remaps.get(name)
    while not remap is None:
        name = remap
        remap = remaps.get(name)
    return name


def transform_name(name, ns = "/", private_ns = "", remaps = None):
    name = resolve_name(name, ns = ns, private_ns = private_ns)
    if remaps:
        return remaps.get(name, name)
    return name


###############################################################################
# Testing
###############################################################################

if __name__ == "__main__":
    assert resolve_name("bar", ns="/", private_ns="/node1") == "/bar"
    assert resolve_name("/bar", ns="/", private_ns="/node1") == "/bar"
    assert resolve_name("~bar", ns="/", private_ns="/node1") == "/node1/bar"
    assert resolve_name("bar", ns="/wg", private_ns="/wg/node2") == "/wg/bar"
    assert resolve_name("/bar", ns="/wg", private_ns="/wg/node2") == "/bar"
    assert resolve_name("~bar", ns="/wg", private_ns="/wg/node2") == "/wg/node2/bar"
    assert resolve_name("foo/bar", ns="/wg", private_ns="/wg/node3") == "/wg/foo/bar"
    assert resolve_name("/foo/bar", ns="/wg", private_ns="/wg/node3") == "/foo/bar"
    assert resolve_name("~foo/bar", ns="/wg", private_ns="/wg/node3") == "/wg/node3/foo/bar"
    print "All tests passed!"
