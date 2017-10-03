
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

    def add_publisher(self, topic, message_type, queue_size = 0, nesting = 0,
                      global_ref = False, file = None, line = None):
        self.publishers.append((topic, message_type, queue_size, nesting,
                                file, line, global_ref))

    def add_subscriber(self, topic, message_type, queue_size = 0, nesting = 0,
                       global_ref = False, file = None, line = None):
        self.subscribers.append((topic, message_type, queue_size, nesting,
                                 file, line, global_ref))

    def add_server(self, service, message_type, nesting = 0,
                   global_ref = False, file = None, line = None):
        self.servers.append((service, message_type, nesting, file, line,
                             global_ref))

    def add_client(self, service, message_type, nesting = 0,
                   global_ref = False, file = None, line = None):
        self.clients.append((service, message_type, nesting, file, line,
                             global_ref))


class Topic(Resource):
    def __init__(self, name, ns = "/", private_ns = "", message_type = None):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.message_type = message_type
        self.publishers = []
        self.subscribers = []

    @property
    def is_disconnected(self):
        p = len(self.publishers)
        s = len(self.subscribers)
        return p + s > 0 and (p == 0 or s == 0)

    def add_publisher(self, node, message_type, queue_size = 0, nesting = 0,
                      global_ref = False, file = None, line = None):
        self.publishers.append((node, message_type, queue_size, nesting,
                                file, line, global_ref))

    def add_subscriber(self, node, message_type, queue_size = 0, nesting = 0,
                       global_ref = False, file = None, line = None):
        self.subscribers.append((node, message_type, queue_size, nesting,
                                 file, line, global_ref))


class Service(Resource):
    def __init__(self, name, ns = "/", private_ns = "", message_type = None):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.message_type = message_type
        self.server = None
        self.clients = []

    @property
    def is_disconnected(self):
        s = 1 if not self.server is None else 0
        c = len(self.clients)
        return s + c > 0 and (s == 0 or c == 0)

    def set_server(self, node, message_type, nesting = 0,
                   global_ref = False, file = None, line = None):
        self.server = (node, message_type, nesting, file, line,
                       global_ref)

    def add_client(self, node, message_type, nesting = 0,
                   global_ref = False, file = None, line = None):
        self.clients.append((node, message_type, nesting, file, line,
                             global_ref))


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
        self.resources  = resources if not resources is None else {}
        self.collisions = collisions if not collisions is None else {}
        self.remaps     = remaps if not remaps is None else {}

    @property
    def n_collisions(self):
        n = 0
        for name, values in self.collisions.iteritems():
            n += len(values)
        return n

    @property
    def n_remaps(self):
        n = 0
        for name, values in self.remaps.iteritems():
            n += len(values)
        return n

    def get_node(self, name, remaps = False):
        return self._get(name, Node, remaps)

    def get_nodes(self, count = False):
        return self._count_all(Node) if count else self._get_all(Node)

    def get_param(self, name, remaps = False):
        return self._get(name, Parameter, remaps)

    def get_params(self, count = False):
        return self._count_all(Parameter) if count else self._get_all(Parameter)

    def get_topic(self, name, remaps = False):
        return self._get(name, Topic, remaps)

    def get_topics(self, count = False):
        return self._count_all(Topic) if count else self._get_all(Topic)

    def get_service(self, name, remaps = False):
        return self._get(name, Service, remaps)

    def get_services(self, count = False):
        return self._count_all(Service) if count else self._get_all(Service)

    def _get(self, name, cls, remaps):
        if isinstance(remaps, dict):
            name = remaps.get(name, name)
        elif remaps:
            values = self.remaps.get(name)
            if values:
                name = values[-1]
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

    def _count_all(self, cls):
        n = 0
        for _, r in self.resources.iteritems():
            if isinstance(r, cls):
                n += 1
        for _, rs in self.collisions.iteritems():
            for r in rs:
                if isinstance(r, cls):
                    n += 1
        return n

    def register(self, resource, remaps = False):
        name = resource.full_name
        if isinstance(remaps, dict):
            name = remaps.get(name, name)
            resource.full_name = name
        elif remaps:
            values = self.remaps.get(name)
            if values:
                name = values[-1]
                resource.full_name = name
        resource.namespace = name.rsplit("/", 1)[0]
        if name in self.resources:
            if not name in self.collisions:
                self.collisions[name] = []
            self.collisions[name].append(resource)
        else:
            self.resources[name] = resource

    def remap(self, source, target):
        values = self.remaps.get(source, [])
        values.append(target)
        self.remaps[source] = values

    def remove_param(self, name, remaps = False):
        if isinstance(remaps, dict):
            name = remaps.get(name, name)
        elif remaps:
            values = self.remaps.get(name)
            if values:
                name = values[-1]
    # ----- remove from collisions first to ease the shift
    # ----- if it is in resources too
        rs = self.collisions.get(name)
        if rs:
            i = len(rs) - 1
            while i >= 0:
                if isinstance(r, Parameter):
                    del rs[i]
                i -= 1
            if not rs:
                del self.collisions[name]
        r = self.resources.get(name)
        if isinstance(r, Parameter):
            del self.resources[name]
            rs = self.collisions.get(name)
            if rs:
                self.resources[name] = rs.pop(0)
                if not rs:
                    del self.collisions[name]

    def merge(self, other):
        for resource in other.resources.itervalues():
            self.register(resource)
        for resources in other.collisions.itervalues():
            for resource in resources:
                self.register(resource)
        for source, targets in other.remaps.iteritems():
            for target in targets:
                self.remap(source, target)


###############################################################################
# ROS Model - Configuration
###############################################################################

# A configuration is more or less equivalent to an application.
# It is the result of a launch file, plus environment, parameters, etc.
# A configuration becomes invalid if there are unknown references.

class Configuration(object):
    def __init__(self, name, package, env, resources = None):
        self.name = name
        self.package = package
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


def is_global_name(name):
    return name and name[0] == "/"


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
