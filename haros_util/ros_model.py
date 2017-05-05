
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


class Topic(Resource):
    def __init__(self, name, ns = "/", private_ns = ""):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.publishers = []
        self.subscribers = []


class Service(Resource):
    def __init__(self, name, ns = "/", private_ns = ""):
        Resource.__init__(self, name, ns = ns, private_ns = private_ns)
        self.server = None
        self.clients = []


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

    def get_node(self, name, remap = False):
        return self._get(name, Node, remap)

    def get_nodes(self):
        return self._get_all(Node)

    def get_param(self, name, remap = False):
        return self._get(name, Parameter, remap)

    def get_params(self):
        return self._get_all(Parameter)

    def get_topic(self, name, remap = False):
        return self._get(name, Topic, remap)

    def get_topics(self):
        return self._get_all(Topic)

    def get_service(self, name, remap = False):
        return self._get(name, Service, remap)

    def get_services(self):
        return self._get_all(Service)

    def _get(self, name, cls, remap):
        if remap:
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
        remap = self.remaps.get(name)
        while not remap is None:
            name = remap
            remap = self.remaps.get(name)
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
