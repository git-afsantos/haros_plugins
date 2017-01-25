
###
# standard packages
from collections import namedtuple

###
# internal packages
from clang_plugin.cpp_model import CppBlock, CppControlFlow, CppFunction, \
                                   CppFunctionCall, CppGlobalScope, \
                                   CppOperator, CppVariable

###############################################################################
# Collectibles
###############################################################################

SubscribeTuple = namedtuple("SubscribeTuple",
                            ["topic", "queue_size", "callback",
                             "nesting", "variable", "function", "line"])

AdvertiseTuple = namedtuple("AdvertiseTuple",
                            ["topic", "queue_size", "message_type",
                             "nesting", "variable", "function", "line"])

PublishTuple = namedtuple("PublishTuple",
                          ["variable", "nesting", "function", "line", "scope"])

AdvertiseServiceTuple = namedtuple("AdvertiseServiceTuple",
                                   ["topic", "callback", "nesting",
                                    "variable", "function", "line"])

ServiceClientTuple = namedtuple("ServiceClientTuple",
                                ["topic", "message_type", "nesting",
                                 "variable", "function", "line"])

SpinRateTuple = namedtuple("SpinRateTuple",
                           ["rate", "variable", "function", "line"])



###############################################################################
# File Collector
###############################################################################

class FileCollector(object):
    def __init__(self, function_collectors):
        self.subscribe          = {}
        self.subscribers        = {}
        self.advertise          = {}
        self.publish            = {}
        self.publishers         = {}
        self.advertise_service  = {}
        self.servers            = {}
        self.service_client     = {}
        self.clients            = {}
        self.spin_rate          = {}
        self.publish_rate       = {}
        self._collect(function_collectors)

    @classmethod
    def from_global_scope(cls, global_scope):
        assert isinstance(global_scope, CppGlobalScope)
        data = [FunctionCollector(f) for f in collect_functions(global_scope)]
        return FileCollector(data)

    def _collect(self, function_collectors):
        self._basic_occurrences(function_collectors)
        self._publish_occurrences(function_collectors)
        self._publish_rates(function_collectors)

    def _basic_occurrences(self, function_collectors):
        for c in function_collectors:
            for var in c.spin_rate:
                self.spin_rate[var.variable] = var
            for var in c.subscribe:
                self.subscribe[var.topic] = var
                if var.variable:
                    self.subscribers[var.variable] = var.topic
            for var in c.advertise:
                self.advertise[var.topic] = var
                if var.variable:
                    self.publishers[var.variable] = var.topic
            for var in c.advertise_service:
                self.advertise_service[var.topic] = var
                if var.variable:
                    self.servers[var.variable] = var.topic
            for var in c.service_client:
                self.service_client[var.topic] = var
                if var.variable:
                    self.clients[var.variable] = var.topic

    def _publish_occurrences(self, function_collectors):
        for c in function_collectors:
            for call in c.publish:
                if call.variable in self.publishers:
                    if not call.variable in self.publish:
                        self.publish[call.variable] = []
                    self.publish[call.variable].append(call)

    def _publish_rates(self, function_collectors):
        # TODO this needs a serious rework
        for c in function_collectors:
            for sleep in c.sleep:
                rate = 0
                if sleep.method_of.name in self.spin_rate:
                    rate = self.spin_rate[sleep.method_of.name].rate
                sleep_scope = sleep.scope
                while isinstance(sleep_scope, CppBlock):
                    sleep_scope = sleep_scope.scope
                for var, publish in self.publish.iteritems():
                    topic = self.publishers[var]
                    # size = self.advertise[topic].queue_size
                    for p in publish:
                        pub_scope = p.scope
                        while isinstance(pub_scope, CppBlock):
                            pub_scope = pub_scope.scope
                        if p.function == c.function or sleep_scope == pub_scope \
                                or p.function in c.function_set:
                            self.publish_rate[topic] = rate



###############################################################################
# Function Collector
###############################################################################

class FunctionCollector(object):
    def __init__(self, function):
        assert isinstance(function, CppFunction)
        self.function           = function.name
        self.subscribe          = []
        self.advertise          = []
        self.publish            = []
        self.advertise_service  = []
        self.service_client     = []
        self.spin_rate          = []
        self.sleep              = []
        self.function_set       = set()
        for statement in function.body.body:
            self._collect(statement)

    def _collect(self, statement, nesting = 0):
        if isinstance(statement, CppVariable):
            self._from_variable(statement, nesting)
        elif isinstance(statement, CppFunctionCall):
            self._from_call(statement, nesting)
        elif isinstance(statement, CppControlFlow):
            # TODO reduce nesting on "else if"
            for branch in statement.branches:
                for stmt in branch[1].body:
                    self._collect(stmt, nesting = nesting + 1)

    def _from_variable(self, variable, nesting):
        name = variable.name
        if variable.result == "ros::Rate":
            if isinstance(variable.value, CppFunctionCall):
                if len(variable.value.arguments) > 0:
                    value = variable.value.arguments[0]
                else:
                    value = 0
                o = SpinRateTuple(value, name, self.function, variable.line)
                self.spin_rate.append(o)
        elif variable.result == "ros::Publisher" \
                or variable.result == "ros::Subscriber" \
                or variable.result == "ros::ServiceServer" \
                or variable.result == "ros::ServiceClient":
            if isinstance(variable.value, CppFunctionCall):
                self._from_call(variable.value, nesting, variable = name)

    def _from_call(self, call, nesting, variable = None):
        name = call.name
        if name == "operator=":
            call.simplify()
            if isinstance(call.arguments[1], CppFunctionCall):
                self._from_call(call.arguments[1], nesting,
                                variable = call.arguments[0].name)
            return
        self.function_set.add(name)
        if name == "advertise" and call.result == "ros::Publisher":
            call.simplify()
            o = AdvertiseTuple(call.arguments[0], call.arguments[1],
                               call.template, nesting, variable,
                               self.function, call.line)
            self.advertise.append(o)
        elif name == "subscribe" and call.result == "ros::Subscriber":
            call.simplify()
            callback = call.arguments[2]
            if isinstance(callback, CppOperator) and callback.is_unary:
                callback = callback.arguments[0]
            o = SubscribeTuple(call.arguments[0], call.arguments[1],
                               callback.name, nesting, variable,
                               self.function, call.line)
            self.subscribe.append(o)
        elif name == "publish" and call.method_of \
                and call.method_of.result == "ros::Publisher":
            call.simplify()
            o = PublishTuple(call.method_of.name, nesting,
                             self.function, call.line, call.scope)
            self.publish.append(o)
        elif name == "advertiseService" and call.result == "ros::ServiceServer":
            call.simplify()
            callback = call.arguments[1]
            if isinstance(callback, CppOperator) and callback.is_unary:
                callback = callback.arguments[0]
            o = AdvertiseServiceTuple(call.arguments[0], callback.name, nesting,
                                      variable, self.function, call.line)
            self.advertise_service.append(o)
        elif name == "serviceClient" and call.result == "ros::ServiceClient":
            call.simplify()
            o = ServiceClientTuple(call.arguments[0], call.template, nesting,
                                   variable, self.function, call.line)
            self.service_client.append(o)
        elif name == "sleep" and call.method_of \
                and call.method_of.result == "ros::Rate":
            call.simplify()
            self.sleep.append(call)

    def has_results(self):
        return self.subscribe or self.advertise or self.publish \
                or self.advertise_service or self.service_client \
                or self.spin_rate or self.sleep

    def show_results(self):
        for r in self.subscribe:
            print "[{}] {} = subscribe({}, {}, {})".format(r[3], r[4], r[0], r[1], r[2])
        for r in self.advertise:
            print "[{}] {} = advertise<{}>({}, {})".format(r[3], r[4], r[2], r[0], r[1])
        for r in self.publish:
            print "[{}] {}.publish()".format(r[1], r[0])
        for r in self.advertise_service:
            print "[{}] {} = advertiseService({}, {})".format(r[2], r[3], r[0], r[1])
        for r in self.service_client:
            print "[{}] {} = serviceClient<{}>({})".format(r[2], r[3], r[1], r[0])
        for r in self.spin_rate:
            print r[0] + " spinning at " + str(r[1]) + "Hz"



###############################################################################
# Helper Functions
###############################################################################

def collect_functions(scope = None):
    global_scope = scope or CppGlobalScope.INSTANCE
    all_functions = list(global_scope.functions)
    for n in global_scope.namespaces:
        all_functions.extend(n.functions)
        for c in n.classes:
            all_functions.extend(c.methods)
    for c in global_scope.classes:
        all_functions.extend(c.methods)
    return all_functions
