
###
# standard packages
from collections import namedtuple, Counter

###
# internal packages
from clang_plugin.cpp_model import CppBlock, CppControlFlow, CppFunction, \
                                   CppFunctionCall, CppDefaultArgument, \
                                   CppGlobalScope, CppOperator, CppVariable, \
                                   CppStatement, CppExpression

###############################################################################
# Collectibles
###############################################################################

advertise_count = 0
subscribe_count = 0
pubsub_var = 0
collected_functions = 0

SUB_TYPE_1 = "subscribe(string topic, uint32_t queue_size, " \
             "void method(M), T *obj, TransportHints)"
SUB_TYPE_2 = "subscribe(string topic, uint32_t queue_size, " \
             "void function(M), TransportHints)"
SUB_TYPE_3 = "subscribe(string topic, uint32_t queue_size, " \
             "void boost::function(M), VoidConstPtr, TransportHints)"
SUB_TYPE_4 = "subscribe(SubscribeOptions)"

SubscribeTuple = namedtuple("SubscribeTuple",
                            ["topic", "queue_size", "callback", "message_type",
                             "nesting", "variable", "function", "line",
                             "overload", "transport_hints", "param_topic"])

ADV_TYPE_1 = "advertise(string topic, uint32_t queue_size, bool latch)"
ADV_TYPE_2 = "advertise(string topic, uint32_t queue_size, " \
             "SubscriberStatusCallback, SubscriberStatusCallback, " \
             "VoidConstPtr, bool latch)"
ADV_TYPE_3 = "advertise(AdvertiseOptions)"

AdvertiseTuple = namedtuple("AdvertiseTuple",
                            ["topic", "queue_size", "message_type",
                             "nesting", "variable", "function", "line",
                             "overload", "latch", "param_topic"])

PublishTuple = namedtuple("PublishTuple",
                          ["variable", "nesting", "function", "line", "scope"])

ADV_SRV_TYPE_1 = "advertiseService(string service, " \
                 "bool method(MReq, MRes), T *obj)"
ADV_SRV_TYPE_2 = "advertiseService(string service, bool function(MReq, MRes))"
ADV_SRV_TYPE_3 = "advertiseService(string service, " \
                 "bool boost::function(MReq, MRes), VoidConstPtr)"
ADV_SRV_TYPE_4 = "advertiseService(AdvertiseServiceOptions)"

AdvertiseServiceTuple = namedtuple("AdvertiseServiceTuple",
                                   ["topic", "callback", "message_type",
                                    "nesting", "variable", "function", "line",
                                    "overload", "service_event", "param_topic"])

SRV_CLI_TYPE_1 = "serviceClient(string service, bool persistent, M_string)"
SRV_CLI_TYPE_2 = "serviceClient(ServiceClientOptions)"

ServiceClientTuple = namedtuple("ServiceClientTuple",
                                ["topic", "message_type", "nesting",
                                 "variable", "function", "line", "overload",
                                 "param_topic"])

SpinRateTuple = namedtuple("SpinRateTuple",
                           ["rate", "variable", "function", "line"])

ParamTuple = namedtuple("ParamTuple", ["param", "variable", "default",
                                       "param_type", "function", "line"])



###############################################################################
# Statistics
###############################################################################

class SubscriberStatistics(object):
    def __init__(self):
        self.hardcoded_topics   = 0
        self.reference_topics   = 0
        self.call_topics        = 0
        self.operator_topics    = 0
        self.param_topics       = 0
        self.hardcoded_queues   = []
        self.infinite_queues    = 0
        self.msg_to_queue       = {}
        self.nesting            = []
        self.global_topics      = 0
        self.transport_hints    = 0
        self.function_callbacks = 0
        self.method_callbacks   = 0
        self.boost_callbacks    = 0

    @property
    def total_subscribers(self):
        return len(self.nesting)

    def collect_subscribe(self, datum):
        if isinstance(datum.topic, basestring):
            self.hardcoded_topics += 1
            if datum.topic.startswith("/"):
                self.global_topics += 1
        elif isinstance(datum.topic, CppReference):
            self.reference_topics += 1
        elif isinstance(datum.topic, CppOperator):
            self.operator_topics += 1
        elif isinstance(datum.topic, CppFunctionCall):
            self.call_topics += 1
        if datum.param_topic:
            self.param_topics += 1
        self.nesting.append(datum.nesting)
        if isinstance(datum.queue_size, (int, long)):
            self.hardcoded_queues.append(datum.queue_size)
            if datum.queue_size == 0:
                self.infinite_queues += 1
            self._collect_message(datum)
        if datum.transport_hints:
            self.transport_hints += 1
        if datum.overload == SUB_TYPE_1:
            self.method_callbacks += 1
        elif datum.overload == SUB_TYPE_2:
            self.function_callbacks += 1
        elif datum.overload == SUB_TYPE_3:
            self.boost_callbacks += 1

    def _collect_message(self, datum):
        m = datum.message_type
        q = datum.queue_size
        if not m in self.msg_to_queue:
            self.msg_to_queue[m] = []
        self.msg_to_queue[m].append(q)


    _SUBSCRIBE_HEADERS = [
        "Subscribe Calls", "Hard-coded Topics", "Reference Topics",
        "Function Call Topics", "Operator Topics", "Param Topics",
        "Global Topics",
        "Use of TransportHints", "Function Callbacks", "Method Callbacks",
        "Boost Function Callbacks", "Hard-coded Queue Sizes", "Infinite Queues",
        "Median Queue Size", "Min. Queue Size", "Max. Queue Size",
        "Median Control Nesting", "Min. Control Nesting", "Max. Control Nesting"
    ]

    def csv_subscribe(self):
        rows = [SubscriberStatistics._SUBSCRIBE_HEADERS]
        if not self.total_subscribers:
            return rows
        data = [self.total_subscribers, self.hardcoded_topics,
                self.reference_topics, self.call_topics, self.operator_topics,
                self.param_topics, self.global_topics, self.transport_hints,
                self.function_callbacks, self.method_callbacks,
                self.boost_callbacks]
        if self.hardcoded_queues:
            data.extend([len(self.hardcoded_queues), self.infinite_queues,
                         median(self.hardcoded_queues),
                         min(self.hardcoded_queues), max(self.hardcoded_queues)])
        else:
            data.extend([0, 0, None, None, None])
        if self.nesting:
            data.extend([median(self.nesting), min(self.nesting),
                         max(self.nesting)])
        else:
            data.extend([None, None, None])
        rows.append(data)
        return rows


class PublisherStatistics(object):
    def __init__(self):
        self.hardcoded_topics   = 0
        self.reference_topics   = 0
        self.call_topics        = 0
        self.operator_topics    = 0
        self.param_topics       = 0
        self.latching_topics    = 0
        self.hardcoded_queues   = []
        self.infinite_queues    = 0
        self.msg_to_queue       = {}
        self.advertise_nesting  = []
        self.publish_nesting    = []
        self.global_topics      = 0
        self.subscriber_status  = 0

    @property
    def total_publishers(self):
        return len(self.advertise_nesting)

    def collect_advertise(self, datum):
        if isinstance(datum.topic, basestring):
            self.hardcoded_topics += 1
            if datum.topic.startswith("/"):
                self.global_topics += 1
        elif isinstance(datum.topic, CppReference):
            self.reference_topics += 1
        elif isinstance(datum.topic, CppOperator):
            self.operator_topics += 1
        elif isinstance(datum.topic, CppFunctionCall):
            self.call_topics += 1
        if datum.param_topic:
            self.param_topics += 1
        self.advertise_nesting.append(datum.nesting)
        if isinstance(datum.queue_size, (int, long)):
            self.hardcoded_queues.append(datum.queue_size)
            if datum.queue_size == 0:
                self.infinite_queues += 1
            self._collect_message(datum)
        if datum.latch:
            self.latching_topics += 1
        if datum.overload == ADV_TYPE_2:
            self.subscriber_status += 1

    def collect_publish(self, datum):
        self.publish_nesting.append(datum.nesting)

    def _collect_message(self, datum):
        m = datum.message_type
        q = datum.queue_size
        if not m in self.msg_to_queue:
            self.msg_to_queue[m] = []
        self.msg_to_queue[m].append(q)


    _PUBLISH_HEADERS = [
        "Advertise Calls", "Hard-coded Topics", "Reference Topics",
        "Function Call Topics", "Operator Topics", "Param Topics",
        "Global Topics",
        "Latching Publishers", "Use of SubscriberStatus",
        "Hard-coded Queue Sizes", "Infinite Queues", "Median Queue Size",
        "Min. Queue Size", "Max. Queue Size",
        "Median Control Nesting", "Min. Control Nesting", "Max. Control Nesting",
        "Publish Calls", "Median Control Nesting",
        "Min. Control Nesting", "Max. Control Nesting"
    ]

    def csv_publish(self):
        rows = [PublisherStatistics._PUBLISH_HEADERS]
        data = [self.total_publishers, self.hardcoded_topics,
                self.reference_topics, self.call_topics, self.operator_topics,
                self.param_topics, self.global_topics,
                self.latching_topics, self.subscriber_status]
        if self.hardcoded_queues:
            data.extend([len(self.hardcoded_queues), self.infinite_queues,
                         median(self.hardcoded_queues),
                         min(self.hardcoded_queues), max(self.hardcoded_queues)])
        else:
            data.extend([0, 0, None, None, None])
        if self.advertise_nesting:
            data.extend([median(self.advertise_nesting),
                         min(self.advertise_nesting), max(self.advertise_nesting)])
        else:
            data.extend([None, None, None])
        if self.publish_nesting:
            data.extend([len(self.publish_nesting), median(self.publish_nesting),
                         min(self.publish_nesting), max(self.publish_nesting)])
        else:
            data.extend([0, None, None, None])
        rows.append(data)
        return rows


class RpcStatistics(object):
    def __init__(self):
        self.hardcoded_topics   = 0
        self.reference_topics   = 0
        self.call_topics        = 0
        self.operator_topics    = 0
        self.param_topics       = 0
        self.server_nesting     = []
        self.client_nesting     = []
        self.global_topics      = 0
        self.function_callbacks = 0
        self.method_callbacks   = 0
        self.boost_callbacks    = 0

    @property
    def total_rpc(self):
        return len(self.server_nesting) + len(self.client_nesting)

    def collect_server(self, datum):
        if isinstance(datum.topic, basestring):
            self.hardcoded_topics += 1
            if datum.topic.startswith("/"):
                self.global_topics += 1
        elif isinstance(datum.topic, CppReference):
            self.reference_topics += 1
        elif isinstance(datum.topic, CppOperator):
            self.operator_topics += 1
        elif isinstance(datum.topic, CppFunctionCall):
            self.call_topics += 1
        if datum.param_topic:
            self.param_topics += 1
        self.server_nesting.append(datum.nesting)
        if datum.overload == ADV_SRV_TYPE_1:
            self.method_callbacks += 1
        elif datum.overload == ADV_SRV_TYPE_2:
            self.function_callbacks += 1
        elif datum.overload == ADV_SRV_TYPE_3:
            self.boost_callbacks += 1

    def collect_client(self, datum):
        if isinstance(datum.topic, basestring):
            self.hardcoded_topics += 1
            if datum.topic.startswith("/"):
                self.global_topics += 1
        self.client_nesting.append(datum.nesting)


    _RPC_HEADERS = [
        "Total Services", "Hard-coded Services", "Reference Services",
        "Function Call Services", "Operator Services", "Param Services",
        "Global Services",
        "Service Servers", "Function Callbacks", "Method Callbacks",
        "Boost Function Callbacks", "Median Control Nesting",
        "Min. Control Nesting", "Max. Control Nesting",
        "Service Clients", "Median Control Nesting",
        "Min. Control Nesting", "Max. Control Nesting"
    ]

    def csv_service(self):
        rows = [RpcStatistics._RPC_HEADERS]
        if self.total_rpc:
            data = [self.total_rpc, self.hardcoded_topics,
                    self.reference_topics, self.call_topics,
                    self.operator_topics, self.param_topics, self.global_topics,
                    len(self.server_nesting), self.function_callbacks,
                    self.method_callbacks, self.boost_callbacks
            ]
            if self.server_nesting:
                data.extend([median(self.server_nesting),
                        min(self.server_nesting), max(self.server_nesting)])
            else:
                data.extend([None, None, None])
            if self.client_nesting:
                data.extend([len(self.client_nesting),
                        median(self.client_nesting),
                        min(self.client_nesting), max(self.client_nesting)])
            else:
                data.extend([0, None, None, None])
            rows.append(data)
        return rows


class ParamStatistics(object):
    def __init__(self):
        self.total_params       = 0
        self.hardcoded_params   = 0
        self.global_params      = 0
        self.defaults           = 0
        self.param_type         = Counter()

    def collect_param(self, datum):
        self.total_params += 1
        if isinstance(datum.param, basestring):
            self.hardcoded_params += 1
            if datum.param.startswith("/"):
                self.global_params += 1
            if datum.default:
                self.defaults += 1
        if datum.param_type:
            self.param_type[datum.param_type] += 1

    _PARAM_HEADERS = [
        "Total Parameters", "Hard-coded Parameters",
        "Global Parameters", "Uses Default Value"
    ]

    def csv_param(self):
        rows = [ParamStatistics._PARAM_HEADERS]
        if self.total_params:
            rows.append([self.total_params, self.hardcoded_params,
                self.global_params, self.defaults
            ])
        return rows

    _TYPE_HEADERS = ["Parameter Type", "Count"]

    def csv_param_type(self):
        rows = [[t, c] for t, c in self.param_type.iteritems()]
        rows.insert(0, ParamStatistics._TYPE_HEADERS)
        return rows



###############################################################################
# Visitor Collector
###############################################################################

class VisitorCollector(object):
    def __init__(self):
        self.pub                = PublisherStatistics()
        self.sub                = SubscriberStatistics()
        self.rpc                = RpcStatistics()
        self.param              = ParamStatistics()
        self.total_rates        = 0
        self.hardcoded_rates    = []
        self.function_calls     = []
        self.distinct_calls     = []

    @property
    def total_pub_sub(self):
        return self.sub.total_subscribers + self.pub.total_publishers

    @property
    def total_rpc(self):
        return self.rpc.total_rpc

    @property
    def global_topics(self):
        return self.pub.global_topics + self.sub.global_topics \
               + self.rpc.global_topics

    @property
    def function_callbacks(self):
        return self.sub.function_callbacks + self.rpc.function_callbacks

    @property
    def method_callbacks(self):
        return self.sub.method_callbacks + self.rpc.method_callbacks

    @property
    def boost_callbacks(self):
        return self.sub.boost_callbacks + self.rpc.boost_callbacks

    def _from_variable(self, variable, nesting):
        global pubsub_var
        name = variable.name
        if variable.result == "ros::Rate":
            if isinstance(variable.value, CppFunctionCall):
                value = None
                if len(variable.value.arguments) > 0:
                    value = variable.value.arguments[0]
                o = SpinRateTuple(value, name, self.function, variable.line)
                self.spin_rate.append(o)
        elif variable.result == "ros::Publisher" \
                or variable.result == "ros::Subscriber" \
                or variable.result == "ros::ServiceServer" \
                or variable.result == "ros::ServiceClient":
            if variable.result == "ros::Publisher" or variable.result == "ros::Subscriber":
                pubsub_var += 1
            self._collect(variable.value, nesting, variable = name)
        elif isinstance(variable.value, CppFunctionCall) \
                and variable.value.name == "param":
            self._from_call(variable.value, nesting, variable = name)

    _FUNCTION_NAMES = ("advertise", "subscribe", "publish", "advertiseService",
                       "serviceClient", "sleep", "param", "getParam", "Rate",
                       "getParamCached")

    def visit_function_call(self, call):
        global advertise_count
        global subscribe_count
        # TODO a function map seems better by now...
        name = call.name
        if name in VisitorCollector._FUNCTION_NAMES:
            call.simplify()
            nest = control_nesting(call)

        if name == "operator=":
            call.simplify()
            if len(call.arguments) >= 2 and isinstance(call.arguments[1], CppFunctionCall):
                self._from_call(call.arguments[1], nesting,
                                variable = call.arguments[0].name)
            elif len(call.arguments) < 2:
                print "[CLANG]", name, call.arguments
            return
        self.function_set.add(name)
        self.function_calls += 1
        if name == "advertise" and call.result == "ros::Publisher":
            advertise_count += 1
            self._advertise_call(call, nesting, variable = variable)
        elif name == "subscribe" and call.result == "ros::Subscriber":
            subscribe_count += 1
            self._subscribe_call(call, nesting, variable = variable)
        elif name == "publish" and call.method_of \
                and call.method_of.result == "ros::Publisher":
            call.simplify()
            o = PublishTuple(call.method_of.name, nesting,
                             self.function, call.line, call.scope)
            self.publish.append(o)
        elif name == "advertiseService" and call.result == "ros::ServiceServer":
            self._advertise_service_call(call, nesting, variable = variable)
        elif name == "serviceClient" and call.result == "ros::ServiceClient":
            self._service_client_call(call, nesting, variable = variable)
        elif name == "sleep" and call.method_of \
                and call.method_of.result == "ros::Rate":
            call.simplify()
            self.sleep.append(call)
        elif name == "param":
            call.simplify()
            self._param_call(call, nesting, variable = variable)
        elif name == "getParam" or name == "getParamCached":
            call.simplify()
            self._get_param_call(call, nesting, variable = variable)


    def _advertise_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        call.simplify()
        topic       = None
        queue_size  = None
        latch       = False
        if nargs == 1:
            atype = ADV_TYPE_3
        else:
            assert nargs == 3 or nargs == 6
            atype = ADV_TYPE_1 if nargs == 3 else ADV_TYPE_2
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            latch       = call.arguments[-1]
            latch       = latch if isinstance(latch, bool) else None
        self.advertise.append(AdvertiseTuple(topic, queue_size,
                call.template, nesting, variable, self.function, call.line,
                atype, latch))

    def _subscribe_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        call.simplify()
        topic       = None
        queue_size  = None
        callback    = None
        msg_type    = None
        hints       = False
        if nargs == 1:
            # assert call.arguments[0].result == "ros::SubscribeOptions"
            stype = SUB_TYPE_4
        else:
            # nargs == 4 (function) or nargs == 5 (method/boost)
            assert nargs == 4 or nargs == 5
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            cb          = call.arguments[2]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            if nargs == 4:
                stype = SUB_TYPE_2
            else:
                if isinstance(call.arguments[3], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    stype = SUB_TYPE_3
                else:
                    stype = SUB_TYPE_1
            hints = not isinstance(call.arguments[-1], CppDefaultArgument)
        self.subscribe.append(SubscribeTuple(topic, queue_size, callback,
                msg_type, nesting, variable, self.function,
                call.line, stype, hints))

    def _advertise_service_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        call.simplify()
        service     = None
        callback    = None
        msg_type    = None
        events      = False
        if nargs == 1:
            atype = ADV_SRV_TYPE_4
        else:
            # nargs == 2 (function) or nargs == 3 (method/boost)
            assert nargs == 2 or nargs == 3
            service = call.arguments[0]
            cb = call.arguments[1]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            events = "ros::ServiceEvent" in cb.result
            if nargs == 2:
                atype = ADV_SRV_TYPE_2
            else:
                if isinstance(call.arguments[2], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    atype = ADV_SRV_TYPE_3
                else:
                    atype = ADV_SRV_TYPE_1
        self.advertise_service.append(AdvertiseServiceTuple(service,
                callback, msg_type, nesting, variable, self.function, call.line,
                atype, events))

    def _service_client_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        call.simplify()
        service = None
        ctype = SRV_CLI_TYPE_1
        if nargs == 1:
            if "ros::ServiceClientOptions" in call.arguments[0].result:
                ctype = SRV_CLI_TYPE_2
        else:
            service = call.arguments[0]
        self.service_client.append(ServiceClientTuple(service,
                call.template, nesting, variable, self.function,
                call.line, ctype))

    def _param_call(self, call, nesting, variable = None):
        n = len(call.arguments)
        if n == 2:      # T param(std::string &key, T &default_val)
            result = call.result
        elif n == 3:    # void param(std::string &key, T &val, T &default_val)
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
        self.ros_parameters.append(ParamTuple(call.arguments[0],
                variable, True, result, self.function, call.line))

    def _get_param_call(self, call, nesting, variable = None):
        if len(call.arguments) >= 2:
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
            self.ros_parameters.append(ParamTuple(call.arguments[0],
                    variable, False, result, self.function, call.line))


    def _msg_type_from_callback(self, callback):
        type_string = callback.result
        type_string = type_string.split(None, 1)[1]
        if not (type_string[0] == "(" and type_string[-1] == ")"):
            print "[CLANG] Unknown message type:", type_string
            return type_string
        type_string = type_string[1:-1]
        is_const = type_string.startswith("const ")
        if is_const:
            type_string = type_string[6:]
        is_ref = type_string.endswith(" &")
        if is_ref:
            type_string = type_string[:-2]
        is_ptr = type_string.endswith("::ConstPtr")
        if is_ptr:
            type_string = type_string[:-10]
        else:
            is_ptr = type_string.endswith("ConstPtr")
            if is_ptr:
                type_string = type_string[:-8]
        return type_string


    def collect(self, function_collector):
        for datum in function_collector.subscribe:
            self.sub.collect_subscribe(datum)
        for datum in function_collector.advertise:
            self.pub.collect_advertise(datum)
        for datum in function_collector.publish:
            self.pub.collect_publish(datum)
        for datum in function_collector.advertise_service:
            self.rpc.collect_server(datum)
        for datum in function_collector.service_client:
            self.rpc.collect_client(datum)
        for datum in function_collector.ros_parameters:
            self.param.collect_param(datum)
        for datum in function_collector.spin_rate:
            self.total_rates += 1
            if isinstance(datum.rate, (int, long)):
                self.hardcoded_rates.append(datum.rate)
        self.function_calls.append(function_collector.function_calls)
        self.distinct_calls.append(len(function_collector.function_set))

    def collect_from_global_scope(self, global_scope):
        assert isinstance(global_scope, CppGlobalScope)
        data = [FunctionCollector(f) for f in collect_functions(global_scope)]
        for collector in data:
            self.collect(collector)
        return data


    _MSG_TYPE_HEADERS = [
        "Message Type", "Median Subscriber Queue",
        "Min. Subscriber Queue", "Max. Subscriber Queue",
        "Median Publisher Queue",
        "Min. Publisher Queue", "Max. Publisher Queue"
    ]

    def csv_message_types(self):
        rows = [GlobalCollector._MSG_TYPE_HEADERS]
        for msg, qs in self.sub.msg_to_queue.iteritems():
            if not msg in self.pub.msg_to_queue:
                rows.append([msg, median(qs), min(qs), max(qs), None, None, None])
        for msg, qs in self.pub.msg_to_queue.iteritems():
            inq = self.sub.msg_to_queue.get(msg)
            if inq:
                rows.append([msg, median(inq), min(inq), max(inq),
                             median(qs), min(qs), max(qs)])
            else:
                rows.append([msg, None, None, None, median(qs), min(qs), max(qs)])
        return rows

    _OTHER_HEADERS = [
        "Spin Rates", "Hard-coded Spin Rates", "Median Spin Rate",
        "Min. Spin Rate", "Max. Spin Rate", "Median Function Calls",
        "Min. Function Calls", "Max. Function Calls", "Median Unique Calls",
        "Min. Unique Calls", "Max. Unique Calls"
    ]

    def csv_other(self):
        rows = [GlobalCollector._OTHER_HEADERS]
        data = [self.total_rates]
        if self.hardcoded_rates:
            data.extend([len(self.hardcoded_rates), median(self.hardcoded_rates),
                         min(self.hardcoded_rates), max(self.hardcoded_rates)])
        else:
            data.extend([0, None, None, None])
        if self.function_calls:
            data.extend([median(self.function_calls), min(self.function_calls),
                         max(self.function_calls), median(self.distinct_calls),
                         min(self.distinct_calls), max(self.distinct_calls)])
        else:
            data.extend([None, None, None, None, None, None])
        rows.append(data)
        return rows



###############################################################################
# Global Collector
###############################################################################



class GlobalCollector(object):
    def __init__(self):
        self.pub                = PublisherStatistics()
        self.sub                = SubscriberStatistics()
        self.rpc                = RpcStatistics()
        self.param              = ParamStatistics()
        self.total_rates        = 0
        self.hardcoded_rates    = []
        self.function_calls     = []
        self.distinct_calls     = []
        self.pub_data           = []
        self.sub_data           = []
        self.rpc_data           = []
        self.other_data         = []

    @property
    def total_pub_sub(self):
        return self.sub.total_subscribers + self.pub.total_publishers

    @property
    def total_rpc(self):
        return self.rpc.total_rpc

    @property
    def global_topics(self):
        return self.pub.global_topics + self.sub.global_topics \
               + self.rpc.global_topics

    @property
    def function_callbacks(self):
        return self.sub.function_callbacks + self.rpc.function_callbacks

    @property
    def method_callbacks(self):
        return self.sub.method_callbacks + self.rpc.method_callbacks

    @property
    def boost_callbacks(self):
        return self.sub.boost_callbacks + self.rpc.boost_callbacks

    def collect(self, function_collector, store = False):
        for datum in function_collector.subscribe:
            self.sub.collect_subscribe(datum)
            if store: self.sub_data.append(datum)
        for datum in function_collector.advertise:
            self.pub.collect_advertise(datum)
            if store: self.pub_data.append(datum)
        for datum in function_collector.publish:
            self.pub.collect_publish(datum)
            if store: self.pub_data.append(datum)
        for datum in function_collector.advertise_service:
            self.rpc.collect_server(datum)
            if store: self.rpc_data.append(datum)
        for datum in function_collector.service_client:
            self.rpc.collect_client(datum)
            if store: self.rpc_data.append(datum)
        for datum in function_collector.ros_parameters:
            self.param.collect_param(datum)
            if store: self.other_data.append(datum)
        for datum in function_collector.spin_rate:
            self.total_rates += 1
            if isinstance(datum.rate, (int, long)):
                self.hardcoded_rates.append(datum.rate)
            if store: self.other_data.append(datum)
        self.function_calls.append(function_collector.function_calls)
        self.distinct_calls.append(len(function_collector.function_set))

    def collect_from_global_scope(self, global_scope, store = False):
        assert isinstance(global_scope, CppGlobalScope)
        data = [FunctionCollector(f) for f in collect_functions(global_scope)]
        for collector in data:
            self.collect(collector, store = store)
        return data


    _MSG_TYPE_HEADERS = [
        "Message Type", "Median Subscriber Queue",
        "Min. Subscriber Queue", "Max. Subscriber Queue",
        "Median Publisher Queue",
        "Min. Publisher Queue", "Max. Publisher Queue"
    ]

    def csv_message_types(self):
        rows = [GlobalCollector._MSG_TYPE_HEADERS]
        for msg, qs in self.sub.msg_to_queue.iteritems():
            if not msg in self.pub.msg_to_queue:
                rows.append([msg, median(qs), min(qs), max(qs), None, None, None])
        for msg, qs in self.pub.msg_to_queue.iteritems():
            inq = self.sub.msg_to_queue.get(msg)
            if inq:
                rows.append([msg, median(inq), min(inq), max(inq),
                             median(qs), min(qs), max(qs)])
            else:
                rows.append([msg, None, None, None, median(qs), min(qs), max(qs)])
        return rows

    _OTHER_HEADERS = [
        "Spin Rates", "Hard-coded Spin Rates", "Median Spin Rate",
        "Min. Spin Rate", "Max. Spin Rate", "Median Function Calls",
        "Min. Function Calls", "Max. Function Calls", "Median Unique Calls",
        "Min. Unique Calls", "Max. Unique Calls"
    ]

    def csv_other(self):
        rows = [GlobalCollector._OTHER_HEADERS]
        data = [self.total_rates]
        if self.hardcoded_rates:
            data.extend([len(self.hardcoded_rates), median(self.hardcoded_rates),
                         min(self.hardcoded_rates), max(self.hardcoded_rates)])
        else:
            data.extend([0, None, None, None])
        if self.function_calls:
            data.extend([median(self.function_calls), min(self.function_calls),
                         max(self.function_calls), median(self.distinct_calls),
                         min(self.distinct_calls), max(self.distinct_calls)])
        else:
            data.extend([None, None, None, None, None, None])
        rows.append(data)
        return rows


    def str_pub_data(self):
        return "\n".join(map(repr, self.pub_data))

    def str_sub_data(self):
        return "\n".join(map(repr, self.sub_data))

    def str_rpc_data(self):
        return "\n".join(map(repr, self.rpc_data))

    def str_other_data(self):
        return "\n".join(map(repr, self.other_data))



###############################################################################
# File Collector
###############################################################################

class FileCollector(object):
    def __init__(self, function_collectors):
        self.subscribe          = {}
        self.subscribe_unknown  = []    # for unknown topic
        self.subscribers        = {}
        self.advertise          = {}
        self.advertise_unknown  = []    # for unknown topic
        self.publish            = {}
        self.publishers         = {}
        self.advertise_service  = {}
        self.advertise_service_unknown  = []    # for unknown service
        self.servers            = {}
        self.service_client     = {}
        self.service_client_unknown     = []    # for unknown service
        self.clients            = {}
        self.spin_rate          = {}
        self.publish_rate       = {}
        self._collect(function_collectors)

    @classmethod
    def from_global_scope(cls, global_scope):
        assert isinstance(global_scope, CppGlobalScope)
        data = [FunctionCollector(f) for f in collect_functions(global_scope)]
        return FileCollector(data)

    def num_subscribe(self):
        return len(self.subscribe) + len(self.subscribe_unknown)

    def num_advertise(self):
        return len(self.advertise) + len(self.advertise_unknown)

    def num_advertise_service(self):
        return len(self.advertise_service) + len(self.advertise_service_unknown)

    def num_service_client(self):
        return len(self.service_client) + len(self.service_client_unknown)

    def _collect(self, function_collectors):
        self._basic_occurrences(function_collectors)
        self._publish_occurrences(function_collectors)
        self._publish_rates(function_collectors)

    def _basic_occurrences(self, function_collectors):
        for c in function_collectors:
            for var in c.spin_rate:
                self.spin_rate[var.variable] = var
            for var in c.subscribe:
                if isinstance(var.topic, basestring):
                    self.subscribe[var.topic] = var
                else:
                    self.subscribe_unknown.append(var)
                if var.variable:
                    self.subscribers[var.variable] = var.topic
            for var in c.advertise:
                if isinstance(var.topic, basestring):
                    self.advertise[var.topic] = var
                else:
                    self.advertise_unknown.append(var)
                if var.variable:
                    self.publishers[var.variable] = var.topic
            for var in c.advertise_service:
                if isinstance(var.topic, basestring):
                    self.advertise_service[var.topic] = var
                else:
                    self.advertise_service_unknown.append(var)
                if var.variable:
                    self.servers[var.variable] = var.topic
            for var in c.service_client:
                if isinstance(var.topic, basestring):
                    self.service_client[var.topic] = var
                else:
                    self.service_client_unknown.append(var)
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
        self.ros_parameters     = []
        self.param_vars         = set()
        self.function_set       = set()
        self.function_calls     = 0
        for statement in function.body.body:
            self._collect(statement)

    def _collect(self, statement, nesting = 0, variable = None):
        if isinstance(statement, CppVariable):
            self._from_variable(statement, nesting)
        elif isinstance(statement, CppFunctionCall):
            self._from_call(statement, nesting, variable)
        elif isinstance(statement, CppControlFlow):
            # TODO reduce nesting on "else if"
            for branch in statement.branches:
                for stmt in branch[1].body:
                    self._collect(stmt, nesting = nesting + 1, variable = variable)
        elif isinstance(statement, CppBlock):
            for stmt in statement.body:
                self._collect(stmt, nesting, variable)
        elif isinstance(statement, CppStatement):
            if statement.value:
                self._collect(statement.value, nesting, variable)
        elif isinstance(statement, CppOperator):
            if statement.name == "=" or statement.name == "[op]":
                for arg in statement.arguments:
                    self._collect(arg, nesting, variable)

    def _from_variable(self, variable, nesting):
        global pubsub_var
        name = variable.name
        if variable.result == "ros::Rate":
            if isinstance(variable.value, CppFunctionCall):
                value = None
                if len(variable.value.arguments) > 0:
                    value = variable.value.arguments[0]
                o = SpinRateTuple(value, name, self.function, variable.line)
                self.spin_rate.append(o)
        elif variable.result == "ros::Publisher" \
                or variable.result == "ros::Subscriber" \
                or variable.result == "ros::ServiceServer" \
                or variable.result == "ros::ServiceClient":
            if variable.result == "ros::Publisher" or variable.result == "ros::Subscriber":
                pubsub_var += 1
            self._collect(variable.value, nesting, variable = name)
        elif isinstance(variable.value, CppFunctionCall) \
                and variable.value.name == "param":
            self._from_call(variable.value, nesting, variable = name)

    def _from_call(self, call, nesting, variable = None):
        global advertise_count
        global subscribe_count
        # TODO a function map seems better by now...
        name = call.name
        if name == "operator=":
            if len(call.arguments) >= 2 and isinstance(call.arguments[1], CppFunctionCall):
                self._from_call(call.arguments[1], nesting,
                                variable = call.arguments[0].name)
            elif len(call.arguments) < 2:
                print "[CLANG]", name, call.arguments
            return
        self.function_set.add(name)
        self.function_calls += 1
        if name == "advertise" and call.result == "ros::Publisher":
            advertise_count += 1
            self._advertise_call(call, nesting, variable = variable)
        elif name == "subscribe" and call.result == "ros::Subscriber":
            subscribe_count += 1
            self._subscribe_call(call, nesting, variable = variable)
        elif name == "publish" and call.method_of \
                and call.method_of.result == "ros::Publisher":
            o = PublishTuple(call.method_of.name, nesting,
                             self.function, call.line, call.scope)
            self.publish.append(o)
        elif name == "advertiseService" and call.result == "ros::ServiceServer":
            self._advertise_service_call(call, nesting, variable = variable)
        elif name == "serviceClient" and call.result == "ros::ServiceClient":
            self._service_client_call(call, nesting, variable = variable)
        elif name == "sleep" and call.method_of \
                and call.method_of.result == "ros::Rate":
            self.sleep.append(call)
        elif name == "param":
            self._param_call(call, nesting, variable = variable)
        elif name == "getParam" or name == "getParamCached":
            self._get_param_call(call, nesting, variable = variable)
        else:
            for arg in call.arguments:
                self._collect(arg)


    def _advertise_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        topic       = None
        queue_size  = None
        latch       = False
        var         = ()
        if nargs == 1:
            atype = ADV_TYPE_3
        else:
            assert nargs == 3 or nargs == 6
            atype = ADV_TYPE_1 if nargs == 3 else ADV_TYPE_2
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            latch       = call.arguments[-1]
            latch       = latch if isinstance(latch, bool) else None
            if isinstance(topic, CppExpression):
                var     = topic.variables
        self.advertise.append(AdvertiseTuple(topic, queue_size,
                call.template, nesting, variable, self.function, call.line,
                atype, latch, any(v in self.param_vars for v in var)))

    def _subscribe_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        topic       = None
        queue_size  = None
        callback    = None
        msg_type    = None
        hints       = False
        var         = ()
        if nargs == 1:
            # assert call.arguments[0].result == "ros::SubscribeOptions"
            stype = SUB_TYPE_4
        else:
            # nargs == 4 (function) or nargs == 5 (method/boost)
            assert nargs == 4 or nargs == 5
            topic       = call.arguments[0]
            queue_size  = call.arguments[1]
            cb          = call.arguments[2]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            if nargs == 4:
                stype = SUB_TYPE_2
            else:
                if isinstance(call.arguments[3], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    stype = SUB_TYPE_3
                else:
                    stype = SUB_TYPE_1
            hints = not isinstance(call.arguments[-1], CppDefaultArgument)
            if isinstance(topic, CppExpression):
                var = topic.variables
        self.subscribe.append(SubscribeTuple(topic, queue_size, callback,
                msg_type, nesting, variable, self.function,
                call.line, stype, hints, any(v in self.param_vars for v in var)))

    def _advertise_service_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        service     = None
        callback    = None
        msg_type    = None
        events      = False
        var         = ()
        if nargs == 1:
            atype = ADV_SRV_TYPE_4
        else:
            # nargs == 2 (function) or nargs == 3 (method/boost)
            assert nargs == 2 or nargs == 3
            service = call.arguments[0]
            cb = call.arguments[1]
            if isinstance(cb, CppOperator) and cb.is_unary:
                cb = cb.arguments[0]
            callback = cb.name
            msg_type = self._msg_type_from_callback(cb)
            events = "ros::ServiceEvent" in cb.result
            if nargs == 2:
                atype = ADV_SRV_TYPE_2
            else:
                if isinstance(call.arguments[2], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    atype = ADV_SRV_TYPE_3
                else:
                    atype = ADV_SRV_TYPE_1
            if isinstance(service, CppExpression):
                var = service.variables
        self.advertise_service.append(AdvertiseServiceTuple(service,
                callback, msg_type, nesting, variable, self.function, call.line,
                atype, events, any(v in self.param_vars for v in var)))

    def _service_client_call(self, call, nesting, variable = None):
        nargs = len(call.arguments)
        service = None
        ctype = SRV_CLI_TYPE_1
        var = ()
        if nargs == 1:
            if "ros::ServiceClientOptions" in call.arguments[0].result:
                ctype = SRV_CLI_TYPE_2
        else:
            service = call.arguments[0]
            if isinstance(service, CppExpression):
                var = service.variables
        self.service_client.append(ServiceClientTuple(service,
                call.template, nesting, variable, self.function,
                call.line, ctype, any(v in self.param_vars for v in var)))

    def _param_call(self, call, nesting, variable = None):
        n = len(call.arguments)
        if n == 2:      # T param(std::string &key, T &default_val)
            result = call.result
        elif n == 3:    # void param(std::string &key, T &val, T &default_val)
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
        self.ros_parameters.append(ParamTuple(call.arguments[0],
                variable, True, result, self.function, call.line))
        if variable:
            self.param_vars.add(variable)

    def _get_param_call(self, call, nesting, variable = None):
        if len(call.arguments) >= 2:
            result = None
            if call.arguments[1]:
                variable = call.arguments[1].name
                result = call.arguments[1].result
            self.ros_parameters.append(ParamTuple(call.arguments[0],
                    variable, False, result, self.function, call.line))
            if variable:
                self.param_vars.add(variable)


    def _msg_type_from_callback(self, callback):
        type_string = callback.result
        type_string = type_string.split(None, 1)[1]
        if not (type_string[0] == "(" and type_string[-1] == ")"):
            print "[CLANG] Unknown message type:", type_string
            return type_string
        type_string = type_string[1:-1]
        is_const = type_string.startswith("const ")
        if is_const:
            type_string = type_string[6:]
        is_ref = type_string.endswith(" &")
        if is_ref:
            type_string = type_string[:-2]
        is_ptr = type_string.endswith("::ConstPtr")
        if is_ptr:
            type_string = type_string[:-10]
        else:
            is_ptr = type_string.endswith("ConstPtr")
            if is_ptr:
                type_string = type_string[:-8]
        return type_string



###############################################################################
# Helper Functions
###############################################################################

def control_nesting(entity):
    nesting = 0
    s = entity.scope
    while s:
        if isinstance(s, CppControlFlow):
            nesting += 1
        elif isinstance(s, CppFunction) or isinstance(s, CppClass) \
                or isinstance(s, CppNamespace) or isinstance(s, CppGlobalScope):
            break
        s = s.scope
    return nesting


def collect_functions(scope = None):
    global collected_functions
    global_scope = scope or CppGlobalScope.INSTANCE
    all_functions = list(global_scope.functions)
    for c in global_scope.classes:
        all_functions.extend(c.functions)
    s = list(global_scope.namespaces)
    while s:
        ns = s
        s = []
        for n in ns:
            all_functions.extend(n.functions)
            for c in n.classes:
                all_functions.extend(c.functions)
            s.extend(n.namespaces)
    collected_functions += len(all_functions)
    return all_functions


def median(values):
    values = sorted(values)
    n = len(values)
    i = (n - 1) // 2
    if (n % 2):
        return values[i]
    else:
        return (values[i] + values[i+1]) / 2.0

def mean(values):
    return float(sum(values)) / max(len(values), 1)
