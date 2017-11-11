
################################################################################
#   Standard Libraries
################################################################################

from collections import defaultdict

################################################################################
#   Third-Party Libraries
################################################################################

from bonsai.model import (
    CodeExpression, CodeReference, CodeOperator, CodeFunctionCall
)
from bonsai.analysis import CodeQuery, resolve_expression, get_control_depth


################################################################################
#   Basic Statistics Classes
################################################################################

class StatValues(object):
    def __init__(self, name = ""):
        self.name       = name
        self.total      = 0
        self.literal    = 0     # value is a literal
        self.reference  = 0     # value comes from a reference (e.g. variable)
        self.call       = 0     # value is the result of a function call
        self.operator   = 0     # value is the result of an operator
        self.param      = 0     # value comes from a ROS parameter
        self.values     = []    # observed and resolved values

    @property
    def counting(self):
        return self.literal + self.reference + self.call + self.operator

    PARAM_FUNCTIONS = ("param", "getParam", "get_param")

    def add(self, codeobj):
        assert isinstance(codeobj, CodeExpression.TYPES)
        self.total += 1
        if isinstance(codeobj, CodeExpression.LITERALS):
            self.literal += 1
        elif isinstance(codeobj, CodeReference):
            self.reference += 1
        elif isinstance(codeobj, CodeOperator):
            self.operator += 1
        elif isinstance(codeobj, CodeFunctionCall):
            self.call += 1
        value = resolve_expression(codeobj)
        if isinstance(value, CodeExpression.LITERALS):
            self.values.append(value)
        elif (isinstance(value, CodeFunctionCall)
                and value.name in self.PARAM_FUNCTIONS):
            self.param += 1
        return value



class SubscriberStatistics(object):
    def __init__(self):
        self.topics = StatValues("Topics")
        self.queues = StatValues("Queues")
        self.msg_to_queue       = {}
        self.nesting            = []
        self.global_topics      = 0
        self.infinite_queues    = 0
        self.transport_hints    = 0
        self.function_callbacks = 0
        self.method_callbacks   = 0
        self.boost_callbacks    = 0
        self.custom_msgs        = 0

    @property
    def total_subscribers(self):
        return self.topics.total

    @property
    def repeating_topics(self):
        return len(self.topics.values) - len(set(self.topics.values))

    def collect_subscribe(self, call):
        if len(call.arguments) > 1:
            topic = self.topics.add(call.arguments[0])
            if isinstance(topic, basestring) and topic.startswith("/"):
                self.global_topics += 1
            queue = self.queues.add(call.arguments[1])
            if queue == 0:
                self.infinite_queues += 1
            callback = call.arguments[2]
            if isinstance(callback, CodeOperator) and callback.is_unary:
                callback = callback.arguments[0]
            msg_type = self._msg_type_from_callback(callback)
            if nargs == 4:
                stype = SUB_TYPE_2
            else:
                if isinstance(call.arguments[3], CppDefaultArgument) \
                        or cb.result.startswith("boost::function"):
                    stype = SUB_TYPE_3
                else:
                    stype = SUB_TYPE_1
            hints = not isinstance(call.arguments[-1], CppDefaultArgument)


class PublisherStatistics(object):
    pass


class ServiceStatistics(object):
    pass


class ParameterStatistics(object):
    pass


class SpinnerStatistics(object):
    pass




class RosCppStatistics(object):
    def __init__(self):
        self.pub    = PublisherStatistics()
        self.sub    = SubscriberStatistics()
        self.srv    = ServiceStatistics()
        self.param  = ParameterStatistics()
        self.spin   = SpinnerStatistics()
        self.function_calls     = []
        self.distinct_calls     = []
        self.pub_data           = []
        self.sub_data           = []
        self.rpc_data           = []
        self.other_data         = []


################################################################################
#   Statistics Collector - Entry Point
################################################################################

class StatisticsCollector(object):
    def __init__(self):
        self.all = RosCppStatistics()
        self.pkg = defaultdict(RosCppStatistics)

    def collect(self, global_scope):
        """Collects data from the given bonsai Global Scope object."""
        q = CodeQuery(global_scope).all_calls
        for call in q.where_name("advertise").where_result("ros::Publisher"):
            self.all.pub.collect_advertise(call)
        for call in q.where_name("subscribe").where_result("ros::Subscriber"):
            self.all.sub.collect_subscribe(call)
        for call in (q.where_name("advertiseService")
                      .where_result("ros::ServiceServer")):
            self.all.srv.collect_advertise(call)
        for call in (q.where_name("serviceClient")
                      .where_result("ros::ServiceClient")):
            self.all.srv.collect_client(call)
