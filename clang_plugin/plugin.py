
# TODO
# resolve references
# references across files in the same package

# https://github.com/llvm-mirror/clang/blob/master/bindings/python/clang/cindex.py

# might be interesting
# https://github.com/pybee-attic/sealang

import os

import clang.cindex as clang

from ctypes import ArgumentError
from collections import namedtuple


###############################################################################
# HAROS Plugin Interface
###############################################################################

def pre_analysis():
    clang.Config.set_library_path("/usr/lib/llvm-3.8/lib")
    index = clang.Index.create()
    return (index, {})


def file_analysis(iface, scope):
    file_path   = scope.get_path()
    index       = iface.state[0]
    includes    = iface.state[1]
    _find_includes(iface, scope.package, includes)
    assert scope.package.id in includes
    args = ["-I" + path for path in includes[scope.package.id]]
    args.append("-x")
    args.append("c++")
    # args.append("-std=c++11")
    unit = index.parse(file_path, args)
    # check for problems
    if unit.diagnostics:
        for d in unit.diagnostics:
            if d.severity >= clang.Diagnostic.Error:
                print d
    # traverse nodes
    data = _ast_analysis(unit, file_path)
    _report_results(iface, data)


###############################################################################
# CMake Analysis
###############################################################################

_DEFAULT_INCLUDES = ["/usr/lib/llvm-3.8/lib/clang/3.8.0/include",
                     "/usr/include/eigen3", "/usr/include/ImageMagick"]

def _find_includes(iface, package, includes):
    if package.id in includes:
        return
    includes[package.id] = _read_package_includes(package)
    for id in package.dependencies:
        dep = iface.find_package(id)
        if dep:
            _find_includes(iface, dep, includes)
            includes[package.id].extend(includes[dep.id])
    includes[package.id] = list(set(includes[package.id]))


def _read_package_includes(package):
    cmake_path = os.path.join(package.path, "CMakeLists.txt")
    dirs = _read_cmake(cmake_path)
    return map(lambda t: _replace_cmake_tokens(t, package.path), dirs)

def _read_cmake(cmake_path):
    all_dirs = list(_DEFAULT_INCLUDES)
    if os.path.isfile(cmake_path):
        with open(cmake_path, "r") as f:
            idirs = None    # include_directories
            cdirs = None    # catkin_package
            for line in f:
                line = line.strip()
                if idirs is None and line.startswith("include_directories("):
                    idirs = []
                    line = line[20:]
                if not idirs is None:
                    tokens = line.split(")")
                    idirs.extend(tokens[0].split())
                    if len(tokens) > 1:
                        all_dirs.extend(idirs)
                        idirs = None
                        continue
                if cdirs is None and line.startswith("catkin_package("):
                    cdirs = []
                    line = line[15:]
                if not cdirs is None:
                    tokens = line.split(")")
                    cdirs.extend(tokens[0].split())
                    if len(tokens) > 1:
                        flag = False
                        stop_at = ("LIBRARIES", "CATKIN_DEPENDS", "DEPENDS", "CFG_EXTRAS")
                        for d in cdirs:
                            if d in stop_at:
                                flag = False
                            elif flag:
                                all_dirs.append(d)
                            elif d == "INCLUDE_DIRS":
                                flag = True
                        cdirs = None
                        continue
    return all_dirs

_REPLACEMENTS = {
    "catkin":   "/home/andre/catkin_ws/devel/include",
}

def _replace_cmake_tokens(token, pkg_path = ""):
    token = token.replace("${PROJECT_SOURCE_DIR}/", "")
    if token[0] == "$" :
        if token.endswith("_INCLUDE_DIRS}"):
            component = token[2:-14]
            if component in _REPLACEMENTS:
                return _REPLACEMENTS[component]
            else:
                return _REPLACEMENTS["catkin"]
        return _REPLACEMENTS["catkin"]
    return os.path.join(pkg_path, token)



###############################################################################
# Analyser
###############################################################################

def _ast_analysis(unit, file_path):
    cursor = unit.cursor
    # we cannot reuse the same instance of global scope, or else we get stuff
    # from other files mixed in
    global_scope = CppGlobalScope()
    for node in cursor.get_children():
        if node.location.file and node.location.file.name == file_path:
            global_scope.add_from_cursor(node)
    data = [FunctionCollector(f) for f in collect_functions(global_scope)]
    return FileCollector(data)


def _report_results(iface, data):
    iface.report_metric("subscribers", len(data.subscribe))
    iface.report_metric("publishers", len(data.advertise))
    iface.report_metric("active_publishers", len(data.publish))
    iface.report_metric("service_servers", len(data.advertise_service))
    iface.report_metric("service_clients", len(data.service_client))
    for topic, datum in data.subscribe.iteritems():
        iface.report_metric("subscribe_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
    for topic, datum in data.advertise.iteritems():
        iface.report_metric("advertise_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
        if isinstance(datum.queue_size, (int, long)):
            iface.report_metric("queue_size", datum.queue_size,
                                line = datum.line, function = datum.function)
    for var, datum in data.publish.iteritems():
        iface.report_metric("publish_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    for topic, datum in data.advertise_service.iteritems():
        iface.report_metric("advertise_service_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    for topic, datum in data.service_client.iteritems():
        iface.report_metric("service_client_nesting", datum.nesting,
                            line = datum.line, function = datum.function)
    for var, datum in data.spin_rate.iteritems():
        if isinstance(datum.rate, (int, long)):
            iface.report_metric("spin_rate", datum.rate,
                                function = datum.function)


SubscribeTuple = namedtuple("SubscribeTuple",
                            ["topic", "queue_size", "callback",
                             "nesting", "variable", "function", "line"])

AdvertiseTuple = namedtuple("AdvertiseTuple",
                            ["topic", "queue_size", "message_type",
                             "nesting", "variable", "function", "line"])

PublishTuple = namedtuple("PublishTuple",
                          ["variable", "nesting", "function", "line"])

AdvertiseServiceTuple = namedtuple("AdvertiseServiceTuple",
                                   ["topic", "callback", "nesting",
                                    "variable", "function", "line"])

ServiceClientTuple = namedtuple("ServiceClientTuple",
                                ["topic", "message_type", "nesting",
                                 "variable", "function", "line"])

SpinRateTuple = namedtuple("SpinRateTuple",
                           ["rate", "variable", "function", "line"])


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
        self._collect(function_collectors)

    def _collect(self, function_collectors):
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
        for c in function_collectors:
            for call in c.publish:
                if call.variable in self.publishers:
                    self.publish[call.variable] = call


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
        for statement in function.body.body:
            self._collect(statement)

    def _collect(self, statement, nesting = 0):
        if isinstance(statement, CppVariable):
            self._from_variable(statement, nesting)
        elif isinstance(statement, CppFunctionCall):
            self._from_call(statement, nesting)
        elif isinstance(statement, CppControlFlow):
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
            if isinstance(call.arguments[1], CppFunctionCall):
                self._from_call(call.arguments[1], nesting,
                                variable = call.arguments[0].name)
            return
        if name == "advertise" and call.result == "ros::Publisher":
            o = AdvertiseTuple(call.arguments[0], call.arguments[1],
                               call.template, nesting, variable,
                               self.function, call.line)
            self.advertise.append(o)
        elif name == "subscribe" and call.result == "ros::Subscriber":
            callback = call.arguments[2]
            if isinstance(callback, LazyCppEntity):
                callback = callback.evaluate()
            if isinstance(callback, CppOperator) and callback.is_unary:
                callback = callback.arguments[0]
            o = SubscribeTuple(call.arguments[0], call.arguments[1],
                               callback.name, nesting, variable,
                               self.function, call.line)
            self.subscribe.append(o)
        elif name == "publish" and call.method_of \
                and call.method_of.result == "ros::Publisher":
            o = PublishTuple(call.method_of.name, nesting,
                             self.function, call.line)
            self.publish.append(o)
        elif name == "advertiseService" and call.result == "ros::ServiceServer":
            callback = call.arguments[1]
            if isinstance(callback, LazyCppEntity):
                callback = callback.evaluate()
            if isinstance(callback, CppOperator) and callback.is_unary:
                callback = callback.arguments[0]
            o = AdvertiseServiceTuple(call.arguments[0], callback.name, nesting,
                                      variable, self.function, call.line)
            self.advertise_service.append(o)
        elif name == "serviceClient" and call.result == "ros::ServiceClient":
            o = ServiceClientTuple(call.arguments[0], call.template, nesting,
                                   variable, self.function, call.line)
            self.service_client.append(o)

    def has_results(self):
        return self.subscribe or self.advertise or self.publish \
                or self.advertise_service or self.service_client \
                or self.spin_rate

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
# Base Class for Language Constructs
###############################################################################

class CppBasicEntity(object):
    def __init__(self, scope, filename = None, line = 0, column = 0):
        self.scope  = scope
        self.file   = filename
        self.line   = line
        self.column = column

    @classmethod
    def from_cursor(cls, cursor, scope = None):
        node = cls(scope or CppGlobalScope.INSTANCE)
        node._read_cursor(cursor)
        return node

    def _read_cursor(self, cursor):
        try:
            self.file   = cursor.location.file.name
            self.line   = cursor.location.line
            self.column = cursor.location.column
        except ArgumentError as e:
            self.file   = None
            self.line   = None
            self.column = None

    def _traverse(self, cursor):
        for child in cursor.get_children():
            self._traverse(child)

    def pretty_str(self, indent = 0):
        return (" " * indent) + self.__str__()

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "[unknown]"


class LazyCppEntity(object):
    def __init__(self, scope, cursor):
        self.scope  = scope
        self.cursor = cursor

    def evaluate(self):
        return _parse(self.cursor, scope = self.scope, lazy = True)

    def pretty_str(self, indent = 0):
        return (" " * indent) + self.__str__()

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "[LazyCppEntity]"



###############################################################################
# Analysis Objects
###############################################################################

class CppExpression(CppBasicEntity):
    def __init__(self, scope, name = "", result = "[type]", cursor = None):
        CppBasicEntity.__init__(self, scope)
        if cursor is None:
            self.name   = name
            self.result = result
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        CppBasicEntity._read_cursor(self, cursor)
        self.name   = cursor.spelling
        self.result = cursor.type.spelling or "[type]"

    def pretty_str(self, indent = 0):
        return (" " * indent) + self.name

    def __repr__(self):
        return "[{}] {}".format(self.result, self.name)


class CppReference(CppExpression):
    def __init__(self, scope, name = "", result = "[type]", cursor = None):
        self.field_of = None
        CppExpression.__init__(self, scope, name, result, cursor)
        # CppExpression already calls self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.DECL_REF_EXPR \
                or cursor.kind == clang.CursorKind.MEMBER_REF \
                or cursor.kind == clang.CursorKind.MEMBER_REF_EXPR
        CppExpression._read_cursor(self, cursor)
        if cursor.kind == clang.CursorKind.MEMBER_REF_EXPR:
            child = next(cursor.get_children(), None)
            if child:
                self.field_of = _parse(child, scope = self.scope)

    def pretty_str(self, indent = 0):
        spaces = (" " * indent)
        if self.field_of:
            return spaces + self.field_of.pretty_str(indent) + "." + self.name
        return spaces + self.name

    def __str__(self):
        return "#" + self.name

    def __repr__(self):
        if self.field_of:
            return "[{}] ({}).{}".format(self.result, self.field_of, self.name)
        return "[{}] #{}".format(self.result, self.name)


class CppOperator(CppExpression):
    _unary_tokens = ("+", "-", "++", "--", "*", "&", "!", "~")
    _binary_tokens = ("+", "-", "*", "/", "%", "&", "|", "^", "<<", ">>",
                      "<", ">", "<=", ">=", "==", "!=", "&&", "||", "=",
                      "+=", "-=", "*=", "/=", "%=", "<<=", ">>=", "&=",
                      "|=", "^=", ",")

    def __init__(self, scope, name = "", result = "[type]",
                 args = (), cursor = None):
        CppExpression.__init__(self, scope, name, result, cursor)
        # CppExpression already calls self._read_cursor(cursor)
        if cursor is None:
            self.arguments  = args
            self.is_unary   = False
            self.is_binary  = False

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.UNARY_OPERATOR \
                or cursor.kind == clang.CursorKind.BINARY_OPERATOR \
                or cursor.kind == clang.CursorKind.COMPOUND_ASSIGNMENT_OPERATOR
        CppExpression._read_cursor(self, cursor)
        if cursor.kind == clang.CursorKind.UNARY_OPERATOR:
            self.name       = CppOperator._parse_unary(cursor)
            arg             = next(cursor.get_children())
            self.arguments  = (_parse(arg, scope = self.scope, lazy = True),)
            self.is_unary   = True
            self.is_binary  = False
        else:
            self.name       = CppOperator._parse_binary(cursor)
            args            = list(cursor.get_children())
            assert len(args) >= 2
            self.arguments  = (_parse(args[0], scope = self.scope, lazy = True),
                               _parse(args[1], scope = self.scope, lazy = True))
            self.is_unary   = False
            self.is_binary  = True

    @staticmethod
    def _parse_unary(cursor):
        tokens = list(cursor.get_tokens())
        if tokens:
            token = tokens[0].spelling
            # The only alpha tokens are "new" and "delete",
            # but they have their own CursorKind
            if token[0].isalpha():
                # The last token seems to be what ends the expression, e.g. ';'
                token = tokens[-2].spelling
                assert not token[0].isalpha()
                if token == "++" or token == "--":
                    return "_" + token
            elif token in CppOperator._unary_tokens:
                return token
        return "[op]"

    @staticmethod
    def _parse_binary(cursor):
        # There are no alpha operators
        # I think "->" and "->*" might have their own CursorKind
        # All operators seem to be infix; get the last token of the first child
        tokens = list(next(cursor.get_children()).get_tokens())
        if tokens:
            token = tokens[-1].spelling
            if token in CppOperator._binary_tokens:
                return token
        return "[op]"

    def pretty_str(self, indent = 0):
        indent = (" " * indent)
        if self.is_unary:
            if self.name.startswith("_"):
                return "{}{}{}".format(indent, pretty_str(self.arguments[0]),
                                       self.name[1:])
            return "{}{}{}".format(indent, self.name,
                                   pretty_str(self.arguments[0]))
        else:
            return "{}{} {} {}".format(indent, pretty_str(self.arguments[0]),
                                       self.name,
                                       pretty_str(self.arguments[1]))

    def __repr__(self):
        if self.is_unary:
            return "[{}] {}({})".format(self.result, self.name,
                                        self.arguments[0])
        else:
            return "[{}] ({}){}({})".format(self.result, self.arguments[0],
                                            self.name, self.arguments[1])


class CppFunctionCall(CppExpression):
    def __init__(self, scope, name = "<anonymous>", result = "[type]",
                 args = None, cursor = None):
        CppExpression.__init__(self, scope, name, result, cursor)
        # CppExpression already calls self._read_cursor(cursor)
        self.template = None
        if cursor is None:
            self.arguments      = args if not args is None else []
            self.is_constructor = self.result.split("::")[-1] == self.name
            self.method_of      = None

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.CALL_EXPR
        CppExpression._read_cursor(self, cursor)
        self._traverse(cursor)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CALL_EXPR and cursor.spelling:
            self.name = cursor.spelling
            self.is_constructor = self.result.split("::")[-1] == self.name
            args = list(cursor.get_arguments())
            children = [c for c in cursor.get_children() if not c in args]
            self._parse_non_arguments(children)
            if not args and self.is_constructor:
                args = children
            self.arguments = [_parse(arg, scope = self.scope, arguments = True,
                                     lazy = True) for arg in args]
        else:
            child = next(cursor.get_children(), None)
            if child:
                self._traverse(child)

    def _parse_non_arguments(self, nodes):
        children = None
        for cursor in nodes:
            if cursor.kind == clang.CursorKind.MEMBER_REF_EXPR \
                    and cursor.spelling == self.name:
                children = list(cursor.get_children())
                break
        if children:
            self.method_of = _parse(children[0], scope = self.scope)
            template = []
            for cursor in children[1:]:
                if cursor.kind == clang.CursorKind.NAMESPACE_REF \
                        or cursor.kind == clang.CursorKind.TYPE_REF:
                    template.append(cursor.spelling)
            if template:
                self.template = "::".join(template)

    def pretty_str(self, indent = 0):
        indent = " " * indent
        temp = "<" + self.template + ">" if self.template else ""
        if self.name.startswith("operator"):
            operator = self.name[8:]
            if operator in CppOperator._binary_tokens:
                return "{}{} {} {}".format(indent, pretty_str(self.arguments[0]),
                                           operator,
                                           pretty_str(self.arguments[1]))
        args = ", ".join([pretty_str(arg) for arg in self.arguments])
        if self.method_of:
            return "{}{}.{}{}({})".format(indent, self.method_of.pretty_str(),
                                          self.name, temp, args)
        return "{}{}{}({})".format(indent, self.name, temp, args)

    def __repr__(self):
        temp = "<" + self.template + ">" if self.template else ""
        args = ", ".join([str(arg) for arg in self.arguments])
        if self.is_constructor:
            return "[{}] new {}({})".format(self.result, self.name, args)
        if self.method_of:
            return "[{}] {}.{}{}({})".format(self.result, self.method_of.name,
                                           self.name, temp, args)
        return "[{}] {}{}({})".format(self.result, self.name, temp, args)


class CppControlFlow(CppBasicEntity):
    def __init__(self, scope, name = "", branches = None, cursor = None):
        CppBasicEntity.__init__(self, scope)
        self.variables = []
        if cursor is None:
            self.name = name
            self.branches = branches if not branches is None else []
            # branches :: [(CppExpression, CppBlock)]
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.WHILE_STMT \
                or cursor.kind == clang.CursorKind.FOR_STMT \
                or cursor.kind == clang.CursorKind.DO_STMT \
                or cursor.kind == clang.CursorKind.IF_STMT \
                or cursor.kind == clang.CursorKind.SWITCH_STMT
        CppBasicEntity._read_cursor(self, cursor)
        if cursor.kind == clang.CursorKind.WHILE_STMT:
            self.name = "while"
            self.branches = self._parse_while(cursor)
        elif cursor.kind == clang.CursorKind.FOR_STMT:
            self.name = "for"
            self.branches = self._parse_for(cursor)
        elif cursor.kind == clang.CursorKind.DO_STMT:
            self.name = "do"
            self.branches = self._parse_do(cursor)
        elif cursor.kind == clang.CursorKind.IF_STMT:
            self.name = "if"
            self.branches = self._parse_if(cursor)
        elif cursor.kind == clang.CursorKind.SWITCH_STMT:
            self.name = "switch"
            self.branches = self._parse_switch(cursor)

    def _parse_if(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 2
        condition = _parse(children[0], scope = self.scope, lazy = True)
        body = CppBlock.from_cursor(children[1], scope = self.scope)
        branches = [(condition, body)]
        if len(children) >= 3:
            # this is the "else" branch
            # condition = CppOperator(self.scope, "!", "bool", (condition,))
            condition = True
            body = CppBlock.from_cursor(children[2], scope = self.scope)
            branches.append((condition, body))
        return branches

    def _parse_switch(self, cursor):
        # switch is hard to parse, this is neither correct nor pretty
        children = list(cursor.get_children())
        assert len(children) >= 2
        var = _parse(children[0], scope = self.scope, lazy = True)
        assert children[1].kind == clang.CursorKind.COMPOUND_STMT
        branches = []
        condition = True
        body = None
        for node in children[1].get_children():
            if node.kind == clang.CursorKind.CASE_STMT:
                if body:
                    branches.append((condition, body))
                statements = list(node.get_children())
                body = CppBlock(self)
                value = _parse(statements[0], scope = self.scope, lazy = True)
                condition = CppOperator(self.scope, "==", "bool", (var, value))
                for statement in statements[1:]:
                    body.append_statement(_parse(statement, scope = body))
            elif node.kind == clang.CursorKind.DEFAULT_STMT:
                if body:
                    branches.append((condition, body))
                body = CppBlock(self)
                condition = True
                for statement in node.get_children():
                    body.append_statement(_parse(statement, scope = body))
            elif node.kind == clang.CursorKind.BREAK_STMT:
                if body:
                    branches.append((condition, body))
                body = None
                condition = True
            elif body:
                body.append_statement(_parse(node, scope = body))
        return branches

    def _parse_for(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 1
        # body always comes last
        body = CppBlock.from_cursor(children[-1], scope = self)
        if len(children) == 1:
            # just body
            condition = True
        elif len(children) == 2:
            # condition + body
            condition = _parse(children[0], scope = self.scope, lazy = True)
        elif len(children) >= 4:
            # var + condition + increment + body
            condition = _parse(children[1], scope = self, lazy = True)
            body.append_statement(_parse(children[2], scope = body))
            self.variables.append(_parse(children[0], scope = self))
        elif children[0].kind == clang.CursorKind.DECL_STMT:
            # var + condition + body
            condition = _parse(children[1], scope = self, lazy = True)
            self.variables.append(_parse(children[0], scope = self))
        else:
            # condition + increment + body
            condition = _parse(children[0], scope = self.scope, lazy = True)
            body.append_statement(_parse(children[1], scope = body))
        return [(condition, body)]

    def _parse_while(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 2
        condition = _parse(children[0], scope = self.scope, lazy = True)
        body = CppBlock.from_cursor(children[1], scope = self.scope)
        return [(condition, body)]

    def _parse_do(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 2
        condition = _parse(children[1], scope = self.scope, lazy = True)
        body = CppBlock.from_cursor(children[0], scope = self.scope)
        return [(condition, body)]

    def get_variables(self):
        variables = []
        for branch in self.branches:
            for statement in branch[1].body:
                if isinstance(statement, CppVariable):
                    variables.append(statement)
                elif isinstance(statement, CppControlFlow):
                    variables.extend(statement.get_variables())
        return variables

    def pretty_str(self, indent = 0):
        spaces = " " * indent
        if self.name == "if":
            condition = pretty_str(self.branches[0][0])
            pretty = spaces + "if (" + condition + "):\n"
            pretty += self.branches[0][1].pretty_str(indent = indent + 2)
            if len(self.branches) == 2:
                pretty += "\n" + spaces + "else:\n"
                pretty += self.branches[1][1].pretty_str(indent = indent + 2)
        elif self.name == "while":
            condition = pretty_str(self.branches[0][0])
            pretty = spaces + "while (" + condition + "):\n"
            pretty += self.branches[0][1].pretty_str(indent = indent + 2)
        elif self.name == "for":
            condition = pretty_str(self.branches[0][0])
            variables = ",".join([v.pretty_str() for v in self.variables])
            pretty = spaces + "for (" + variables + "; " + condition + "):\n"
            pretty += self.branches[0][1].pretty_str(indent = indent + 2)
        elif self.name == "do":
            condition = pretty_str(self.branches[0][0])
            pretty = spaces + "do:\n"
            pretty += self.branches[0][1].pretty_str(indent = indent + 2)
            pretty = spaces + "while (" + condition + ")"
        elif self.name == "switch":
            pretty = spaces + "switch:\n"
            for branch in self.branches:
                condition = pretty_str(branch[0])
                pretty += spaces + "  case (" + condition + "):\n"
                pretty += branch[1].pretty_str(indent = indent + 4)
        return pretty

    def __repr__(self):
        return "{} {}".format(self.name, self.branches)



###############################################################################
# Language Model
###############################################################################


# To use with "return", "break", "continue"...
class CppStatement(CppBasicEntity):
    def __init__(self, scope, name = "[statement]", cursor = None):
        CppBasicEntity.__init__(self, scope)
        self.name = name
        self.value = None
        if cursor:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.RETURN_STMT \
                or cursor.kind == clang.CursorKind.BREAK_STMT \
                or cursor.kind == clang.CursorKind.CONTINUE_STMT
        CppBasicEntity._read_cursor(self, cursor)
        if cursor.kind == clang.CursorKind.RETURN_STMT:
            self.name   = "return"
            expression  = next(cursor.get_children(), None)
            self.value  = _parse(expression, scope = self.scope, lazy = True) \
                          if expression else None
        elif cursor.kind == clang.CursorKind.BREAK_STMT:
            self.name   = "break"
        elif cursor.kind == clang.CursorKind.CONTINUE_STMT:
            self.name   = "continue"

    def append_statement(self, statement):
        if statement:
            self.body.append(statement)

    def pretty_str(self, indent = 0):
        indent = " " * indent
        if not self.value is None:
            return indent + self.name + " " + pretty_str(self.value)
        return indent + self.name

    def __repr__(self):
        if not self.value is None:
            return self.name + " " + str(self.value)
        return self.name


class CppBlock(CppBasicEntity):
    def __init__(self, scope, body = None, cursor = None):
        CppBasicEntity.__init__(self, scope)
        if cursor is None:
            self.body = body if not body is None else []
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        CppBasicEntity._read_cursor(self, cursor)
        self.body = _parse_body(cursor, scope = self)

    def append_statement(self, statement):
        if statement:
            self.body.append(statement)

    def extend_with(self, statements):
        self.body.extend(statements)

    def get_variables(self):
        variables = []
        for statement in self.body:
            if isinstance(statement, CppVariable):
                variables.append(statement)
            elif isinstance(statement, CppControlFlow):
                variables.extend(statement.get_variables())
        return variables

    def pretty_str(self, indent = 0):
        if self.body:
            return "\n".join([pretty_str(stmt, indent) for stmt in self.body])
        else:
            return (" " * indent) + "[empty]"

    def __repr__(self):
        return str(self.body)


class CppVariable(CppBasicEntity):
    def __init__(self, scope, cursor = None):
        CppBasicEntity.__init__(self, scope)
        self.value = None
        if cursor is None:
            self.name   = "<anonymous>"
            self.result = "[type]"
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.VAR_DECL \
                or cursor.kind == clang.CursorKind.FIELD_DECL
        CppBasicEntity._read_cursor(self, cursor)
        self.name   = cursor.spelling or "<anonymous>"
        self.result = cursor.type.spelling
        children = list(cursor.get_children())
        if children and children[-1].kind != clang.CursorKind.TYPE_REF:
            self.value = _parse(children[-1], scope = self.scope, lazy = True)

    def pretty_str(self, indent = 0):
        indent = " " * indent
        return "{}{} {} = {}".format(indent, self.result, self.name,
                                     pretty_str(self.value))

    def __repr__(self):
        return "[{}] {} = ({})".format(self.result, self.name, self.value)


class CppNamedEntity(CppBasicEntity):
    def __init__(self, scope, name = "<anonymous>", filename = None,
                 line = 0, column = 0):
        CppBasicEntity.__init__(self, scope, filename, line, column)
        self.name   = name
        self.id     = scope.id + "::" + self.name

    def _read_cursor(self, cursor):
        CppBasicEntity._read_cursor(self, cursor)
        self.name   = cursor.spelling if cursor.spelling else "<anonymous>"
        self.id     = cursor.get_usr() or self.scope.id + "::" + self.name

    def __repr__(self):
        return self.name


class CppFunction(CppNamedEntity):
    def __init__(self, scope, cursor = None):
        CppNamedEntity.__init__(self, scope)
        if cursor is None:
            self.signature      = "<anonymous>()"
            self.result         = "[type]"
            self.parameters     = []
            self.body           = CppBlock(self)
            self.is_constructor = False
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.FUNCTION_DECL \
                or cursor.kind == clang.CursorKind.CXX_METHOD \
                or cursor.kind == clang.CursorKind.CONSTRUCTOR \
                or cursor.kind == clang.CursorKind.DESTRUCTOR
        CppNamedEntity._read_cursor(self, cursor)
        self.signature      = cursor.displayname
        self.result         = cursor.result_type.spelling
        self.parameters     = []
        self.body           = CppBlock(self)
        self.is_method = cursor.kind == clang.CursorKind.CXX_METHOD
        self.is_constructor = cursor.kind == clang.CursorKind.CONSTRUCTOR
        self.is_destructor = cursor.kind == clang.CursorKind.DESTRUCTOR
        self._traverse(cursor)

    def _traverse(self, cursor):
        children = cursor.get_children()
        c = next(children, None)
        while c:
            if c.kind == clang.CursorKind.PARM_DECL:
                self.parameters.append(c.displayname)
            elif c.kind == clang.CursorKind.MEMBER_REF:
                # This is for constructors, we need the sibling
                member = CppReference.from_cursor(c, scope = self)
                value = _parse(next(children), scope = self, lazy = True)
                result = member.result
                assignment = CppOperator(self, "=", result, (member, value))
                self.body.append_statement(assignment)
            elif c.kind == clang.CursorKind.COMPOUND_STMT:
                self.body.extend_with(_parse_body(c, scope = self))
            c = next(children, None)

    def get_variables(self):
        return self.body.get_variables()

    def pretty_str(self, indent = 0):
        spaces = " " * indent
        params = ", ".join(self.parameters)
        pretty = "{}{} {}({}):\n".format(spaces, self.result, self.name, params)
        pretty += self.body.pretty_str(indent + 2)
        return pretty

    def __repr__(self):
        params = ", ".join([p for p in self.parameters])
        return "[{}] {}({})".format(self.result, self.name, params)


class CppClass(CppNamedEntity):
    def __init__(self, scope, cursor = None):
        CppNamedEntity.__init__(self, scope)
        if cursor is None:
            self.children       = []
            self.fields         = []
            self.methods        = []
            self.constructors   = []
            self.destructors    = []
            self.superclasses   = []
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.CLASS_DECL
        CppNamedEntity._read_cursor(self, cursor)
        self.children       = []
        self.fields         = []
        self.methods        = []
        self.constructors   = []
        self.destructors    = []
        self.superclasses   = []
        CppBasicEntity._traverse(self, cursor)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CXX_METHOD:
            child = CppFunction(self, cursor = cursor)
            self.children.append(child)
            self.methods.append(child)
        elif cursor.kind == clang.CursorKind.FIELD_DECL:
            child = CppVariable(self, cursor = cursor)
            self.children.append(child)
            self.fields.append(child)
        elif cursor.kind == clang.CursorKind.CONSTRUCTOR:
            child = CppFunction(self, cursor = cursor)
            self.children.append(child)
            self.constructors.append(child)
        elif cursor.kind == clang.CursorKind.DESTRUCTOR:
            child = CppFunction(self, cursor = cursor)
            self.children.append(child)
            self.destructors.append(child)
        elif cursor.kind == clang.CursorKind.CXX_BASE_SPECIFIER:
            self.superclasses.append(cursor.spelling)
        else:
            CppBasicEntity._traverse(self, cursor)

    def pretty_str(self, indent = 0):
        spaces = " " * indent
        pretty = spaces + "class " + self.name
        if self.superclasses:
            superclasses = ", ".join(self.superclasses)
            pretty += "(" + superclasses + ")"
        pretty += ":\n"
        pretty += "\n\n".join([c.pretty_str(indent + 2) for c in self.children])
        return pretty

    def __repr__(self):
        return "[class {}]".format(self.name)


class CppNamespace(CppNamedEntity):
    def __init__(self, scope, cursor = None):
        CppNamedEntity.__init__(self, scope)
        if cursor is None:
            self.children   = []
            self.variables  = []
            self.functions  = []
            self.classes    = []
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.NAMESPACE
        CppNamedEntity._read_cursor(self, cursor)
        self.children   = []
        self.variables  = []
        self.functions  = []
        self.classes    = []
        CppBasicEntity._traverse(self, cursor)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CLASS_DECL:
            child = CppClass(self, cursor = cursor)
            self.children.append(child)
            self.classes.append(child)
        elif cursor.kind == clang.CursorKind.FUNCTION_DECL \
                or cursor.kind == clang.CursorKind.CXX_METHOD \
                or cursor.kind == clang.CursorKind.CONSTRUCTOR \
                or cursor.kind == clang.CursorKind.DESTRUCTOR:
            child = CppFunction(self, cursor = cursor)
            self.children.append(child)
            self.functions.append(child)
        elif cursor.kind == clang.CursorKind.VAR_DECL:
            child = CppVariable(self, cursor = cursor)
            self.children.append(child)
            self.variables.append(child)
        else:
            CppBasicEntity._traverse(self, cursor)

    def pretty_str(self, indent = 0):
        spaces = " " * indent
        pretty = spaces + "namespace " + self.name + ":\n"
        pretty += "\n\n".join([c.pretty_str(indent + 2) for c in self.children])
        return pretty

    def __repr__(self):
        return "[namespace {}]".format(self.name)


class CppGlobalScope(object):
    def __init__(self):
        self.id         = "<global>"
        self.children   = []
        self.namespaces = []
        self.classes    = []
        self.functions  = []
        self.variables  = []

    def add_from_cursor(self, cursor):
        if cursor.kind == clang.CursorKind.NAMESPACE:
            child = CppNamespace(self, cursor = cursor)
            self.children.append(child)
            self.namespaces.append(child)
        elif cursor.kind == clang.CursorKind.CLASS_DECL:
            child = CppClass(self, cursor = cursor)
            self.children.append(child)
            self.classes.append(child)
        elif cursor.kind == clang.CursorKind.FUNCTION_DECL:
            child = CppFunction(self, cursor = cursor)
            self.children.append(child)
            self.functions.append(child)
        elif cursor.kind == clang.CursorKind.VAR_DECL:
            child = CppVariable(self, cursor = cursor)
            self.children.append(child)
            self.variables.append(child)

    def pretty_str(self):
        return "\n\n".join([c.pretty_str() for c in self.children])

CppGlobalScope.INSTANCE = CppGlobalScope()



###############################################################################
# Parsing Functions
###############################################################################


# NOTE: sometimes there are literals without tokens. It seems to happen only
#       with ROS_INFO and/or macro-related stuff.
# NOTE: most of the stuff without tokens comes from built-in or macro-related
#       stuff. The only interesting situation without tokens is default args.
def _parse(cursor, **options):
    scope = options.get("scope", CppGlobalScope.INSTANCE)
    if cursor.kind == clang.CursorKind.STRING_LITERAL:
        return cursor.spelling
    if cursor.kind == clang.CursorKind.INTEGER_LITERAL:
        token = next(cursor.get_tokens(), None)
        if token:
            if token.spelling.startswith("0x"):
                return int(token.spelling, 16)
            if token.spelling[-1].isalpha():
                if token.spelling[-2].isalpha():
                    return int(token.spelling[:-2], 0)
                return int(token.spelling[:-1], 0)
            return int(token.spelling, 0)
        return 0
    if cursor.kind == clang.CursorKind.FLOATING_LITERAL:
        token = next(cursor.get_tokens(), None)
        if token:
            if token.spelling[-1].isalpha():
                return float(token.spelling[:-1])
            return float(token.spelling)
        return 0.0
    if cursor.kind == clang.CursorKind.CHARACTER_LITERAL:
        token = next(cursor.get_tokens(), None)
        return token.spelling if token else "\0"
    if cursor.kind == clang.CursorKind.CXX_BOOL_LITERAL_EXPR:
        token = next(cursor.get_tokens(), None)
        return token and token.spelling == "true"
    if cursor.kind == clang.CursorKind.VAR_DECL:
        return CppVariable.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.CXX_THIS_EXPR:
        return CppReference(scope, name = "this", result = cursor.type.spelling)
    if cursor.kind == clang.CursorKind.DECL_REF_EXPR \
            or cursor.kind == clang.CursorKind.MEMBER_REF_EXPR:
        return CppReference.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.CALL_EXPR:
        if cursor.spelling == "basic_string":
            return _parse_basic_string(cursor, scope)
        return CppFunctionCall.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.WHILE_STMT \
            or cursor.kind == clang.CursorKind.FOR_STMT \
            or cursor.kind == clang.CursorKind.DO_STMT \
            or cursor.kind == clang.CursorKind.IF_STMT \
            or cursor.kind == clang.CursorKind.SWITCH_STMT:
        if not next(cursor.get_tokens(), None):
            # This is to try to avoid ROS_INFO and similar things.
            return None
        return CppControlFlow.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.UNARY_OPERATOR \
            or cursor.kind == clang.CursorKind.BINARY_OPERATOR \
            or cursor.kind == clang.CursorKind.COMPOUND_ASSIGNMENT_OPERATOR:
        if not options.get("lazy", False):
            return CppOperator.from_cursor(cursor, scope = scope)
        return LazyCppEntity(scope, cursor)
    if cursor.kind == clang.CursorKind.RETURN_STMT \
            or cursor.kind == clang.CursorKind.BREAK_STMT \
            or cursor.kind == clang.CursorKind.CONTINUE_STMT:
        return CppStatement.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.COMPOUND_STMT:
        return CppBlock.from_cursor(cursor, scope = scope)
    if cursor.kind == clang.CursorKind.CXX_FUNCTIONAL_CAST_EXPR:
        children = list(cursor.get_children())
        if children and children[-1].kind == clang.CursorKind.UNEXPOSED_EXPR:
            return _parse(children[-1], **options)
    if options.get("arguments", False):
        if cursor.kind == clang.CursorKind.UNEXPOSED_EXPR:
            child = next(cursor.get_children(), None)
            if not child:
                return "[{}] (default)".format(cursor.type.spelling)
        for c in cursor.get_children():
            result = _parse(c, **options)
            if not result is None:
                return result
        return None
    child = next(cursor.get_children(), None)
    return _parse(child, **options) if child else None


def _parse_body(cursor, scope = None):
    body = []
    if cursor.kind == clang.CursorKind.NULL_STMT:
        return body
    if cursor.kind == clang.CursorKind.COMPOUND_STMT:
        children = list(cursor.get_children())
    else:
        children = [cursor]
    for c in children:
        statement = _parse(c, scope = scope)
        if statement:
            body.append(statement)
    return body


def _parse_basic_string(cursor, scope = None):
    child = next(cursor.get_children(), None)
    if not child:
        return ""
    if child.kind == clang.CursorKind.UNEXPOSED_EXPR:
        child = next(child.get_children(), None)
        if not child:
            return None
        if child.kind == clang.CursorKind.STRING_LITERAL:
            return child.spelling
        return CppFunctionCall.from_cursor(cursor, scope = scope)
    return None



###############################################################################
# Script Functions
###############################################################################


def pretty_str(something, indent = 0):
    if isinstance(something, CppBasicEntity):
        return something.pretty_str(indent = indent)
    else:
        return (" " * indent) + str(something)



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
    for f in collect_functions():
        c = FunctionCollector(f)
        if c.has_results():
            print "FUNCTION " + f.name + ":"
            c.show_results()
            print ""



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
    # file_path = "/home/andre/catkin_ws/src/beginner_tutorial/src/listener.cpp"
    # file_path = "/home/andre/catkin_ws/src/beginner_tutorial/src/talker.cpp"
    file_path = "/home/andre/kobuki/src/kobuki/kobuki_node/src/library/kobuki_ros.cpp"
    clang.Config.set_library_path("/usr/lib/llvm-3.8/lib")
    index = clang.Index.create()
    # print_ast(file_path, index)
    analysis(file_path, index)


###############################################################################
# Main Runnable
###############################################################################

if __name__ == "__main__":
    main()
