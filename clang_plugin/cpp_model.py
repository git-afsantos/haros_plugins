
###
# standard packages
from ctypes import ArgumentError

###
# third-party packages
import clang.cindex as clang

###
# internal packages
from haros_util.events import Event

advertise_list = []
subscribe_list = []
function_counter = 0

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
        self._actual = None

    @property
    def name(self):
        if not self._actual:
            self._actual = self.evaluate()
        return self._actual.name

    @property
    def result(self):
        if not self._actual:
            self._actual = self.evaluate()
        return self._actual.result

    def evaluate(self):
        self._actual = _parse(self.cursor, scope = self.scope, lazy = False)
        return self._actual

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

    @property
    def variables(self):
        return []

    def pretty_str(self, indent = 0):
        return (" " * indent) + self.name

    def __repr__(self):
        return "[{}] {}".format(self.result, self.name)


class CppDefaultArgument(CppExpression):
    def __init__(self, scope, result = "[type]", cursor = None):
        CppExpression.__init__(self, scope, "(default)", result, cursor)

    def _read_cursor(self, cursor):
        # I think extracting file name causes problems...
        # CppBasicEntity._read_cursor(self, cursor)
        self.name   = "(default)"
        self.result = cursor.type.spelling or "[type]"


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

    @property
    def variables(self):
        return [self.name]

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
            self.arguments  = (_parse(arg, scope = self.scope),)
            self.is_unary   = True
            self.is_binary  = False
        else:
            self.name       = CppOperator._parse_binary(cursor)
            args            = list(cursor.get_children())
            if len(args) >= 2:
                self.arguments  = (_parse(args[0], scope = self.scope),
                                   _parse(args[1], scope = self.scope))
                self.is_unary   = False
                self.is_binary  = True
            else:
                print "[CLANG] binary op with less than two args", self.name
                self.arguments = ()
                self.is_unary   = False
                self.is_binary  = False

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
                if not token[0].isalpha():
                    if token == "++" or token == "--":
                        return "_" + token
                else:
                    print "[CLANG] unknown unary operator", [t.spelling for t in tokens]
            elif token in CppOperator._unary_tokens:
                return token
        return "[op]"

    @staticmethod
    def _parse_binary(cursor):
        # There are no alpha operators
        # I think "->" and "->*" might have their own CursorKind
        # All operators seem to be infix; get the last token of the first child
        child = next(cursor.get_children(), None)
        if not child:
            print "[CLANG] binary op without child", cursor.spelling
            return "[op]"
        tokens = list(child.get_tokens())
        if tokens:
            token = tokens[-1].spelling
            if token in CppOperator._binary_tokens:
                return token
        return "[op]"

    @property
    def variables(self):
        vs = []
        for a in self.arguments:
            if isinstance(a, CppExpression):
                vs.extend(a.variables)
        return vs

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
    onInstance = Event()

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
        CppFunctionCall.onInstance(self)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CALL_EXPR and cursor.spelling:
            self.name = cursor.spelling

            if self.name == "advertise":
                advertise_list.append(self.result)
            elif self.name == "subscribe":
                subscribe_list.append(self.result)

            self.is_constructor = self.result.split("::")[-1] == self.name
            args = list(cursor.get_arguments())
            children = [c for c in cursor.get_children() if not c in args]
            self._parse_non_arguments(children)
            if not args and self.is_constructor:
                args = children
            self.arguments = [_parse(arg, scope = self.scope, arguments = True) \
                              for arg in args]
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
            # TODO multiple templates
            template = []
            for cursor in children[1:]:
                if cursor.kind == clang.CursorKind.NAMESPACE_REF \
                        or cursor.kind == clang.CursorKind.TYPE_REF:
                    template.append(cursor.spelling)
            if template:
                self.template = "::".join(template)

    @property
    def variables(self):
        vs = []
        for a in self.arguments:
            if isinstance(a, CppExpression):
                vs.extend(a.variables)
        return vs

    def simplify(self):
        i = 0
        while i < len(self.arguments):
            if isinstance(self.arguments[i], LazyCppEntity):
                self.arguments[i] = self.arguments[i].evaluate()
            i += 1

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
        condition = _parse(children[0], scope = self)
        body = CppBlock.from_cursor(children[1], scope = self)
        branches = [(condition, body)]
        if len(children) >= 3:
            # this is the "else" branch
            # condition = CppOperator(self, "!", "bool", (condition,))
            condition = True
            body = CppBlock.from_cursor(children[2], scope = self)
            branches.append((condition, body))
        return branches

    def _parse_switch(self, cursor):
        # switch is hard to parse, this is neither correct nor pretty
        children = list(cursor.get_children())
        assert len(children) >= 2
        var = _parse(children[0], scope = self)
        if children[1].kind == clang.CursorKind.COMPOUND_STMT:
            children = list(children[1].get_children())
        else:
            children = children[1:]
        branches = []
        condition = True
        body = None
        for node in children:
            if node.kind == clang.CursorKind.CASE_STMT:
                if body:
                    branches.append((condition, body))
                statements = list(node.get_children())
                body = CppBlock(self)
                value = _parse(statements[0], scope = self)
                condition = CppOperator(self, "==", "bool", (var, value))
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
            condition = _parse(children[0], scope = self)
        elif len(children) >= 4:
            # var + condition + increment + body
            condition = _parse(children[1], scope = self)
            body.append_statement(_parse(children[2], scope = body))
            self.variables.append(_parse(children[0], scope = self))
        elif children[0].kind == clang.CursorKind.DECL_STMT:
            # var + condition + body
            condition = _parse(children[1], scope = self)
            self.variables.append(_parse(children[0], scope = self))
        else:
            # condition + increment + body
            condition = _parse(children[0], scope = self)
            body.append_statement(_parse(children[1], scope = body))
        return [(condition, body)]

    def _parse_while(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 2
        condition = _parse(children[0], scope = self)
        body = CppBlock.from_cursor(children[1], scope = self)
        return [(condition, body)]

    def _parse_do(self, cursor):
        children = list(cursor.get_children())
        assert len(children) >= 2
        condition = _parse(children[1], scope = self)
        body = CppBlock.from_cursor(children[0], scope = self)
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
            self.value  = _parse(expression, scope = self.scope) \
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
            self.value = _parse(children[-1], scope = self.scope)

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
            self.is_destructor  = False
        else:
            self._read_cursor(cursor)

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.FUNCTION_DECL \
                or cursor.kind == clang.CursorKind.CXX_METHOD \
                or cursor.kind == clang.CursorKind.CONSTRUCTOR \
                or cursor.kind == clang.CursorKind.DESTRUCTOR
        global function_counter
        function_counter += 1
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
                value = _parse(next(children), scope = self)
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
            #self.fields         = []
            #self.methods        = []
            #self.constructors   = []
            #self.destructors    = []
            self.superclasses   = []
        else:
            self._read_cursor(cursor)

    @property
    def functions(self):
        return [c for c in self.children if isinstance(c, CppFunction)]

    @property
    def fields(self):
        return [c for c in self.children if isinstance(c, CppVariable)]

    @property
    def methods(self):
        return [c for c in self.children \
                  if isinstance(c, CppFunction) and c.is_method]

    @property
    def constructors(self):
        return [c for c in self.children \
                  if isinstance(c, CppFunction) and c.is_constructor]

    @property
    def destructors(self):
        return [c for c in self.children \
                  if isinstance(c, CppFunction) and c.is_destructor]

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.CLASS_DECL
        CppNamedEntity._read_cursor(self, cursor)
        self.children       = []
        #self.fields         = []
        #self.methods        = []
        #self.constructors   = []
        #self.destructors    = []
        self.superclasses   = []
        CppBasicEntity._traverse(self, cursor)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CXX_METHOD:
            self.children.append(CppFunction(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.FIELD_DECL:
            self.children.append(CppVariable(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.CONSTRUCTOR:
            self.children.append(CppFunction(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.DESTRUCTOR:
            self.children.append(CppFunction(self, cursor = cursor))
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
            #self.variables  = []
            #self.functions  = []
            #self.classes    = []
        else:
            self._read_cursor(cursor)

    @property
    def namespaces(self):
        return [c for c in self.children if isinstance(c, CppNamespace)]

    @property
    def classes(self):
        return [c for c in self.children if isinstance(c, CppClass)]

    @property
    def functions(self):
        return [c for c in self.children if isinstance(c, CppFunction)]

    @property
    def variables(self):
        return [c for c in self.children if isinstance(c, CppVariable)]

    def _read_cursor(self, cursor):
        assert cursor.kind == clang.CursorKind.NAMESPACE
        CppNamedEntity._read_cursor(self, cursor)
        self.children   = []
        #self.variables  = []
        #self.functions  = []
        #self.classes    = []
        CppBasicEntity._traverse(self, cursor)

    def _traverse(self, cursor):
        if cursor.kind == clang.CursorKind.CLASS_DECL:
            self.children.append(CppClass(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.FUNCTION_DECL \
                or cursor.kind == clang.CursorKind.CXX_METHOD \
                or cursor.kind == clang.CursorKind.CONSTRUCTOR \
                or cursor.kind == clang.CursorKind.DESTRUCTOR:
            self.children.append(CppFunction(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.VAR_DECL:
            self.children.append(CppVariable(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.NAMESPACE:
            self.children.append(CppNamespace(self, cursor = cursor))
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
        #self.namespaces = []   maybe lists of visitors
        #self.classes    = []
        #self.functions  = []
        #self.variables  = []

    @property
    def namespaces(self):
        return [c for c in self.children if isinstance(c, CppNamespace)]

    @property
    def classes(self):
        return [c for c in self.children if isinstance(c, CppClass)]

    @property
    def functions(self):
        return [c for c in self.children if isinstance(c, CppFunction)]

    @property
    def variables(self):
        return [c for c in self.children if isinstance(c, CppVariable)]

    def add_from_cursor(self, cursor):
        # TODO missing ENUM_DECL, TYPEDEF_DECL, STRUCT_DECL
        # CLASS_TEMPLATE, FUNCTION_TEMPLATE, USING_DECLARATION
        # UNEXPOSED_DECL, USING_DIRECTIVE
        if cursor.kind == clang.CursorKind.NAMESPACE:
            self.children.append(CppNamespace(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.CLASS_DECL:
            self.children.append(CppClass(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.FUNCTION_DECL \
                or cursor.kind == clang.CursorKind.CXX_METHOD \
                or cursor.kind == clang.CursorKind.CONSTRUCTOR \
                or cursor.kind == clang.CursorKind.DESTRUCTOR:
            self.children.append(CppFunction(self, cursor = cursor))
        elif cursor.kind == clang.CursorKind.VAR_DECL:
            self.children.append(CppVariable(self, cursor = cursor))

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
        token = cursor.spelling
        if token.startswith('"'):
            token = token[1:-1]
        return token
    if cursor.kind == clang.CursorKind.INTEGER_LITERAL:
        token = next(cursor.get_tokens(), None)
        if token:
            token = token.spelling
            while token.endswith(("U", "u", "L", "l")):
                token = token[:-1]
            return int(token, 0)
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
                return CppDefaultArgument.from_cursor(cursor, scope = scope)
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
            token = child.spelling
            if token.startswith('"'):
                token = token[1:-1]
            return token
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
