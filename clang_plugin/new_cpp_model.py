###
# standard packages
from collections import deque
from ctypes import ArgumentError
import os

###
# third-party packages
import clang.cindex as clang


CK = clang.CursorKind

###############################################################################
# Notes to Self
###############################################################################

# This works with a queue of builders.
# Builders store cursors and other context specific information.
# The AST is built breadth-first, instead of depth-first.
# As the objects are built, they are added to their respective parents.

# After the program model is built, a traversal may be required to detect and
#   remove invalid or redundant constructs.


###############################################################################
# Language Model
###############################################################################

class CppEntity(object):
    def __init__(self, scope, parent):
        self.scope = scope
        self.parent = parent

    def walk_preorder(self):
        yield self
        for child in self._children():
            for descendant in child.walk_preorder():
                yield descendant

    def filter(self, cls, recursive = False):
        objects = []
        if recursive:
            for cppobj in self.walk_preorder():
                if isinstance(cppobj, cls):
                    objects.append(cppobj)
        else:
            if isinstance(self, cls):
                objects.append(self)
            for child in self._children():
                if isinstance(child, cls):
                    objects.append(child)
        return objects

    def _validity_check(self):
        return True

    def _children(self):
        return
        yield

    def _lookup_parent(self, cls):
        cppobj = self.parent
        while not cppobj is None and not isinstance(cppobj, cls):
            cppobj = cppobj.parent
        return cppobj


    def pretty_str(self, indent = 0):
        return (" " * indent) + self.__str__()

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return "[unknown]"


class CppStatementGroup(object):
    def statement(self, i):
        return self.body.statement(i)

    def statement_after(self, i):
        """Return the statement after the ith one, or None."""
        try:
            return self.statement(i + 1)
        except IndexError as e:
            return None

    def __len__(self):
        return len(self.body)

# ----- Common Entities -------------------------------------------------------

class CppVariable(CppEntity):
    def __init__(self, scope, parent, id, name, result):
        CppEntity.__init__(self, scope, parent)
        self.id = id
        self.name = name
        self.full_type = result
        self.result = result[6:] if result.startswith("const ") else result
        self.value = None
        self.member_of = None
        self.references = []
        self.writes = []

    @property
    def is_local(self):
        return (isinstance(self.scope, CppStatement)
                or (isinstance(self.scope, CppFunction)
                    and not self in self.scope.parameters))

    @property
    def is_global(self):
        return isinstance(self.scope, (CppGlobalScope, CppNamespace))

    @property
    def is_parameter(self):
        return (isinstance(self.scope, CppFunction)
                and self in self.scope.parameters)

    @property
    def is_member(self):
        return isinstance(self.scope, CppClass)


    def _add(self, cppobj):
        assert isinstance(cppobj, CppExpression.TYPES)
        self.value = cppobj

    def _children(self):
        if isinstance(self.value, CppEntity):
            yield self.value


    def pretty_str(self, indent = 0):
        indent = " " * indent
        return "{}{} {} = {}".format(indent, self.result, self.name,
                                     pretty_str(self.value))

    def __repr__(self):
        return "[{}] {} = ({})".format(self.result, self.name, self.value)


class CppFunction(CppEntity, CppStatementGroup):
    def __init__(self, scope, parent, id, name, result):
        CppEntity.__init__(self, scope, parent)
        self.id = id
        self.name = name
        self.full_type = result
        self.result = result[6:] if result.startswith("const ") else result
        self.parameters = []
        self.template_parameters = 0
        self.body = CppBlock(self, self, explicit = True)
        self.member_of = None
        self.references = []
        self._definition = self

    @property
    def is_definition(self):
        return self._definition is self

    @property
    def is_constructor(self):
        return self.member_of and self.name == self.member_of.name

    def _add(self, cppobj):
        assert isinstance(cppobj, (CppStatement, CppExpression))
        self.body._add(cppobj)

    def _children(self):
        for cppobj in self.parameters:
            yield cppobj
        for cppobj in self.body._children():
            yield cppobj

    def _afterpass(self):
        if hasattr(self, "_fi"):
            return
        fi = 0
        for cppobj in self.walk_preorder():
            cppobj._fi = fi
            fi += 1
            if isinstance(cppobj, CppOperator) and cppobj.is_assignment:
                if cppobj.arguments and isinstance(cppobj.arguments[0],
                                                   CppReference):
                    # left side can be CALL_EXPR: operator[] or operator()
                    # or ARRAY_SUBSCRIPT_EXPR: a[]
                    # or UNARY_OPERATOR: *a
                    # or PAREN_EXPR: (*a)
                    var = cppobj.arguments[0].reference
                    if isinstance(var, CppVariable):
                        var.writes.append(cppobj)


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        params = ", ".join(map(lambda p: p.result + " " + p.name,
                            self.parameters))
        if self.is_constructor:
            pretty = "{}{}({}):\n".format(spaces, self.name, params)
        else:
            pretty = "{}{} {}({}):\n".format(spaces, self.result,
                                             self.name, params)
        if not self._definition is self:
            pretty += spaces + "  [declaration]"
        else:
            pretty += self.body.pretty_str(indent + 2)
        return pretty

    def __repr__(self):
        params = ", ".join([str(p) for p in self.parameters])
        return "[{}] {}({})".format(self.result, self.name, params)


class CppClass(CppEntity):
    def __init__(self, scope, parent, id, name):
        CppEntity.__init__(self, scope, parent)
        self.id = id
        self.name = name
        self.members = []
        self.superclasses = []
        self.member_of = None
        self.references = []
        self._definition = self

    @property
    def is_definition(self):
        return True # TODO

    def _add(self, cppobj):
        assert isinstance(cppobj, (CppFunction, CppVariable, CppClass))
        self.members.append(cppobj)
        cppobj.member_of = self

    def _children(self):
        for cppobj in self.members:
            yield cppobj

    def _afterpass(self):
        for cppobj in self.members:
            if isinstance(cppobj, CppVariable):
                continue
            if not cppobj.is_definition:
                cppobj._definition.member_of = self
            cppobj._afterpass()


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        pretty = spaces + "class " + self.name
        if self.superclasses:
            superclasses = ", ".join(self.superclasses)
            pretty += "(" + superclasses + ")"
        pretty += ":\n"
        pretty += "\n\n".join([c.pretty_str(indent + 2) for c in self.members])
        return pretty

    def __repr__(self):
        return "[class {}]".format(self.name)


class CppNamespace(CppEntity):
    def __init__(self, scope, parent, name):
        CppEntity.__init__(self, scope, parent)
        self.name = name
        self.children = []

    def _add(self, cppobj):
        assert isinstance(cppobj, (CppNamespace, CppClass,
                                    CppFunction, CppVariable))
        self.children.append(cppobj)

    def _children(self):
        for cppobj in self.children:
            yield cppobj

    def _afterpass(self):
        for cppobj in self.children:
            if isinstance(cppobj, CppVariable):
                continue
            cppobj._afterpass()


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        pretty = spaces + "namespace " + self.name + ":\n"
        pretty += "\n\n".join([c.pretty_str(indent + 2) for c in self.children])
        return pretty

    def __repr__(self):
        return "[namespace {}]".format(self.name)


class CppGlobalScope(CppEntity):
    def __init__(self):
        CppEntity.__init__(self, None, None)
        self.children = []

    def _add(self, cppobj):
        assert isinstance(cppobj, (CppNamespace, CppClass,
                                    CppFunction, CppVariable))
        self.children.append(cppobj)

    def _children(self):
        for cppobj in self.children:
            yield cppobj

    def _afterpass(self):
        for cppobj in self.children:
            if isinstance(cppobj, CppVariable):
                continue
            cppobj._afterpass()


    def pretty_str(self, indent = 0):
        return "\n\n".join([cppobj.pretty_str(indent = indent) \
                            for cppobj in self.children])


# ----- Expression Entities ---------------------------------------------------

class CppExpression(CppEntity):
    def __init__(self, scope, parent, name, result, paren = False):
        CppEntity.__init__(self, scope, parent)
        self.name = name
        self.full_type = result
        self.result = result[6:] if result.startswith("const ") else result
        self.parenthesis = paren

    LITERALS = (int, long, float, bool, basestring)

    @property
    def function(self):
        return self._lookup_parent(CppFunction)

    @property
    def statement(self):
        return self._lookup_parent(CppStatement)

    def pretty_str(self, indent = 0):
        if self.parenthesis:
            return (" " * indent) + "(" + self.name + ")"
        return (" " * indent) + self.name

    def __repr__(self):
        return "[{}] {}".format(self.result, self.name)


class SomeCpp(CppExpression):
    def __init__(self, result):
        CppExpression.__init__(self, None, None, result, result)

SomeCpp.INTEGER = SomeCpp("int")
SomeCpp.FLOATING = SomeCpp("float")
SomeCpp.CHARACTER = SomeCpp("char")
SomeCpp.BOOL = SomeCpp("bool")


CppExpression.TYPES = (int, long, float, bool,
                       basestring, SomeCpp, CppExpression)


class CppReference(CppExpression):
    def __init__(self, scope, parent, name, result):
        CppExpression.__init__(self, scope, parent, name, result)
        self.field_of = None
        self.reference = None

    def _set_field(self, cppobj):
        assert isinstance(cppobj, CppExpression)
        self.field_of = cppobj

    def _children(self):
        if self.field_of:
            yield self.field_of


    def pretty_str(self, indent = 0):
        spaces = (" " * indent)
        pretty = "{}({})" if self.parenthesis else "{}{}"
        name = self.name
        if self.field_of:
            o = self.field_of
            if isinstance(o, CppFunctionCall) and o.name == "operator->":
                name = o.arguments[0].pretty_str() + "->" + self.name
            else:
                name = o.pretty_str() + "." + self.name
        return pretty.format(spaces, name)

    def __str__(self):
        return "#" + self.name

    def __repr__(self):
        if self.field_of:
            return "[{}] ({}).{}".format(self.result, self.field_of, self.name)
        return "[{}] #{}".format(self.result, self.name)


class CppOperator(CppExpression):
    _UNARY_TOKENS = ("+", "-", "++", "--", "*", "&", "!", "~")

    _BINARY_TOKENS = ("+", "-", "*", "/", "%", "&", "|", "^", "<<", ">>",
                      "<", ">", "<=", ">=", "==", "!=", "&&", "||", "=",
                      "+=", "-=", "*=", "/=", "%=", "<<=", ">>=", "&=",
                      "|=", "^=", ",")

    def __init__(self, scope, parent, name, result, args = None):
        CppExpression.__init__(self, scope, parent, name, result)
        self.arguments = args or ()

    @property
    def is_unary(self):
        return len(self.arguments) == 1

    @property
    def is_binary(self):
        return len(self.arguments) == 2

    @property
    def is_assignment(self):
        return (self.name == "=" or self.name == "+=" or self.name == "-="
                or self.name == "*=" or self.name == "/=" or self.name == "%="
                or self.name == "&=" or self.name == "|=" or self.name == "^="
                or self.name == "<<=" or self.name == ">>=")

    def _add(self, cppobj):
        assert isinstance(cppobj, CppExpression.TYPES)
        self.arguments = self.arguments + (cppobj,)

    def _children(self):
        for cppobj in self.arguments:
            if isinstance(cppobj, CppExpression):
                yield cppobj


    def pretty_str(self, indent = 0):
        indent = (" " * indent)
        pretty = "{}({})" if self.parenthesis else "{}{}"
        operator = self.name
        if self.is_unary:
            if self.name.startswith("_"):
                operator = pretty_str(self.arguments[0]) + self.name[1:]
            else:
                operator += pretty_str(self.arguments[0])
        else:
            operator = "{} {} {}".format(pretty_str(self.arguments[0]),
                                         self.name,
                                         pretty_str(self.arguments[1]))
        return pretty.format(indent, operator)

    def __repr__(self):
        if self.is_unary:
            return "[{}] {}({})".format(self.result, self.name,
                                        self.arguments[0])
        elif self.is_binary:
            return "[{}] ({}){}({})".format(self.result, self.arguments[0],
                                            self.name, self.arguments[1])
        return "[{}] {}".format(self.result, self.name)


class CppFunctionCall(CppExpression):
    def __init__(self, scope, parent, name, result):
        CppExpression.__init__(self, scope, parent, name, result)
        self.full_name = name
        self.template = None
        self.arguments = ()
        self.method_of = None
        self.reference = None

    @property
    def is_constructor(self):
        result = self.result.split("::")[-1]
        if result.endswith(" *"):
            result = result[:-2]
        return result == self.name

    def _add(self, cppobj):
        assert isinstance(cppobj, CppExpression.TYPES)
        self.arguments = self.arguments + (cppobj,)

    def _set_method(self, cppobj):
        assert isinstance(cppobj, CppExpression)
        self.method_of = cppobj
        self.full_name = cppobj.result + "::" + self.name

    def _children(self):
        if self.method_of:
            yield self.method_of
        for cppobj in self.arguments:
            if isinstance(cppobj, CppExpression):
                yield cppobj


    def pretty_str(self, indent = 0):
        indent = " " * indent
        pretty = "{}({})" if self.parenthesis else "{}{}"
        call = self.name
        operator = self.name[8:]
        args = [pretty_str(arg) for arg in self.arguments]
        if operator in CppOperator._BINARY_TOKENS:
            call = "{} {} {}".format(args[0], operator, args[1])
        else:
            temp = "<" + ",".join(self.template) + ">" if self.template else ""
            args = ", ".join(args)
            if self.method_of:
                o = self.method_of
                if isinstance(o, CppFunctionCall) and o.name == "operator->":
                    call = "{}->{}{}({})".format(o.arguments[0].pretty_str(),
                                                 self.name, temp, args)
                else:
                    call = "{}.{}{}({})".format(o.pretty_str(),
                                                self.name, temp, args)
            elif self.is_constructor:
                call = "new {}{}({})".format(self.name, temp, args)
            else:
                call = "{}{}({})".format(self.name, temp, args)
        return pretty.format(indent, call)

    def __repr__(self):
        temp = "<" + ",".join(self.template) + ">" if self.template else ""
        args = ", ".join([str(arg) for arg in self.arguments])
        if self.is_constructor:
            return "[{}] new {}({})".format(self.result, self.name, args)
        if self.method_of:
            return "[{}] {}.{}{}({})".format(self.result, self.method_of.name,
                                           self.name, temp, args)
        return "[{}] {}{}({})".format(self.result, self.name, temp, args)


class CppDefaultArgument(CppExpression):
    def __init__(self, scope, parent, result):
        CppExpression.__init__(self, scope, parent, "(default)", result)

# ----- Statement Entities ----------------------------------------------------

class CppStatement(CppEntity):
    def __init__(self, scope, parent):
        CppEntity.__init__(self, scope, parent)
        self._si = -1

    @property
    def function(self):
        return self._lookup_parent(CppFunction)


class CppJumpStatement(CppStatement):
    def __init__(self, scope, parent, name):
        CppStatement.__init__(self, scope, parent)
        self.name = name
        self.value = None

    def _add(self, cppobj):
        assert isinstance(cppobj, CppExpression.TYPES)
        self.value = cppobj

    def _children(self):
        if isinstance(self.value, CppExpression):
            yield self.value


    def pretty_str(self, indent = 0):
        indent = " " * indent
        if not self.value is None:
            return indent + self.name + " " + pretty_str(self.value)
        return indent + self.name

    def __repr__(self):
        if not self.value is None:
            return self.name + " " + str(self.value)
        return self.name


class CppExpressionStatement(CppStatement):
    def __init__(self, scope, parent, expression = None):
        CppStatement.__init__(self, scope, parent)
        self.expression = expression

    def _children(self):
        if isinstance(self.expression, CppExpression):
            yield self.expression

    def pretty_str(self, indent = 0):
        return pretty_str(self.expression, indent = indent)

    def __repr__(self):
        return repr(self.expression)


class CppBlock(CppStatement, CppStatementGroup):
    def __init__(self, scope, parent, explicit = True):
        CppStatement.__init__(self, scope, parent)
        self.body = []
        self.explicit = explicit

    def statement(self, i):
        return self.body[i]

    def _add(self, cppobj):
        assert isinstance(cppobj, CppStatement)
        cppobj._si = len(self.body)
        self.body.append(cppobj)

    def _children(self):
        for cppobj in self.body:
            yield cppobj


    def pretty_str(self, indent = 0):
        if self.body:
            return "\n".join([stmt.pretty_str(indent) for stmt in self.body])
        else:
            return (" " * indent) + "[empty]"

    def __repr__(self):
        return str(self.body)


class CppDeclaration(CppStatement):
    def __init__(self, scope, parent):
        CppStatement.__init__(self, scope, parent)
        self.variables = []

    def _add(self, cppobj):
        assert isinstance(cppobj, CppVariable)
        self.variables.append(cppobj)

    def _children(self):
        for cppobj in self.variables:
            yield cppobj


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        return spaces + ", ".join([v.pretty_str() for v in self.variables])

    def __repr__(self):
        return str(self.variables)


class CppControlFlow(CppStatement, CppStatementGroup):
    def __init__(self, scope, parent, name):
        CppStatement.__init__(self, scope, parent)
        self.name       = name
        self.condition  = True
        self.body       = CppBlock(scope, self, explicit = False)

    def get_branches(self):
        return [(self.condition, self.body)]

    def _set_condition(self, condition):
        assert isinstance(condition, CppExpression.TYPES)
        self.condition = condition

    def _set_body(self, body):
        assert isinstance(body, CppStatement)
        if isinstance(body, CppBlock):
            self.body = body
        else:
            self.body._add(body)

    def _children(self):
        if isinstance(self.condition, CppExpression):
            yield self.condition
        for cppobj in self.body._children():
            yield cppobj

    def __repr__(self):
        return "{} {}".format(self.name, self.get_branches())


class CppConditional(CppControlFlow):
    def __init__(self, scope, parent):
        CppControlFlow.__init__(self, scope, parent, "if")
        self.else_body = CppBlock(scope, self, explicit = False)

    @property
    def then_branch(self):
        return (self.condition, self.body)

    @property
    def else_branch(self):
        return (True, self.else_body)

    def statement(self, i):
        """Behaves as if "then" and "else" were concatenated.
            This code is just to avoid creating a new list and
            returning a custom exception message.
        """
        o = len(self.body)
        n = o + len(self.else_body)
        if i >= 0 and i < n:
            if i < o:
                return self.body.statement(i)
            return self.else_body.statement(i - o)
        elif i < 0 and i >= -n:
            if i >= o - n:
                return self.else_body.statement(i)
            return self.body.statement(i - o + n)
        raise IndexError("statement index out of range")

    def statement_after(self, i):
        k = i + 1
        o = len(self.body)
        n = o + len(self.else_body)
        if k > 0:
            if k < o:
                return self.body.statement(k)
            if k > o and k < n:
                return self.else_body.statement(k)
        if k < 0:
            if k < o - n and k > -n:
                return self.body.statement(k)
            if k > o - n:
                return self.else_body.statement(k)
        return None

    def get_branches(self):
        if self.else_branch:
            return [self.then_branch, self.else_branch]
        return [self.then_branch]

    def _add_default_branch(self, body):
        assert isinstance(body, CppStatement)
        if isinstance(body, CppBlock):
            self.else_body = body
        else:
            self.else_body._add(body)

    def __len__(self):
        return len(self.body) + len(self.else_body)

    def _children(self):
        if isinstance(self.condition, CppExpression):
            yield self.condition
        for cppobj in self.body._children():
            yield cppobj
        for cppobj in self.else_body._children():
            yield cppobj


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        condition = pretty_str(self.condition)
        pretty = spaces + "if (" + condition + "):\n"
        pretty += self.body.pretty_str(indent = indent + 2)
        if self.else_body:
            pretty += "\n" + spaces + "else:\n"
            pretty += self.else_body.pretty_str(indent = indent + 2)
        return pretty


class CppLoop(CppControlFlow):
    def __init__(self, scope, parent, name):
        CppControlFlow.__init__(self, scope, parent, name)
        self.declarations = None
        self.increment = None

    def _set_declarations(self, declarations):
        assert isinstance(declarations, CppStatement)
        self.declarations = declarations
        declarations.scope = self.body

    def _set_increment(self, statement):
        assert isinstance(statement, CppStatement)
        self.increment = statement
        statement.scope = self.body

    def _children(self):
        if self.declarations:
            yield self.declarations
        if isinstance(self.condition, CppExpression):
            yield self.condition
        if self.increment:
            yield self.increment
        for cppobj in self.body._children():
            yield cppobj


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        condition = pretty_str(self.condition)
        if self.name == "while":
            pretty = spaces + "while (" + condition + "):\n"
            pretty += self.body.pretty_str(indent = indent + 2)
        elif self.name == "do":
            pretty = spaces + "do:\n"
            pretty += self.body.pretty_str(indent = indent + 2)
            pretty += "\n" + spaces + "while (" + condition + ")"
        elif self.name == "for":
            v = self.declarations.pretty_str() if self.declarations else ""
            i = self.increment.pretty_str(indent = 1) if self.increment else ""
            pretty = spaces + "for ({}; {};{}):\n".format(v, condition, i)
            pretty += self.body.pretty_str(indent = indent + 2)
        return pretty


class CppSwitch(CppControlFlow):
    def __init__(self, scope, parent):
        CppControlFlow.__init__(self, scope, parent, "switch")
        self.cases = []
        self.default_case = None

    def _add_branch(self, value, statement):
        self.cases.append((value, statement))

    def _add_default_branch(self, statement):
        self.default_case = statement


    def pretty_str(self, indent = 0):
        spaces = " " * indent
        condition = pretty_str(self.condition)
        pretty = spaces + "switch (" + condition + "):\n"
        pretty += self.body.pretty_str(indent = indent + 2)
        return pretty



###############################################################################
# Language Entity Builders
###############################################################################

class CppEntityBuilder(object):
    def __init__(self, cursor, scope, parent, insert = None):
        self.scope  = scope
        self.parent = parent
        self.cursor = cursor
        self.insert_method = insert # how to link this entity to the parent
        self.file   = None
        self.line   = None
        self.column = None
        try:
            if cursor.location.file:
                self.file   = cursor.location.file.name
                self.line   = cursor.location.line
                self.column = cursor.location.column
        except ArgumentError as e:
            pass

    def build(self, data):
        """Build an object for the current cursor and
            corresponding builders for the cursor's children.
            Return None if an object cannot be built.
            Return (object, [builders]) otherwise.
        """
        return None

    # Let's add some methods here, just to avoid code duplication.

    def _build_variable(self, data):
        if self.cursor.kind == CK.VAR_DECL \
                or self.cursor.kind == CK.FIELD_DECL:
            id = self.cursor.get_usr()
            name = self.cursor.spelling
            result = self.cursor.type.spelling
            cppobj = CppVariable(self.scope, self.parent, id, name, result)
            data.register(cppobj)
            builders = []
            children = list(self.cursor.get_children())
            if children and children[-1].kind != CK.TYPE_REF:
                b = CppExpressionBuilder(children[-1], self.scope, cppobj)
                builders.append(b)
            return (cppobj, builders)
        return None


    def _lookup_parent(self, cls):
        cppobj = self.parent
        while not cppobj is None and not isinstance(cppobj, cls):
            cppobj = cppobj.parent
        return cppobj



class CppExpressionBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)
        self.name   = cursor.spelling
        self.result = cursor.type.spelling or "[type]"
        self.parenthesis = False

    def build(self, data):
        result = self._build_literal()
        if result is None:
            result = self._build_reference(data)
        if result is None:
            result = self._build_operator()
        if result is None:
            result = self._build_function_call(data)
        if result is None:
            result = self._build_default_argument()
        if result is None:
            result = self._build_other(data)
        if result is None:
            result = self._build_unexposed(data)
        return result


    def _build_literal(self):
        token = next(self.cursor.get_tokens(), None)
        if self.cursor.kind == CK.INTEGER_LITERAL:
            if token:
                token = token.spelling
                while token.endswith(("U", "u", "L", "l")):
                    token = token[:-1]
                return (int(token, 0), ())
            return (SomeCpp.INTEGER, ())
        if self.cursor.kind == CK.FLOATING_LITERAL:
            if token:
                if token.spelling[-1].isalpha():
                    return (float(token.spelling[:-1]), ())
                return (float(token.spelling), ())
            return (SomeCpp.FLOATING, ())
        if self.cursor.kind == CK.CHARACTER_LITERAL:
            return (token.spelling, ()) if token else (SomeCpp.CHARACTER, ())
        if self.cursor.kind == CK.CXX_BOOL_LITERAL_EXPR:
            return (token.spelling == "true", ()) if token \
                                                  else (SomeCpp.BOOL, ())
        if self.cursor.kind == CK.CALL_EXPR and self.name == "basic_string":
            cursor = next(self.cursor.get_children(), None)
            if not cursor:
                return ("", ())
            if cursor.kind == CK.UNEXPOSED_EXPR:
                cursor = next(cursor.get_children(), None)
                if cursor and cursor.kind == CK.STRING_LITERAL:
                    self.cursor = cursor
                    self.name = cursor.spelling
        if self.cursor.kind == CK.STRING_LITERAL:
            if self.name.startswith('"'):
                self.name = self.name[1:-1]
            return (self.name, ())
        return None

    def _build_reference(self, data):
        if self.cursor.kind == CK.DECL_REF_EXPR \
                or self.cursor.kind == CK.MEMBER_REF \
                or self.cursor.kind == CK.MEMBER_REF_EXPR:
            cppobj = CppReference(self.scope, self.parent,
                                  self.name, self.result)
            cppobj.parenthesis = self.parenthesis
            ref = self.cursor.get_definition()
            if ref:
                data.reference(ref.get_usr(), cppobj)
            if self.cursor.kind == CK.MEMBER_REF_EXPR:
                cursor = next(self.cursor.get_children(), None)
                if cursor:
                    builder = CppExpressionBuilder(cursor, self.scope, cppobj,
                                                   insert = cppobj._set_field)
                    return (cppobj, (builder,))
                else:
                    cppobj.field_of = CppReference(self.scope, cppobj,
                                                    "this", "[type]")
            return (cppobj, ())
        if self.cursor.kind == CK.CXX_THIS_EXPR:
            cppobj = CppReference(self.scope, self.parent, "this", self.result)
            cppobj.parenthesis = self.parenthesis
            return (cppobj, ())
        return None

    def _build_operator(self):
        # TODO conditional operator
        name = None
        if self.cursor.kind == CK.UNARY_OPERATOR:
            name = self._parse_unary_operator()
        elif self.cursor.kind == CK.BINARY_OPERATOR \
                or self.cursor.kind == CK.COMPOUND_ASSIGNMENT_OPERATOR:
            name = self._parse_binary_operator()
        if not name is None:
            cppobj = CppOperator(self.scope, self.parent, name, self.result)
            cppobj.parenthesis = self.parenthesis
            builders = [CppExpressionBuilder(c, self.scope, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None

    def _build_function_call(self, data):
        if self.cursor.kind == CK.CXX_NEW_EXPR:
            self.cursor = list(self.cursor.get_children())[-1]
            self.name = self.cursor.spelling
        # NOTE: this is not totally correct, needs revision
        if self.cursor.kind == CK.CALL_EXPR:
            if self.name:
                cppobj = CppFunctionCall(self.scope, self.parent,
                                         self.name, self.result)
                cppobj.parenthesis = self.parenthesis
    # ----- this is still tentative -------------------------------------------
                tokens = [t.spelling for t in self.cursor.get_tokens()]
                try:
                    cppobj.full_name = "".join(tokens[:tokens.index("(")])
                except ValueError as e:
                    if cppobj.is_constructor:
                        cppobj.full_name = cppobj.result + "::" + cppobj.name
                cppobj.template = self._parse_templates(cppobj.name, tokens)
    # -------------------------------------------------------------------------
                ref = self.cursor.get_definition() or self.cursor.referenced
                if ref:
                    data.reference(ref.get_usr(), cppobj)
                builders = []
                args = list(self.cursor.get_arguments())
                for cursor in self.cursor.get_children():
                    if cursor in args:
                        builders.append(CppExpressionBuilder(cursor,
                                        self.scope, cppobj))
                    elif cursor.kind == CK.MEMBER_REF_EXPR \
                            and cursor.spelling == self.name:
                        children = list(cursor.get_children())
                        if not children:
                            continue
                        builders.append(CppExpressionBuilder(children[0],
                                        self.scope, cppobj,
                                        insert = cppobj._set_method))
                if not args and cppobj.is_constructor:
                    for cursor in self.cursor.get_children():
                        builders.append(CppExpressionBuilder(cursor,
                                        self.scope, cppobj))
                return (cppobj, builders)
            else:
                result = None
                cursor = next(self.cursor.get_children(), None)
                if not cursor is None:
                    original    = self.cursor
                    self.cursor = cursor
                    self.name   = cursor.spelling
                    result      = self.build(data)
                    self.cursor = original
                elif isinstance(self.parent, CppVariable):
                    self.name   = self.result.split(":")[-1]
                    result      = self._build_function_call(data)
                return result
        elif self.cursor.kind == CK.CXX_DELETE_EXPR:
            cppobj = CppFunctionCall(self.scope, self.parent,
                                     "delete", self.result)
            cppobj.parenthesis = self.parenthesis
            ref = next(self.cursor.get_children())
            builder = CppExpressionBuilder(ref, self.scope, cppobj)
            return (cppobj, (builder,))
        return None

    def _build_default_argument(self):
        if isinstance(self.parent, CppFunctionCall) \
                and self.cursor.kind == CK.UNEXPOSED_EXPR \
                and not next(self.cursor.get_children(), None):
            cppobj = CppDefaultArgument(self.scope, self.parent, self.result)
            cppobj.parenthesis = self.parenthesis
            return (cppobj, ())
        return None

    def _build_other(self, data):
        # This is skip behaviour: parse child instead.
        if self.cursor.kind == CK.PAREN_EXPR \
                or self.cursor.kind == CK.CSTYLE_CAST_EXPR:
            original = self.cursor
            self.cursor = next(self.cursor.get_children(), None)
            self.parenthesis = original.kind == CK.PAREN_EXPR
            result = None
            if self.cursor:
                self.name = self.cursor.spelling
                result = self.build(data)
            self.cursor = original
            return result
        return None

    def _build_unexposed(self, data):
        if self.cursor.kind == CK.CXX_FUNCTIONAL_CAST_EXPR:
            children = list(self.cursor.get_children())
            if children and children[-1].kind == CK.UNEXPOSED_EXPR:
                self.cursor = children[-1]
        if self.cursor.kind == CK.UNEXPOSED_EXPR:
            cursor = next(self.cursor.get_children(), None)
            if cursor:
                self.cursor = cursor
                self.name   = cursor.spelling
                return self.build(data)
        return None
    

    def _parse_unary_operator(self):
        tokens = list(self.cursor.get_tokens())
        if tokens:
            token = tokens[0].spelling
            if token in CppOperator._UNARY_TOKENS:
                return token
            # The last token seems to be what ends the expression, e.g. ';'
            token = tokens[-2].spelling
            if token in CppOperator._UNARY_TOKENS:
                if token == "++" or token == "--":
                    return "_" + token
                return token
        return "[op]"

    def _parse_binary_operator(self):
        # There are no alpha operators
        # I think "->" and "->*" might have their own CursorKind
        # All operators seem to be infix; get the last token of the first child
        child = next(self.cursor.get_children(), None)
        if child:
            tokens = list(child.get_tokens())
            if tokens:
                token = tokens[-1].spelling
                if token in CppOperator._BINARY_TOKENS:
                    return token
        return "[op]"

    def _parse_templates(self, name, tokens):
        templates = []
        text = "".join(tokens)
        start = text.find("<")
        if (start >= 0 and not "<" in name and not ">" in name
                       and text[:start].endswith(name)):
            matches = 1
            i = start + 1
            while matches > 0 and i < len(text):
                if text[i] == "<":
                    matches += 1
                elif text[i] == ">":
                    matches -= 1
                elif text[i] == "," and matches == 1:
                    templates.append(text[start+1:i])
                    start = i
                i += 1
            templates.append(text[start+1:i-1])
        return tuple(templates)


class CppStatementBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)

    def build(self, data):
        result = self._build_declarations(data)
        result = result or self._build_expression(data)
        result = result or self._build_control_flow()
        result = result or self._build_jump_statement()
        result = result or self._build_block()
        result = result or self._build_unexposed(data)
        result = result or self._build_label_statement(data)
        return result


    def _build_expression(self, data):
        builder = CppExpressionBuilder(self.cursor, self.scope, self.parent)
        result = builder.build(data)
        if result:
            expression = result[0]
            cppobj = CppExpressionStatement(self.scope, self.parent,
                                            expression = expression)
            if isinstance(expression, CppExpression):
                expression.parent = cppobj
            result = (cppobj, result[1])
        return result

    def _build_declarations(self, data):
        if self.cursor.kind == CK.DECL_STMT:
            cppobj = CppDeclaration(self.scope, self.parent)
            original = self.cursor
            self.parent = cppobj
            builders = []
            for cursor in original.get_children():
                self.cursor = cursor
                result = self._build_variable(data)
                if result:
                    cppobj._add(result[0])
                    builders.extend(result[1])
            self.cursor = original
            self.parent = cppobj.parent
            return (cppobj, builders)
        return None

    def _build_control_flow(self):
        if not next(self.cursor.get_tokens(), None):
            # This is to try to avoid ROS_INFO and similar things.
            return None
        if self.cursor.kind == CK.WHILE_STMT:
            return self._build_while_statement()
        if self.cursor.kind == CK.FOR_STMT:
            return self._build_for_statement()
        if self.cursor.kind == CK.DO_STMT:
            return self._build_do_statement()
        if self.cursor.kind == CK.IF_STMT:
            return self._build_if_statement()
        if self.cursor.kind == CK.SWITCH_STMT:
            return self._build_switch_statement()
        return None

    def _build_jump_statement(self):
        if self.cursor.kind == CK.RETURN_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "return")
            expression  = next(self.cursor.get_children(), None)
            builders = [CppExpressionBuilder(expression, self.scope, cppobj)] \
                        if expression else ()
            return (cppobj, builders)
        if self.cursor.kind == CK.BREAK_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "break")
            return (cppobj, ())
        if self.cursor.kind == CK.CONTINUE_STMT:
            cppobj = CppJumpStatement(self.scope, self.parent, "continue")
            return (cppobj, ())
        return None

    def _build_block(self):
        if self.cursor.kind == CK.NULL_STMT:
            return None
        if self.cursor.kind == CK.COMPOUND_STMT:
            cppobj = CppBlock(self.scope, self.parent, explicit = True)
            builders = [CppStatementBuilder(c, cppobj, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None


    def _build_while_statement(self):
        cppobj = CppLoop(self.scope, self.parent, "while")
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        return (cppobj, builders)

    def _build_for_statement(self):
        """NOTE: this is not a complete implementation of for loop parsing.
            Turns out for loops allow a number of wacky things going on,
            such as declaring a variable in place of a condition.
            These more peculiar cases are not covered. See
            http://en.cppreference.com/w/cpp/language/for
        """
        cppobj = CppLoop(self.scope, self.parent, "for")
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 1
        # body always comes last
        builders.append(CppStatementBuilder(children[-1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        if len(children) == 1:
    # ----- just body -------------------------------------
            cppobj.condition = True
        elif len(children) == 2:
    # ----- condition + body ------------------------------
            builders.append(CppExpressionBuilder(children[0], self.scope,
                            cppobj, insert = cppobj._set_condition))
        elif len(children) >= 4:
    # ----- var + condition + increment + body ------------
            builders.append(CppStatementBuilder(children[0], cppobj,
                            cppobj, insert = cppobj._set_declarations))
            builders.append(CppExpressionBuilder(children[1], self.scope,
                            cppobj, insert = cppobj._set_condition))
            builders.append(CppStatementBuilder(children[2], cppobj,
                            cppobj, insert = cppobj._set_increment))
        elif children[0].kind == clang.CursorKind.DECL_STMT:
    # ----- var + condition + body ------------------------
            builders.append(CppStatementBuilder(children[0], cppobj,
                            cppobj, insert = cppobj._set_declarations))
            builders.append(CppExpressionBuilder(children[1], self.scope,
                            cppobj, insert = cppobj._set_condition))
        else:
    # ----- condition + increment + body ------------------
            builders.append(CppExpressionBuilder(children[0], self.scope,
                            cppobj, insert = cppobj._set_condition))
            builders.append(CppStatementBuilder(children[1], cppobj,
                            cppobj, insert = cppobj._set_increment))
        return (cppobj, builders)

    def _build_do_statement(self):
        cppobj = CppLoop(self.scope, self.parent, "do")
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppStatementBuilder(children[0], self.scope, cppobj,
                                            insert = cppobj._set_body))
        builders.append(CppExpressionBuilder(children[1], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        return (cppobj, builders)

    def _build_if_statement(self):
        cppobj = CppConditional(self.scope, self.parent)
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        if len(children) >= 3:
            # this is the "else" branch
            builders.append(CppStatementBuilder(children[2], self.scope, cppobj,
                            insert = cppobj._add_default_branch))
        return (cppobj, builders)

    def _build_switch_statement(self):
        """NOTE:
            This is not a complete implementation of switch statement parsing.
            The switch statement is probably one of the ugliest and less
            restrictive things in the language, and I am not going to support
            every possible case, especially not the pathological ones.
            See http://en.cppreference.com/w/cpp/language/switch
        """
        """The idea is to simply parse a body for the switch.
            Whenever a case is found, look up its switch parent and notify it.
            Return the result of parsing the statement within the case.
            Any statement body should assign a statement index to its children.
            This way, a jump can be represented as a parent plus index.
        """
        cppobj = CppSwitch(self.scope, self.parent)
        builders = []
        children = list(self.cursor.get_children())
        assert len(children) >= 2
        builders.append(CppExpressionBuilder(children[0], self.scope, cppobj,
                                             insert = cppobj._set_condition))
        builders.append(CppStatementBuilder(children[1], self.scope, cppobj,
                                            insert = cppobj._set_body))
        return (cppobj, builders)

    def _build_label_statement(self, data):
        original = self.cursor
        if self.cursor.kind == CK.CASE_STMT:
            switch = self._lookup_parent(CppSwitch)
            children = list(self.cursor.get_children())
            value = CppExpressionBuilder(children[0], self.scope, self.parent)
            value = value.build(data)
            self.cursor = children[1]
            result = self.build(data)
            if result:
                switch._add_branch(value, result[0])
            self.cursor = original
            return result
        if self.cursor.kind == CK.DEFAULT_STMT:
            switch = self._lookup_parent(CppSwitch)
            self.cursor = next(self.cursor.get_children())
            result = self.build(data)
            if result:
                switch._add_default_branch(result[0])
            self.cursor = original
            return result
        # TODO if self.cursor.kind == CK.LABEL_STMT:
        return None

    def _build_unexposed(self, data):
        if self.cursor.kind == CK.UNEXPOSED_STMT:
            cursor = next(self.cursor.get_children(), None)
            if cursor:
                self.cursor = cursor
                return self.build(data)
        return None



class CppTopLevelBuilder(CppEntityBuilder):
    def __init__(self, cursor, scope, parent, insert = None, workspace = ""):
        CppEntityBuilder.__init__(self, cursor, scope, parent, insert = insert)
        self.name = cursor.spelling
        self.workspace = workspace

    def build(self, data):
        result = self._build_variable(data)
        result = result or self._build_function(data)
        result = result or self._build_class(data)
        result = result or self._build_namespace()
        return result


    _FUNCTIONS = (CK.FUNCTION_DECL, CK.FUNCTION_TEMPLATE, CK.CXX_METHOD,
                  CK.CONSTRUCTOR, CK.DESTRUCTOR)

    def _build_function(self, data):
        #NOTE: function and method declarations only have children
        #       for their parameters. Only definitions have more.
        #       Skip declarations?
        if self.cursor.kind in CppTopLevelBuilder._FUNCTIONS:
            id = self.cursor.get_usr()
            result = self.cursor.result_type.spelling
            cppobj = CppFunction(self.scope, self.parent, id,
                                 self.name, result)
            builders = []
            declaration = True
            children = self.cursor.get_children()
            cursor = next(children, None)
            while cursor:
                if cursor.kind == CK.PARM_DECL:
                    id = cursor.get_usr()
                    name = cursor.spelling or cursor.displayname
                    result = cursor.type.spelling or "[type]"
                    var = CppVariable(cppobj, cppobj, id, name, result)
                    data.register(var)
                    cppobj.parameters.append(var)
                elif cursor.kind == CK.TEMPLATE_TYPE_PARAMETER:
                    cppobj.template_parameters += 1
                elif cursor.kind == CK.MEMBER_REF:
                    # This is for constructors, we need the sibling
                    declaration = False
                    result  = cursor.type.spelling or "[type]"
                    op      = CppOperator(cppobj, cppobj, "=", result)
                    member  = CppExpressionBuilder(cursor, cppobj, op)
                    cursor  = next(children)
                    value   = CppExpressionBuilder(cursor, cppobj, op)
                    stmt    = CppExpressionStatement(cppobj, cppobj, op)
                    op.parent = stmt
                    cppobj._add(stmt)
                    builders.append(member)
                    builders.append(value)
                elif cursor.kind == CK.COMPOUND_STMT:
                    declaration = False
                    for c in cursor.get_children():
                        builders.append(CppStatementBuilder(c, cppobj, cppobj))
                cursor = next(children, None)
            data.register(cppobj, declaration = declaration)
            return (cppobj, builders)
        return None

    def _build_class(self, data):
        if self.cursor.kind == CK.CLASS_DECL:
            id = self.cursor.get_usr()
            cppobj = CppClass(self.scope, self.parent, id, self.name)
            data.register(cppobj)
            builders = []
            for cursor in self.cursor.get_children():
                if cursor.kind == CK.CXX_BASE_SPECIFIER:
                    cppobj.superclasses.append(cursor.spelling)
                else:
                    builders.append(CppTopLevelBuilder(cursor, cppobj, cppobj))
            return (cppobj, builders)
        return None

    def _build_namespace(self):
        if self.cursor.kind == CK.NAMESPACE:
            cppobj = CppNamespace(self.scope, self.parent, self.name)
            builders = [CppTopLevelBuilder(c, cppobj, cppobj) \
                        for c in self.cursor.get_children()]
            return (cppobj, builders)
        return None



###############################################################################
# AST Analysis and Parsing
###############################################################################

class CppQuery(object):
    def __init__(self, cppobj):
        assert isinstance(cppobj, CppEntity)
        self.root = cppobj
        self.cls = None
        self.recursive = False
        self.attributes = {}

    @property
    def references(self):
        self.cls = CppReference
        self.recursive = False
        return self

    @property
    def all_references(self):
        self.cls = CppReference
        self.recursive = True
        return self

    @property
    def calls(self):
        self.cls = CppFunctionCall
        self.recursive = False
        return self

    @property
    def all_calls(self):
        self.cls = CppFunctionCall
        self.recursive = True
        return self

    def where_name(self, name):
        self.attributes["name"] = name
        return self

    def where_result(self, result):
        self.attributes["result"] = result
        return self

    def get(self):
        result = []
        for cppobj in self.root.filter(self.cls, recursive = self.recursive):
            passes = True
            for key, value in self.attributes.iteritems():
                if isinstance(value, basestring):
                    if getattr(cppobj, key) != value:
                        passes = False
                else:
                    if not getattr(cppobj, key) in value:
                        passes = False
            if passes:
                result.append(cppobj)
        return result


class AnalysisData(object):
    def __init__(self):
        self.entities   = {}    # USR -> CppEntity
        self._refs      = {}    # USR -> [CppEntity]

    def register(self, cppobj, declaration = False):
        previous = self.entities.get(cppobj.id)
        if declaration and not previous is None:
            cppobj._definition = previous
            return
        if not declaration and not previous is None:
            for ref in previous.references:
                cppobj.references.append(ref)
                ref.reference = cppobj
            previous.references = []
            if isinstance(cppobj, CppFunction):
                previous._definition = cppobj
        self.entities[cppobj.id] = cppobj
        if cppobj.id in self._refs:
            for ref in self._refs[cppobj.id]:
                cppobj.references.append(ref)
                ref.reference = cppobj
            del self._refs[cppobj.id]

    def reference(self, id, ref):
        cppobj = self.entities.get(id)
        if not cppobj is None:
            cppobj.references.append(ref)
            ref.reference = cppobj
        else:
            if not id in self._refs:
                self._refs[id] = []
            self._refs[id].append(ref)
            ref.reference = id


class CppAstParser(object):
    lib_path = None
    includes = "/usr/lib/llvm-3.8/lib/clang/3.8.0/include"
    database = None

    # system required / user optional
    @staticmethod
    def set_library_path(lib_path = "/usr/lib/llvm-3.8/lib"):
        clang.Config.set_library_path(lib_path)
        CppAstParser.lib_path = lib_path

    # required
    @staticmethod
    def set_database(db_path):
        if not CppAstParser.lib_path:
            CppAstParser.set_library_path()
        CppAstParser.database = clang.CompilationDatabase.fromDirectory(db_path)
        CppAstParser.database.db_path = db_path

    # optional
    @staticmethod
    def set_standard_includes(std_includes):
        CppAstParser.STD_INCLUDES = std_includes


    def __init__(self, workspace = ""):
    # public:
        self.workspace      = workspace
        self.global_scope   = CppGlobalScope()
        self.data           = AnalysisData()
    # private:
        self._index         = None
        self._db            = CppAstParser.database

    def parse(self, file_path):
        if self._db is None:
            return self._parse_without_db(file_path)
        return self._parse_from_db(file_path)


    def _parse_from_db(self, file_path):
    # ----- command retrieval -------------------------------------------------
        cmd = self._db.getCompileCommands(file_path) or ()
        if not cmd:
            return None
        for c in cmd:
            with cwd(os.path.join(self._db.db_path, c.directory)):
                args = ["-I" + CppAstParser.includes] + list(c.arguments)[1:]
                if self._index is None:
                    self._index = clang.Index.create()
    # ----- parsing and AST analysis ------------------------------------------
                unit = self._index.parse(None, args)
                self._check_compilation_problems(unit)
                self._ast_analysis(unit.cursor)
        self.global_scope._afterpass()
        return self.global_scope

    def _parse_without_db(self, file_path):
    # ----- command retrieval -------------------------------------------------
        with cwd(os.path.dirname(file_path)):
            args = ["-I" + CppAstParser.includes, file_path]
            if self._index is None:
                self._index = clang.Index.create()
    # ----- parsing and AST analysis ------------------------------------------
            unit = self._index.parse(None, args)
            self._check_compilation_problems(unit)
            self._ast_analysis(unit.cursor)
        self.global_scope._afterpass()
        return self.global_scope

    def _ast_analysis(self, top_cursor):
        assert top_cursor.kind == CK.TRANSLATION_UNIT
        cppobj = self.global_scope
        builders = [CppTopLevelBuilder(c, cppobj, cppobj) \
                    for c in top_cursor.get_children() \
                    if c.location.file \
                        and c.location.file.name.startswith(self.workspace)]
        queue = deque(builders)
        while queue:
            builder = queue.popleft()
            result = builder.build(self.data)
            if result:
                cppobj, builders = result
                if builder.insert_method:
                    builder.insert_method(cppobj)
                else:
                    builder.parent._add(cppobj)
                queue.extend(builders)

    def _check_compilation_problems(self, translation_unit):
        if translation_unit.diagnostics:
            for diagnostic in translation_unit.diagnostics:
                if diagnostic.severity >= clang.Diagnostic.Error:
                    # logging.warning(diagnostic.spelling)
                    print "WARNING", diagnostic.spelling


###############################################################################
# Interface Functions
###############################################################################

def resolve_expression(expression):
    assert isinstance(expression, CppExpression.TYPES)
    if isinstance(expression, CppReference):
        return resolve_reference(expression)
    if isinstance(expression, CppOperator):
        args = []
        for arg in expression.arguments:
            arg = resolve_expression(arg)
            if not isinstance(arg, CppExpression.LITERALS):
                return expression
            args.append(arg)
        if expression.is_binary:
            a = args[0]
            b = args[1]
            if not isinstance(a, CppExpression.LITERALS) \
                    or not isinstance(b, CppExpression.LITERALS):
                return expression
            if expression.name == "+":
                return a + b
            if expression.name == "-":
                return a - b
            if expression.name == "*":
                return a * b
            if expression.name == "/":
                return a / b
            if expression.name == "%":
                return a % b
    # if isinstance(expression, CppExpression.LITERALS):
    # if isinstance(expression, SomeCpp):
    # if isinstance(expression, CppFunctionCall):
    # if isinstance(expression, CppDefaultArgument):
    return expression


def resolve_reference(reference):
    assert isinstance(reference, CppReference)
    if reference.statement is None:
        return None # TODO investigate
    si = reference.statement._si
    if (reference.reference is None
            or isinstance(reference.reference, basestring)):
        return None
    if isinstance(reference.reference, CppVariable):
        var = reference.reference
        value = var.value
        function = reference.function
        for w in var.writes:
            ws = w.statement
            if not w.function is function:
                continue
            if ws._si < si:
                if w.arguments[0].reference is var:
                    value = resolve_expression(w.arguments[1])
                else:
                    continue # TODO
            elif ws._si == si:
                if w.arguments[0] is reference:
                    value = resolve_expression(w.arguments[1])
                else:
                    continue # TODO
        if value is None and var.is_parameter:
            calls = [call for call in function.references \
                          if isinstance(call, CppFunctionCall)]
            if len(calls) != 1:
                return None
            i = function.parameters.index(var)
            if len(calls[0].arguments) <= i:
                return None
            arg = calls[0].arguments[i]
            if isinstance(arg, CppReference):
                return resolve_reference(arg)
            return arg
        return value
    return reference.reference


def is_under_control_flow(cppobj, recursive = False):
    return get_control_depth(cppobj, recursive) > 0

def get_control_depth(cppobj, recursive = False):
    depth = 0
    while not cppobj is None:
        if (isinstance(cppobj, CppBlock)
                and isinstance(cppobj.parent, CppControlFlow)):
            depth += 1
        elif isinstance(cppobj, CppFunction):
            if recursive:
                calls = [get_control_depth(call) for call in cppobj.references
                                if isinstance(call, CppFunctionCall)]
                if calls:
                    depth += max(calls)
            return depth
        cppobj = cppobj.parent
    return depth


###############################################################################
# Helpers
###############################################################################

class cwd:
    """Run a block of code from a specified working directory"""
    def __init__(self, path):
        self.dir = path

    def __enter__(self):
        self.old_dir = os.getcwd()
        os.chdir(self.dir)

    def __exit__(self, exc_type, exc_value, traceback):
        os.chdir(self.old_dir)


def pretty_str(something, indent = 0):
    if isinstance(something, CppEntity):
        return something.pretty_str(indent = indent)
    else:
        return (" " * indent) + repr(something)


if __name__ == "__main__":
    CppAstParser.set_library_path()
    #CppAstParser.set_database("/home/andre/catkin_ws/build")
    parser = CppAstParser(workspace = "/home/andre/")
    parser.parse("/home/andre/cpp/class_example.cpp")
    print parser.global_scope.pretty_str()
    print "\n----------------------------------\n"
    for cppobj in (CppQuery(parser.global_scope).all_references
                   .where_name("a").get()):
        print cppobj.statement.pretty_str()
        print pretty_str(resolve_reference(cppobj))
