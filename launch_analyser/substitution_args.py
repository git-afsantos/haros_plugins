

class SubstitutionError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

###############################################################################
# Substitution Parser
###############################################################################

def parse(text, scope, default = None):
    if default is None:
        return _parse(text, scope)
    try:
        return _parse(text, scope)
    except SubstitutionError as e:
        return default


def _parse(text, scope):
    i = 0
    n = len(text)
    while i < n and text[i] != "$":
        i += 1
    if i == n:
        return text
    prefix = text[:i]
    suffix = _maybe_substitution(text[i:], scope)
    return prefix + suffix


def _maybe_substitution(text, scope):
    assert text[0] == "$"
    if text.startswith("$(") and ")" in text:
        value, k = _substitution(text, scope)
    else:
        value = "$"
        k = 1
    suffix = _parse(text[k:], scope)
    return value + suffix

def _substitution(text, scope):
    expr, k = _read_until_match(text, start = 1)
    expr = expr.split(None, 1)
    cmd = expr[0]
    expr = expr[1]
    if cmd == "arg":
        value = scope.get_arg(expr)
        if value is None:
            raise SubstitutionError(text[:k])
    elif cmd == "find":
        value = scope.get_pkg(expr)
        if value is None:
            raise SubstitutionError(text[:k])
    elif cmd == "env":
        value = scope.get_env(expr)
        if value is None:
            raise SubstitutionError(text[:k])
    elif cmd == "optenv":
        expr = expr.split(None, 1)
        name = expr[0]
        expr = expr[1] if len(expr) > 1 else ""
        value = scope.get_env(name, expr)
    elif cmd == "anon":
        value = scope.get_anon(expr)
    elif cmd == "eval":
        value = "" # TODO
    else:
        raise SubstitutionError(text[:k])
    return (value, k)


def _read_until_match(text, chars = "()", start = 0):
    left = chars[0]
    right = chars[1]
    assert text[start] == left
    i = start + 1
    matches = 1
    while matches > 0:
        if text[i] == left:
            matches += 1
        elif text[i] == right:
            matches -= 1
        i += 1
    return (text[start+1:i-1], i)


###############################################################################
# Backup, in case it is needed
###############################################################################

class LaunchAttribute(object):
    def __init__(self, text, scope):
        self.unknown = []                       # (type, value) references
        self.value = self._parse(text, scope)   # text after substitution

    def _parse(self, text, scope):
        i = 0
        n = len(text)
        while i < n and text[i] != "$":
            i += 1
        if i == n:
            return text
        prefix = text[:i]
        suffix = self._maybe_substitution(text[i:], scope)
        return prefix + suffix

    def _maybe_substitution(self, text, scope):
        assert text[0] == "$"
        if text.startswith("$(") and ")" in text:
            value, k = self._substitution(text, scope)
        else:
            value = "$"
            k = 1
        suffix = self._parse(text[k:], scope)
        return value + suffix

    def _substitution(self, text, scope):
        expr, k = _read_until_match(text, start = 1)
        expr = expr.split(None, 1)
        cmd = expr[0]
        expr = expr[1]
        if cmd == "arg":
            value = scope.get_arg(expr)
            if value is None:
                self.unknown.append(("arg", expr))
                value = text[:k]
        elif cmd == "find":
            value = scope.get_pkg(expr)
            if value is None:
                self.unknown.append(("pkg", expr))
                value = text[:k]
        elif cmd == "env":
            value = scope.get_env(expr)
            if value is None:
                self.unknown.append(("env", expr))
                value = text[:k]
        elif cmd == "optenv":
            expr = expr.split(None, 1)
            name = expr[0]
            expr = expr[1] if len(expr) > 1 else ""
            value = scope.get_env(name, expr)
        elif cmd == "anon":
            value = scope.get_anon(expr)
        elif cmd == "eval":
            value = "" # TODO
        else:
            raise InvalidAttributeError("$(" + cmd + " " + expr + ")")
        return (value, k)

    def __str__(self):
        return self.value
