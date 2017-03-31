###
# standard packages
from distutils.version import LooseVersion
import glob
import os

###
# third-party packages
import cmakegrammar
import cmakeparser


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


def _split_paren_args(text):
    i = text.find("(")
    if i < 0:
        return cmakegrammar.split_args(text)
    args = cmakegrammar.split_args(text[:i])
    expr, k = _read_until_match(text, start = i)
    expr = _split_paren_args(expr)
    rest = _split_paren_args(text[k+1:])
    args.append(expr)
    args.extend(rest)
    while len(args) == 1 and isinstance(args[0], list):
        args = args[0]
    return args


class BuildTarget(object):
    def __init__(self, name, files, is_executable):
        self.name = name
        self.base_name = name
        self.prefix = "" if is_executable else "lib"
        self.suffix = "" if is_executable else ".so"
        self.files = files
        self.links = []

    @property
    def prefixed_name(self):
        return self.prefix + self.base_name

    @property
    def output_name(self):
        return self.prefix + self.base_name + self.suffix

    @classmethod
    def new_target(cls, name, files, directory, is_executable):
        files = [os.path.join(directory, f) for fs in files \
                                            for f in fs.split(";") if f]
        i = 0
        while i < len(files):
            if not os.path.isfile(files[i]):
                replacement = None
                parent = os.path.dirname(files[i])
                prefix = files[i].rsplit(os.sep, 1)[-1]
                for f in os.listdir(parent):
                    joined = os.path.join(parent, f)
                    if f.startswith(prefix) and os.path.isfile(joined):
                        replacement = joined
                        break
                if replacement:
                    files[i] = replacement
                else:
                    del files[i]
                    i -= 1
            i += 1
        return cls(name, files, is_executable)

    def apply_property(self, prop, value):
        if prop == "PREFIX":
            self.prefix = value
        elif prop == "SUFFIX":
            self.suffix = value
        elif prop == "OUTPUT_NAME":
            self.base_name = value
        # TODO elif prop == "<CONFIG>_OUTPUT_NAME":



class CMakeAnalyser(object):
    def __init__(self, environment, finder, srcdir, bindir, variables = None):
        self.resource_finder = finder
        self.variables = variables if not variables is None else {}
        self.environment = environment
        self.data = {
            "project":              None,
            "package_includes":     [],
            "include_dirs":         [],
            "dependencies":         [],
            "system_dependencies":  [],
            "libraries":            {},
            "executables":          {},
            "subdirectories":       []
        }
        self.source_dir = srcdir
        self.binary_dir = bindir
        self.variables["CMAKE_BINARY_DIR"] = bindir

    _CONTROL_FLOW = ("if", "else", "elseif", "foreach", "while")

    def analyse(self, cmakelists, toplevel = True):
        self.directory = os.path.dirname(cmakelists)
        if toplevel:
            self.directory = os.path.abspath(self.directory)
            self.variables["CMAKE_SOURCE_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_SOURCE_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_LIST_FILE"] = cmakelists
        self.variables["CMAKE_CURRENT_LIST_DIR"] = self.directory
        self.variables["CMAKE_CURRENT_BINARY_DIR"] = self.binary_dir + self.directory[len(self.source_dir):]
        parser = cmakeparser.parse_file(cmakelists)
        for stmt in parser.parsetree:
            command = stmt[0].lower()
            args = _split_paren_args(stmt[1]) if stmt[1] else []
            children = stmt[3]
            if command in CMakeAnalyser._CONTROL_FLOW and children:
                self._analyse_control_flow(command, args, children)
            else:
                self._analyse_command(command, args)
        for subdir in self.data["subdirectories"]:
            path = os.path.join(self.directory, subdir, "CMakeLists.txt")
            if os.path.isfile(path):
                analyser = CMakeAnalyser(self.environment, self.resource_finder,
                                         self.source_dir, self.binary_dir,
                                         variables = dict(self.variables))
                analyser.analyse(path, toplevel = False)
                self._merge(analyser)
        if toplevel:
            self._link_targets()

    def _analyse_control_flow(self, command, args, children):
        if command == "else":
            condition = True
        else:
            condition = self._control_arguments(args)
        if condition:
            for stmt in children:
                child_command = stmt[0].lower()
                child_args = _split_paren_args(stmt[1]) if stmt[1] else []
                child_children = stmt[3]
                if child_command in CMakeAnalyser._CONTROL_FLOW and child_children:
                    self._analyse_control_flow(child_command, child_args, child_children)
                else:
                    self._analyse_command(child_command, child_args)

    def _analyse_command(self, command, args):
        args = [self._argument(arg) for arg in args]
        if command == "project" and args[0]:
            self.data["project"] = args[0]
            self.variables["PROJECT_NAME"] = args[0]
        elif command == "include_directories":
            self._process_include_directories(args)
        elif command == "add_subdirectory" and args[0]:
            self.data["subdirectories"].append(args[0])
        elif command == "add_library":
            self._process_library(args)
        elif command == "add_executable":
            self._process_executable(args)
        elif command == "set":
            self._process_set(args)
        elif command == "unset":
            self._process_unset(args)
        elif command == "find_package":
            self._process_find_package(args)
        elif command == "catkin_package":
            self._process_catkin_package(args)
        elif command == "file":
            self._process_file(args)
        elif command == "set_target_properties":
            self._process_set_target_properties(args)
        elif command == "target_link_libraries":
            self._process_link_libraries(args)

    def _process_include_directories(self, args):
        n = len(args)
        assert n >= 1
        i = 0
        before = self.variables.get("CMAKE_INCLUDE_DIRECTORIES_BEFORE") == "ON"
        before = (before or args[0] == "BEFORE") and args[0] != "AFTER"
        if args[0] == "AFTER" or args[0] == "BEFORE":
            i = 1
        while i < n:
            arg = args[i]
            if arg == "SYSTEM":
                i += 1
            elif arg:
                arg = os.path.join(self.directory, arg)
                self.data["include_dirs"].append(arg)
            i += 1

    def _process_library(self, args):
        n = len(args)
        assert n > 1
        name = args[0]
        i = 1
        flags = ("STATIC", "SHARED", "MODULE", "UNKNOWN", "EXCLUDE_FROM_ALL")
        while args[i] in flags:
            i += 1
        if args[i] in ("IMPORTED", "ALIAS", "OBJECT", "INTERFACE"):
            return # TODO
        target = BuildTarget.new_target(name, args[i:], self.directory, False)
        self.data["libraries"][name] = target

    def _process_executable(self, args):
        n = len(args)
        assert n > 1
        name = args[0]
        if args[1] == "IMPORTED" or args[1] == "ALIAS":
            return # TODO
        i = 1
        while args[i] == "WIN32" or args[i] == "MACOSX_BUNDLE" \
                                 or args[i] == "EXCLUDE_FROM_ALL":
            i += 1
        target = BuildTarget.new_target(name, args[i:], self.directory, True)
        self.data["executables"][name] = target

    def _process_set_target_properties(self, args):
        assert len(args) >= 4
        i = args.index("PROPERTIES")
        targets = args[:i]
        k = 0
        while k < len(targets):
            t = targets[k]
            t = self.data["libraries"].get(t, self.data["executables"].get(t))
            if t:
                targets[k] = t
            else:
                del targets[k]
                k -= 1
            k += 1
        i += 1
        while i < len(args):
            prop = args[i]
            value = args[i+1]
            for t in targets:
                t.apply_property(prop, value)
            i += 2

    def _process_link_libraries(self, args):
        assert len(args) >= 2
        t = args[0]
        t = self.data["libraries"].get(t, self.data["executables"].get(t))
        if not t:
            return
        for i in xrange(1, len(args)):
            t.links.append(args[i])

    def _link_targets(self):
        for target in self.data["libraries"].itervalues():
            candidates = target.links
            target.links = []
            for other in candidates:
                d = self.data["libraries"].get(other, self.data["executables"].get(other))
                if d:
                    target.links.append(d)
        for target in self.data["executables"].itervalues():
            candidates = target.links
            target.links = []
            for other in candidates:
                d = self.data["libraries"].get(other, self.data["executables"].get(other))
                if d:
                    target.links.append(d)

    def _process_set(self, args):
        n = len(args)
        assert n > 0
        var = args[0]
        env = var.startswith("ENV{")
        var = var[4:-1] if env else var
        data = self.environment if env else self.variables
        if n == 1:
            if var in data:
                del data[var]
        else:
            i = 1
            while i < n:
                value = args[i]
                if value == "CACHE" or value == "PARENT_SCOPE":
                    break
                i += 1
            value = ";".join(args[1:i])
            data[var] = value

    def _process_unset(self, args):
        assert len(args) > 0
        var = args[0]
        env = var.startswith("ENV{")
        var = var[4:-1] if env else var
        data = self.environment if env else self.variables
        del data[var]

    def _process_find_package(self, args):
        if args[0] != "catkin":
            return
        i = 1
        n = len(args)
        while i < n and args[i] != "COMPONENTS":
            i += 1
        if i < n and args[i] == "COMPONENTS":
            k = i + 1
            while k < n and args[k] != "OPTIONAL_COMPONENTS" \
                        and args[k] != "NO_POLICY_SCOPE":
                k += 1
            args = args[i:k]
            for arg in args:
                if self.resource_finder.find_package(arg):
                    self.variables[arg + "_FOUND"] = "TRUE"
                else:
                    self.variables[arg + "_FOUND"] = "FALSE"
                    self.variables[arg + "-NOTFOUND"] = "TRUE"

    def _process_catkin_package(self, args):
        data = None
        for arg in args:
            if arg == "INCLUDE_DIRS":
                data = self.data["package_includes"]
            elif arg == "CATKIN_DEPENDS":
                data = self.data["dependencies"]
            elif arg == "DEPENDS":
                data = self.data["system_dependencies"]
            elif arg == "LIBRARIES" or arg == "CFG_EXTRAS":
                data = [] # ignored
            else:
                data.append(arg)
        data = self.data["package_includes"]
        i = 0
        n = len(data)
        while i < n:
            data[i] = os.path.join(self.directory, data[i])
            i += 1

    def _process_file(self, args):
        if args[0] == "GLOB":
            var = args[1]
            if args[2] == "RELATIVE":
                args = args[4:]
            else:
                args = args[2:]
            matches = []
            for expr in args:
                expr = self.directory + os.sep + expr
                matches.extend(glob.glob(expr))
            self.variables[var] = ";".join(matches)

    _TRUTH_CONSTANTS = ("1", "ON", "TRUE", "Y", "YES")
    _FALSE_CONSTANTS = ("0", "OFF", "FALSE", "N", "NO", "IGNORE", "NOTFOUND", "")
    _UNARY_OPERATORS = ("DEFINED", "COMMAND", "POLICY", "TARGET", "EXISTS",
                        "IS_DIRECTORY", "IS_SYMLINK", "IS_ABSOLUTE")
    _BINARY_OPERATORS = ("EQUAL", "LESS", "GREATER", "STREQUAL", "STRLESS",
                         "STRGREATER", "MATCHES", "IS_NEWER_THAN",
                         "VERSION_EQUAL", "VERSION_LESS", "VERSION_GREATER")

    def _control_arguments(self, args):
        # Step 0: evaluate ${variables}
        # Step 1: evaluate parentheses
        args = [self._control_arguments(arg) \
                if isinstance(arg, list) \
                else self._argument(arg)
                for arg in args]
        # Step 2: evaluate unary tests
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg in CMakeAnalyser._UNARY_OPERATORS:
                i += 1
                arg = self._unary_test(arg, args[i])
            values.append(arg)
            i += 1
        # Step 3: evaluate binary tests
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg in CMakeAnalyser._BINARY_OPERATORS:
                i += 1
                values.pop()
                arg = self._binary_test(arg, args[i-2], args[i])
            values.append(arg)
            i += 1
        # Step 4: evaluate boolean values
        i = 0
        n = len(values)
        while i < n:
            value = values[i]
            uval = value.upper()
            if uval in CMakeAnalyser._TRUTH_CONSTANTS:
                values[i] = True
            elif uval in CMakeAnalyser._FALSE_CONSTANTS \
                    or uval.endswith("-NOTFOUND"):
                values[i] = False
            elif value != "NOT" and value != "AND" and value != "OR":
                value = self.variables.get(value, "FALSE")
                values[i] = not value in CMakeAnalyser._FALSE_CONSTANTS \
                            and not value.endswith("-NOTFOUND")
            i += 1
        # Step 5: evaluate boolean NOT
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg == "NOT":
                i += 1
                assert isinstance(args[i], bool)
                arg = not args[i]
            values.append(arg)
            i += 1
        # Step 6: evaluate boolean AND and OR
        args = values
        values = []
        i = 0
        n = len(args)
        while i < n:
            arg = args[i]
            if arg == "AND" or arg == "OR":
                i += 1
                values.pop()
                if arg == "AND":
                    arg = args[i-2] and args[i]
                else:
                    arg = args[i-2] or args[i]
            values.append(arg)
            i += 1
        return values[0]

    def _unary_test(self, test, arg):
        if test == "DEFINED":
            return "TRUE" if arg in self.variables else "FALSE"
        if test == "TARGET":
            return "TRUE" if arg in self.data["libraries"] \
                   or arg in self.data["executables"] \
                   else "FALSE"
        if test == "EXISTS":
            return "TRUE" if os.path.exists(args[i]) else "FALSE"
        if test == "IS_DIRECTORY":
            return "TRUE" if os.path.isdir(args[i]) else "FALSE"
        if test == "IS_SYMLINK":
            return "TRUE" if os.path.islink(args[i]) else "FALSE"
        if test == "IS_ABSOLUTE":
            return "TRUE" if os.path.isabs(args[i]) else "FALSE"
        # TODO COMMAND and POLICY
        return "FALSE"

    def _binary_test(self, test, arg1, arg2):
        if not arg1 or not arg2:
            return "FALSE"
        if test == "IS_NEWER_THAN":
            return "TRUE" if not os.path.exists(arg1) \
                   or not os.path.exists(arg2) \
                   or os.path.getmtime(arg1) - os.path.getmtime(arg2) >= 0 \
                   else "FALSE"
        arg1 = self.variables.get(arg1, arg1)
        if test == "MATCHES":
            return "FALSE"  # TODO
        arg2 = self.variables.get(arg2, arg2)
        if test == "STRLESS":
            return "TRUE" if arg1 < arg2 else "FALSE"
        if test == "STRGREATER":
            return "TRUE" if arg1 > arg2 else "FALSE"
        if test == "STREQUAL":
            return "TRUE" if arg1 == arg2 else "FALSE"
        if test == "VERSION_LESS":
            return "TRUE" if LooseVersion(arg1) < LooseVersion(arg2) else "FALSE"
        if test == "VERSION_GREATER":
            return "TRUE" if LooseVersion(arg1) > LooseVersion(arg2) else "FALSE"
        if test == "VERSION_EQUAL":
            return "TRUE" if LooseVersion(arg1) == LooseVersion(arg2) else "FALSE"
        # only the numeric tests remain
        try:
            if test == "LESS":
                return "TRUE" if int(arg1) < int(arg2) else "FALSE"
            if test == "GREATER":
                return "TRUE" if int(arg1) > int(arg2) else "FALSE"
            return "TRUE" if int(arg1) == int(arg2) else "FALSE"
        except ValueError as e:
            return "FALSE"

    def _argument(self, arg):
        i = 0
        n = len(arg)
        while i < n and arg[i] != "$":
            i += 1
        if i == n:
            return arg.replace('"', "")
        prefix = arg[:i]
        suffix = self._maybe_variable(arg[i:])
        return (prefix + suffix).replace('"', "")

    def _maybe_variable(self, arg):
        assert arg[0] == "$"
        is_env_var = arg.startswith("$ENV{")
        if arg.startswith("${") or is_env_var:
            value, k = self._variable(arg, is_env_var)
            suffix = self._argument(arg[k:])
            return value + suffix
        suffix = self._argument(arg[1:])
        return "$" + suffix

    def _variable(self, arg, env):
        var, k = _read_until_match(arg, chars = "{}", start = 4 if env else 1)
        var = self._argument(var)
        if env:
            value = self.environment.get(var, "")
        else:
            value = self.variables.get(var, "")
        return (value, k)

    def _merge(self, other):
        self.data["include_dirs"].extend(other.data["include_dirs"])
        self.data["libraries"].update(other.data["libraries"])
        self.data["executables"].update(other.data["executables"])
