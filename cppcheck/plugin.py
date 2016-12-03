
import os
import subprocess
import xml.etree.ElementTree as ET

# TODO: rule aliases
# TODO: add other checks provided by cppcheck.
#       $ cppcheck --errorlist
# TODO: check tool version to support more checks.

RULES = {
    "uninitMemberVar",
    "unusedFunction",
    "redundantAssignment",
    "unreadVariable",
    "variableScope",
    "harosDeprecatedSTL",
    "harosRegisterKeyword",
    "harosThrowSpecification",
    "harosCaseWithoutBreak",
    "harosBooleanCase",
    "harosEnumWithoutBase",
    "harosInlineAssembly",
    "harosDoublePointer",
    "harosTriplePointer",
    "harosSixParams",
    "harosConstUniquePtr",
    "harosDefaultArguments",
    "harosBooleanVector",
    "harosUnion",
    "harosIntegerTypes",
    "harosAssignIncrement",
    "harosAssignAssignment",
    "harosFloatEquality",
    "harosOverrideUnaryAnd",
    "harosOverrideBinaryAnd",
    "harosOverrideBinaryOr",
    "harosOverrideComma",
    "harosUsingErrno",
    "harosRvalueStdArray"
}


def package_analysis(iface, scope):
    FNULL   = open(os.devnull, "w")
    output  = open(scope.id + ".xml", "w")
    try:
        subprocess.call(["cppcheck", "--xml-version=2", "--enable=all",
                            "--rule-file=" + iface.get_file("rules.xml"),
                            scope.path
                        ], stdout=FNULL, stderr=output)
    finally:
        FNULL.close()
        output.close()
    files   = file_mapping(scope)
    xml     = ET.parse(scope.id + ".xml").getroot()
    errors  = xml.find("errors")
    for error in errors:
        handle_report(iface, files, error)



def handle_report(iface, files, error):
    rule_id = error.get("id")
    if rule_id in RULES:
        location = error.find("location")
        file_id = files.get(location.get("file", default = ""))
        if file_id:
            line = int(location.get("line", default = "0"))
            msg = error.get("verbose", default = error.get("msg"))
            iface.report_file_violation(rule_id, msg, file_id, line = line)


def file_mapping(pkg):
    files = {}
        for f in pkg.source_files:
            if f.language == "cpp":
                files[f.get_path()] = f.id
    return files
