
#Copyright (c) 2016 Andre Santos
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#THE SOFTWARE.

from __future__ import unicode_literals

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
    FNULL = open(os.devnull, "w")
    output = open(scope.id + ".xml", "w")
    try:
        subprocess.call(
            ["cppcheck", "--xml-version=2", "--enable=all", "--inconclusive",
             "--verbose", "--rule-file=" + iface.get_file("rules.xml"),
             scope.path],
            stdout=FNULL, stderr=output)
    finally:
        FNULL.close()
        output.close()
    files = file_mapping(scope)
    try:
        xml = ET.parse(scope.id + ".xml").getroot()
        errors = xml.find("errors")
        for error in errors:
            handle_report(iface, scope, files, error)
    except ET.ParseError as e:
        pass



def handle_report(iface, pkg, files, error):
    scope = pkg
    line = None
    rule_id = error.get("id")
    if rule_id.startswith("missingInclude"):
        return # ignored
    if rule_id not in RULES:
        rule_id = "cppcheckRule"
    msg = error.get("verbose", default=error.get("msg"))
    msg = msg.replace(r"\012", "<br>")
    location = error.find("location")
    if location is not None:
        source_file = files.get(location.get("file", default=""))
        if source_file is not None:
            scope = source_file
            line = int(location.get("line", default="0"))
    iface.report_violation(rule_id, msg, scope=scope, line=line)


def file_mapping(pkg):
    files = {}
    for f in pkg.source_files:
        if f.language == "cpp":
            files[f.path] = f
    return files
