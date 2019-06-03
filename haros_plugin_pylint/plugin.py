
#Copyright (c) 2017 Andre Santos
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

import subprocess

# TODO add a way to suppress certain message types, maybe a config file

def file_analysis(iface, scope):
    cmd = ["pylint", "--persistent=n", "--score=n",
           '--msg-template="{line}:{obj}:({msg_id}) {msg}"',
           scope.path]
    # it is possible that we need cwd=parent_path, env=_get_env()
    # see https://github.com/PyCQA/pylint/blob/master/pylint/epylint.py
    process = subprocess.Popen(cmd, stdout = subprocess.PIPE,
                               stderr = subprocess.STDOUT,
                               universal_newlines=True)
    for report in process.stdout:
        parts = report.split(":", 2)
        if not len(parts) == 3:
            continue
        if not parts[0].isdigit():
            continue
        line = int(parts[0])
        obj_parts = parts[1].split(".", 1)
        fun = obj_parts[-1] or None
        cls = obj_parts[0] if len(obj_parts) > 1 else None
        msg = parts[2]
        report_issue(iface, line, fun, cls, msg)
    process.wait()

def report_issue(iface, line, fun, cls, msg):
    msg_type = msg[1]
    if msg_type == "R":
        rule_id = "refactor"
    elif msg_type == "C":
        rule_id = "convention"
    elif msg_type == "W":
        rule_id = "warning"
    elif msg_type == "E":
        rule_id = "error"
    else:
        assert msg_type == "F"
        rule_id = "fatal"
    iface.report_violation(rule_id, msg, line = line,
                           function = fun, class_ = cls)
