
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

# Using radon programmatically:
# http://radon.readthedocs.io/en/latest/api.html
# https://github.com/rubik/radon/tree/master/radon

from radon.visitors import ComplexityVisitor
from radon.metrics import h_visit, mi_compute
from radon.raw import analyze


def file_analysis(iface, scope):
    with open(scope.path, "r") as f:
        code = f.read()
    cc = analyse_cc(iface, code)
    lloc, ratio = analyse_raw_metrics(iface, code)
    h = analyse_halstead_metrics(iface, code)
    mi = mi_compute(h, cc, lloc, ratio)
    iface.report_metric("maintainability_index", mi)
    if mi < 20:
        iface.report_violation("mi_below_20", "MI of " + str(mi))
    if mi < 65:
        iface.report_violation("mi_below_65", "MI of " + str(mi))


def analyse_cc(iface, code):
    visitor = ComplexityVisitor.from_code(code)
    for f in visitor.functions:
        iface.report_metric("cyclomatic_complexity", f.complexity,
                            line = f.lineno,
                            function = f.name,
                            class_ = f.classname)
        if f.complexity > 10:
            iface.report_violation("max_cyclomatic_complexity_10",
                                   "CC of " + str(f.complexity))
        if f.complexity > 15:
            iface.report_violation("max_cyclomatic_complexity_15",
                                   "CC of " + str(f.complexity))
    return visitor.total_complexity

def analyse_raw_metrics(iface, code):
    metrics = analyze(code)
    iface.report_metric("lloc", metrics.lloc)
    iface.report_metric("sloc", metrics.sloc)
    locom = metrics.comments + metrics.multi
    iface.report_metric("comments", locom)
    ploc = metrics.sloc - locom
    iface.report_metric("ploc", ploc)
    if ploc > 400:
        iface.report_violation("max_file_length_400",
                               "File length (PLOC) of " + str(ploc))
    ratio = locom / float(metrics.sloc) if metrics.sloc != 0 else 0
    iface.report_metric("comment_ratio", ratio)
    if ratio < 0.2:
        iface.report_violation("min_comment_ratio_20",
                                "Comment ratio is below 20%.")
    if ratio > 0.3:
        iface.report_violation("max_comment_ratio_30",
                                "Comment ratio is above 30%.")
    if ratio > 0.4:
        iface.report_violation("max_comment_ratio_40",
                                "Comment ratio is above 40%.")
    return (metrics.lloc, ratio * 100)

def analyse_halstead_metrics(iface, code):
    metrics = h_visit(code)
    iface.report_metric("halstead_volume", metrics.volume)
    if metrics.volume > 8000:
        iface.report_violation("halstead_volume_above_8000",
                               "Halstead Volume of " + str(metrics.volume))
    iface.report_metric("halstead_time", metrics.time)
    iface.report_metric("halstead_bugs", metrics.bugs)
    if metrics.bugs > 2:
        iface.report_violation("halstead_bugs_above_2",
                               "Halstead Bugs of " + str(metrics.bugs))
    return metrics.volume
