
# Using radon programmatically:
# http://radon.readthedocs.io/en/latest/api.html
# https://github.com/rubik/radon/tree/master/radon

from radon.visitors import ComplexityVisitor
from radon.metrics import h_visit, mi_compute
from radon.raw import analyze


def file_analysis(iface, scope):
    with open(scope.get_path(), "r") as f:
        code = f.read()
    cc = analyse_cc(iface, code)
    lloc, ratio = analyse_raw_metrics(iface, code)
    h = analyse_halstead_metrics(iface, code)
    mi = mi_compute(h, cc, lloc, ratio)
    iface.report_metric("maintainability_index", mi)


def analyse_cc(iface, code):
    visitor = ComplexityVisitor.from_code(code)
    for f in visitor.functions:
        iface.report_metric("cyclomatic_complexity", f.complexity,
                            line = f.lineno,
                            function = f.name,
                            class_ = f.classname)
    return visitor.total_complexity

def analyse_raw_metrics(iface, code):
    metrics = analyze(code)
    iface.report_metric("lloc", metrics.lloc)
    iface.report_metric("sloc", metrics.sloc)
    locom = metrics.comments + metrics.multi
    iface.report_metric("comments", locom)
    ploc = metrics.sloc - locom
    iface.report_metric("ploc", ploc)
    if ploc > 40:
        iface.report_violation("max_function_length_40",
                               "Function length of " + str(ploc))
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
    iface.report_metric("halstead_time", metrics.time)
    iface.report_metric("halstead_bugs", metrics.bugs)
    return metrics.volume
