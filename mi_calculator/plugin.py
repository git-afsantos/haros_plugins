
import math

MI_PARAMETERS = ("lloc", "sloc", "cyclomatic_complexity", "comment_ratio",
                 "comments", "halstead_volume")

class Maintainability(object):
    def __init__(self):
        self.mi = None
        self.lloc = None
        self.sloc = None
        self.ratio = None
        self.comments = None
        self.cc = 0
        self.hvol = None

    def compute(self):
        if self.lloc is None:
            if self.sloc is None:
                return None
            self.lloc = self.sloc
        if self.ratio is None:
            if self.comments is None:
                return None
            self.ratio = self.comments / float(self.sloc) if self.sloc else 0
        if self.cc == 0:
            return None
        if self.hvol is None:
            return None
        sloc_scale = math.log(self.lloc)
        volume_scale = math.log(self.hvol)
        comments_scale = math.sqrt(2.46 * math.radians(self.ratio * 100))
        nn_mi = (171 - 5.2 * volume_scale - .23 * self.cc - 16.2 \
                 * sloc_scale + 50 * math.sin(comments_scale))
        return min(max(0., nn_mi * 100 / 171.), 100.)

def pre_process():
    return {}

def process_file_metric(iface, datum):
    metric = datum.metric.id
    if not metric in MI_PARAMETERS and metric != "maintainability_index":
        return
    if not datum.scope.id in iface.state:
        iface.state[datum.scope.id] = Maintainability()
    mi = iface.state[datum.scope.id]
    if metric == "lloc":
        mi.lloc = datum.value
    elif metric == "sloc":
        mi.sloc = datum.value
    elif metric == "comments":
        mi.comments = datum.value
    elif metric == "comment_ratio":
        mi.ratio = datum.value
    elif metric == "cyclomatic_complexity":
        mi.cc += datum.value
    elif metric == "halstead_volume":
        mi.hvol = datum.value
    elif metric == "maintainability_index":
        mi.mi = datum.value

def post_process(iface):
    for id, mi in iface.state.iteritems():
        if not mi.mi is None:
            continue
        value = mi.compute()
        if not value is None:
            iface.report_file_metric("maintainability_index", value, id)
