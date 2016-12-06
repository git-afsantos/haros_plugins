
import math

MI_PARAMETERS = ("lloc", "sloc", "ploc", "cyclomatic_complexity", "comment_ratio",
                 "comments", "halstead_volume")

class Maintainability(object):
    def __init__(self):
        self.mi = None
        self.lloc = None
        self.sloc = None
        self.ploc = None
        self.ratio = None
        self.comments = None
        self.cc = 0
        self.hvol = None

    def compute(self):
        self._prepare()
        if not self._has_lloc():
            return None
        if not self._has_ratio():
            return None
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

    def _prepare(self):
        if self.sloc is None:
            if not self.comments is None:
                if not self.ploc is None:
                    self.sloc = self.comments + self.ploc
                elif not self.lloc is None:
                    self.sloc = self.comments + self.lloc

    def _has_lloc(self):
        if not self.lloc is None:
            return True
        if not self.ploc is None:
            self.lloc = self.ploc
            return True
        if not self.sloc is None:
            if not self.comments is None:
                self.lloc = self.sloc - self.comments
                return True
            if not self.ratio is None:
                self.lloc = self.sloc - int(self.sloc * self.ratio)
                return True
        return False

    def _has_ratio(self):
        if not self.ratio is None:
            return True
        if self.comments is None:
            return False
        if self.sloc is None:
            return False
        self.ratio = self.comments / float(self.sloc) if self.sloc else 0
        return True

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
    elif metric == "ploc":
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
