
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

import math

MI_PARAMETERS = ("lloc", "sloc", "ploc", "cyclomatic_complexity", "comment_ratio",
                 "comments", "halstead_volume")

class Maintainability(object):
    def __init__(self, scope):
        self.scope = scope
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
        if not self._has_lloc() or self.lloc <= 0:
            return None
        if not self._has_ratio():
            return None
        if self.cc == 0:
            return None
        if not self.hvol or self.hvol <= 0:
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
        iface.state[datum.scope.id] = Maintainability(datum.scope)
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
            iface.report_metric("maintainability_index", value, scope = mi.scope)
            if value < 20:
                iface.report_violation("mi_below_20", "MI of " + str(value),
                                       scope = mi.scope)
            if value < 65:
                iface.report_violation("mi_below_65", "MI of " + str(value),
                                       scope = mi.scope)
