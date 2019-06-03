
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

# https://github.com/terryyin/lizard

import lizard

def file_analysis(iface, scope):
    metrics = lizard.analyze_file(scope.path)
    iface.report_metric("ploc", metrics.nloc)
    if metrics.nloc > 400:
        iface.report_violation("max_file_length_400",
                               "File length (PLOC) of " + str(metrics.nloc))
    for fun in metrics.function_list:
        cc = fun.cyclomatic_complexity
        params = fun.parameter_count
        ploc = fun.nloc
        iface.report_metric("cyclomatic_complexity", cc,
                            line = fun.start_line, function = fun.name)
        if cc > 10:
            iface.report_violation("max_cyclomatic_complexity_10",
                                   "function with cyclomatic complexity " + str(cc),
                                   line = fun.start_line, function = fun.name)
        if cc > 15:
            iface.report_violation("max_cyclomatic_complexity_15",
                                   "function with cyclomatic complexity " + str(cc),
                                   line = fun.start_line, function = fun.name)

        iface.report_metric("function_parameters", params,
                            line = fun.start_line, function = fun.name)
        if params > 6:
            iface.report_violation("max_function_parameters_6",
                                   "function with " + str(params) + " parameters",
                                   line = fun.start_line, function = fun.name)

        iface.report_metric("ploc", ploc,
                            line = fun.start_line, function = fun.name)
        if ploc > 40:
            iface.report_violation("max_function_length_40",
                                   "Function length of " + str(ploc),
                                   line = fun.start_line, function = fun.name)
