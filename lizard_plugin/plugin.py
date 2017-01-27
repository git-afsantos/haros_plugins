
# https://github.com/terryyin/lizard

import lizard

def file_analysis(iface, scope):
	metrics = lizard.analyze_file(scope.get_path())
	iface.report_metric("ploc", metrics.nloc)
	for fun in metrics.function_list:
        cc = fun.cyclomatic_complexity
        params = fun.parameter_count
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
		iface.report_metric("ploc", fun.nloc,
							line = fun.start_line, function = fun.name)
