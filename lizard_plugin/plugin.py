
# https://github.com/terryyin/lizard

import lizard

def file_analysis(iface, scope):
	metrics = lizard.analyze_file(scope.get_path())
	iface.report_metric("ploc", metrics.nloc)
	for fun in metrics.function_list:
		iface.report_metric("cyclomatic_complexity", fun.cyclomatic_complexity,
							line = fun.start_line, function = fun.name)
		iface.report_metric("function_parameters", fun.parameter_count,
							line = fun.start_line, function = fun.name)
		iface.report_metric("ploc", fun.nloc,
							line = fun.start_line, function = fun.name)
