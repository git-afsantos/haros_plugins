
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


# >>> print i.__dict__
# {'nloc': 9, 'function_list': [<lizard.FunctionInfo object at 0x10bf7af10>],
# 'filename': '../cpputest/tests/AllTests.cpp'}
# >>> print i.function_list[0].__dict__
# {'cyclomatic_complexity': 1, 'token_count': 22, 'name': 'main', 'parameter_count': 2,
# 'nloc': 3, 'long_name': 'main( int ac , const char ** av )', 'start_line': 30}
