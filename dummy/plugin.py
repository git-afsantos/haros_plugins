print "[dummy] loading plugin"

import dummy.dummysrc as dummysrc

dummysrc.dummy_function()

def file_analysis(iface, scope):
    print "[dummy] analysing file", scope.id
    iface.report_violation("roscpp_4.3", "Dummy violation for " + scope.id, line = 1)
