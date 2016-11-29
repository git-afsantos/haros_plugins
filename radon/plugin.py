
# Using radon programmatically:
# http://radon.readthedocs.io/en/latest/api.html

from radon.complexity import cc_visit
from radon.metrics import h_visit, mi_compute
from radon.raw import analyze


def file_analysis(iface, scope):
    with open(scope.get_path(), "r") as f:
        code = f.read()
    raw_metrics = analyze(code)
    cc_metrics = cc_visit(code)
    h_metrics = h_visit(code)
    print "CC METRICS"
    print cc_metrics
    print "HALSTEAD METRICS"
    print h_metrics


# def handle_com_ratio(ctx, package_id, file_id, value):
    # if value < 20:
        # ctx.writeNonCompliance(1, package_id, file_id=file_id,
            # comment="Comment ratio is below 20%")
    # if value > 30:
        # ctx.writeNonCompliance(2, package_id, file_id=file_id,
            # comment="Comment ratio is above 30%")
    # if value > 40:
        # ctx.writeNonCompliance(3, package_id, file_id=file_id,
            # comment="Comment ratio is above 40%")

