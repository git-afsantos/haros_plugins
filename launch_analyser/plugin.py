###
# standard packages
import csv
import logging
import os

###
# internal packages
from launch_analyser.analyser import LaunchFileAnalyser, Statistics


_log = logging.getLogger(__name__)


###############################################################################
# HAROS plugin
###############################################################################

class InternalState(object):
    def __init__(self):
        self.gstats = Stats("global")
        self.fstats = {}
        self.pkg_depends = {}
        self.unknown_packages = set()

def pre_analysis():
    return InternalState()

def package_analysis(iface, package):
    for sf in package.source_files:
        if sf.language == "launch":
            launch_file = sf.get_path()
            env = dict(os.environ)
            analyser = LaunchFileAnalyser(env, iface)
            merged_launch = analyser.analyse(launch_file)
            if not launch_file in iface.state.fstats:
                stats = analyser.stats
                iface.state.fstats[launch_file] = analyser.stats
                for ref in merged_launch.unknown:
                    if ref[0] == "pkg":
                        iface.state.unknown_packages.add(ref[1])
            for launch in merged_launch.includes:
                iface.state.fstats[launch.name] = None
            if analyser.stats.unknown_pkgs or not merged_launch.valid:
                iface.state.fstats[launch_file] = None
            iface.state.pkg_depends[launch_file] = merged_launch.pkg_depends


def post_analysis(iface):
    print "[LAUNCH] unknown packages", iface.state.unknown_packages
    rows = [Statistics.CSV_HEADERS]
    n = 0
    for lf, stats in iface.state.fstats.iteritems():
        if stats:
            n += 1
            iface.state.gstats.aggregate(stats)
            rows.append(stats.to_csv())
    rows.append(iface.state.gstats.to_csv())
    with open("launch_stats.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for row in rows:
            out.writerow(row)
    iface.export_file("launch_stats.csv")
    with open("launch_pkg_depends.csv", "w") as csvfile:
        out = csv.writer(csvfile)
        for lf, deps in iface.state.pkg_depends.iteritems():
            out.writerow([lf] + list(deps))
    iface.export_file("launch_pkg_depends.csv")
    print "[LAUNCH] considered", n, "launch files"
