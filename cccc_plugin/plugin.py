
#Copyright (c) 2018 Andre Santos
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

import os
import subprocess
import xml.etree.ElementTree as ET


"""
Metrics definitions from CCCC's source code.

if(strcmp(count_tag,"CBO")==0) {
    retval=client_map.size()+supplier_map.size();
} else if(strncmp(count_tag,"FI",2)==0) {
    relationship_map_t::iterator iter;
    iter=supplier_map.begin();
    while(iter!=supplier_map.end()) {
        retval+=(*iter).second->get_count(count_tag);
        iter++;
    }
} else if(strncmp(count_tag,"FO",2)==0) {
    relationship_map_t::iterator iter;
    iter=client_map.begin();
    while(iter!=client_map.end()) {
        retval+=(*iter).second->get_count(count_tag);
        iter++;
    }
} else if(strncmp(count_tag,"IF4",3)==0) {
    char if4_suffix=count_tag[3];
    string fi_variant="FI", fo_variant="FO";
    if(if4_suffix!=0) {
        fi_variant+=if4_suffix;
        fo_variant+=if4_suffix;
    }
    retval=get_count(fi_variant.c_str())*get_count(fo_variant.c_str());
    retval*=retval;
}
"""


def package_analysis(iface, scope):
    outdir = os.path.join(os.getcwd(), scope.name)
    try:
        os.mkdir(outdir)
    except OSError:
        pass # already exists?
    with open(os.devnull, "w") as FNULL:
        subprocess.check_call(["cccc", "--outdir=" + outdir, "--lang=c++"]
                              + list_files(scope), stdout=FNULL, stderr=FNULL)
    with open(os.path.join(outdir, "cccc.xml"), "r") as handle:
        root = ET.parse(handle).getroot()
        summary = root.find("project_summary")
        if not summary is None:
            handle_project_summary(iface, summary)
        summary = root.find("procedural_summary")
        if not summary is None:
            handle_procedural_summary(iface, summary)
        summary = root.find("oo_design")
        if not summary is None:
            handle_oo_design_summary(iface, summary)
        summary = root.find("structural_summary")
        if not summary is None:
            handle_structural_summary(iface, summary)
        summary = root.find("other_extents")
        if not summary is None:
            handle_other_extents_summary(iface, summary)


def handle_project_summary(iface, summary):
    pass

def handle_procedural_summary(iface, summary):
    pass

def handle_oo_design_summary(iface, summary):
    for module in summary.findall("module"):
        report_module_metric("depth_of_inheritance_tree", module, iface)

def handle_structural_summary(iface, summary):
    for module in summary.findall("module"):
        report_module_metric("fan_in", module, iface)
        report_module_metric("fan_out", module, iface, threshold = 1)

def handle_other_extents_summary(iface, summary):
    pass

def list_files(pkg):
    return [f.path for f in pkg.source_files if f.language == "cpp"]

def report_module_metric(metric_id, module, iface, threshold = 0):
    metric_tag = module.find(metric_id)
    value = int(metric_tag.get("value"))
    if value > threshold:
        name = module.find("name").text
        iface.report_metric(metric_id, value, class_ = name)
