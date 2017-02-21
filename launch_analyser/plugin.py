import os

def package_analysis(iface, package):
    count = 0
    for sf in package.source_files:
        if sf.language == "launch" and sf.path == os.path.join("launch", sf.name):
            count += 1
    if count:
        print package.name, count