import os
from setuptools import setup, find_packages

# Utility function to read the README file.
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name            = "haros_plugins",
    version         = "1.0.3",
    author          = "Andre Santos",
    author_email    = "andre.f.santos@inesctec.pt",
    description     = "Plugin repository for HAROS.",
    long_description = read("README.md"),
    long_description_content_type="text/markdown",
    license         = "MIT",
    keywords        = "static-analysis ros",
    url             = "https://github.com/git-afsantos/haros_plugins",
    packages        = find_packages(),
    package_data    = {
        "haros_plugin_cccc": ["plugin.yaml"],
        "haros_plugin_ccd": ["plugin.yaml"],
        "haros_plugin_cppcheck": ["plugin.yaml", "rules.xml"],
        "haros_plugin_cpplint": ["plugin.yaml"],
        "haros_plugin_lizard": ["plugin.yaml"],
        "haros_plugin_mi_calculator": ["plugin.yaml"],
        "haros_plugin_pylint": ["plugin.yaml"],
        "haros_plugin_radon": ["plugin.yaml"],
    },
    install_requires = [
        "lizard",
        "radon",
        "pylint"
    ],
    zip_safe        = True
)
