import os
from setuptools import setup, find_packages

# Utility function to read the README file.
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name            = "haros_plugins",
    version         = "1.0.0",
    author          = "Andre Santos",
    author_email    = "andre.f.santos@inesctec.pt",
    description     = "Plugin repository for HAROS.",
    long_description = read("README.md"),
    long_description_content_type="text/markdown",
    license         = "MIT",
    keywords        = "static-analysis ros",
    url             = "https://github.com/git-afsantos/haros_plugins",
    packages        = find_packages(),
    install_requires = [
        "lizard",
        "radon",
        "pylint"
    ],
    zip_safe        = True
)
