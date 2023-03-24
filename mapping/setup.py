#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    # scripts=['src/dl_detector/detector.py', 'src/dl_detector/object_classification.py'],
    packages=['grid_map'],
    package_dir={'': 'src'},
)

setup(**setup_args)
