#!/usr/bin/env python

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE catkin_python_setup in
##   CMakeLists.txt INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['bridge_msgs'],
    package_dir={'': 'src'}
)

setup(**setup_args)
