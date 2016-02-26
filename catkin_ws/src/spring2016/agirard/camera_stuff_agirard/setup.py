# -*- coding: utf-8 -*-
"""
Created on Fri Feb 26 10:51:26 2016

@author: alex
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['camera_stuff_agirard'],
	package_dir={'': 'include'},
)
setup(**setup_args)