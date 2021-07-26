#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['kineverse'],
   package_dir={'': 'src'},
   requires=['numpy', 
             'casadi', 
             'simplejson',
             'tqdm',
             'sortedcontainers',
             'pandas',
             'matplotlib',
             'urdf_parser_py']
)

setup(**d)