#!/usr/bin/env python

try:
   from distutils.core import setup
   from catkin_pkg.python_setup import generate_distutils_setup

   d = generate_distutils_setup(
      packages=['kineverse'],
      package_dir={'': 'src'},
      requires=['numpy',
                'casadi',
                'tqdm',
                'matplotlib',
                'pandas',
                'simplejson',
                'sortedcontainers',]
   )
   setup(**d)
except ModuleNotFoundError:
   from setuptools import setup, find_packages

   setup(
         name='kineverse',
         version='0.0.1',
         description='Framework for building articulated objects from symbolic expressions',
         author='Adrian Röfer',
         author_email='aroefer@cs.uni-freiburg.de',
         url='https://github.com/ARoefer/kineverse.git',
         packages=find_packages(),
         install_requires=[
                   'numpy',
                   'casadi',
                   'tqdm',
                   'matplotlib',
                   'pandas',
                   'simplejson',
                   'sortedcontainers',
                   'qpoases',
                   'urdf_parser_py']
        )

