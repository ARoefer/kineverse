Getting Started
===============

Hello and welcome to the world of Kineverse! This multi-step guide will get you up and running with the kineverse library.

Installation
------------

Before we can dive in, you will need a couple of libraries to get started. It is assumed that you are familiar with ROS and catkin. 
Kineverse depends five libraries

 - GiskardPy: `https://github.com/SemRoCo/giskardpy`
 - QPOases: `https://github.com/SemRoCo/qpOASES.git`
 - Symengine: `https://github.com/symengine/symengine`
 - SymenginePy: `https://github.com/symengine/symengine.py`
 - Bullet: `https://github.com/ARoefer/bullet3`


The Basics
----------

Kineverse is a library for modelling and exchanging articulation models. Its main focus are the areas of robotic perception and motion generation and gradient based motion generation methods in particular. 
As such, Kineverse comes equipped with a small math library for spatial transformations and gradients. Everything else in Kineverse is built upon these libraries making them a good place to start.

.. figure:: /images/Kineverse_pkgs.png
    :alt: Kineverse package dependencies
    :align: center

    Dependency structure of Kineverse's packages

