Kineverse
=========

Kineverse is a framework for building articulation models using symbolic mathematical expressions and constraints. 
It is meant to be used in a ROS environment and features a client-server implementation which allows articulation models to be modified and updated across multiple nodes in a running ROS system.


Installation
------------
Kineverse depends on a number of different libraries, some of which need to be installed manually. The installation has been tested for Ubuntu 16.04/18.04 and ROS versions Kinetic/Melodic.

### Kineverse
Go to a ROS workspace and clone Kineverse.

```bash
cd ros_ws/src  # Go to a ROS workspace

git clone https://github.com/ARoefer/kineverse.git
```

### Installing Symengine
Symengine is a symbolic math framework that forms the backbone of Kineverse. There are two ways of installing Symengine: from `pip`, or from source. I would suggest installing it from source, since the pip version lacks the LLVM feature of Symengine. While that feature is not strictly required for Kineverse, there are functions which do rely on it.

```bash
# OPTION 1: Pip install
pip install symengine

# OPTION 2: Source install
sudo apt install llvm-6.0-dev  # Install LLVM dev-kit

# GMP is needed to build Symengine
sudo apt-get install cmake libgmp-dev

# Clone Symengine
cd libs # Go to a directory where you want to store the symengine code
git clone https://github.com/symengine/symengine.git
git clone https://github.com/symengine/symengine.py.git
cd symengine
git checkout `cat ../symengine.py/symengine_version.txt`
cmake -DWITH_LLVM:BOOL=ON .
make -j8
sudo make install
cd ../symengine.py
sudo python setup.py install
```

You should be able to fire up your favorite Python interpreter and do:

```python
import symengine as se
```
If this does not work, try restarting your machine.

### QP-Oases
Kineverse contains a small, QP-based motion generation module. This module requires the QP-Oases ROS package to be installed. 

```bash
cd ros_ws/src  # Go to a ROS workspace
git clone https://github.com/SemRoCo/qpOASES.git
```

Once you are done cloning the package, you can rebuild and re-source your workspace. 

### Python Packages
Kineverse depends on a number of Python pacakges that can be installed from pip.

```bash
pip install numpy pandas matplotlib tqdm sortedcontainers
```


### Installing Bullet
Kineverse includes a small geometry query system which uses a custom wrapper for the Bullet Physics Engine. Unfortunately the installation is quite manual and requires a modification of the `PYTHONPATH` variable.

```bash
cd libs  # Go to a directory where you want to store the bullet code
git clone https://github.com/ARoefer/bullet3.git
cd bullet3
./build_cmake_pybullet_2.7_double.sh

export PYTHONPATH=$PYTHONPATH:$PWD/build_cmake/better_python:$PWD/examples/pybullet  # Extend the python path
```

If you do not want to keep adding the library to your path manually, I advise adding the export of `PYTHONPATH` to your `.bashrc` file.

If everything worked, you should be able to import the wrapper in a Python environment. 

```python
import betterpybullet as bpb
```

If that does not work, try restarting your system.
