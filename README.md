Kineverse
=========

Kineverse is a framework for building models of articulated objects/robots using symbolic mathematical expressions and constraints. 
It is meant to be used in a ROS environment and features a client-server implementation which allows articulation models to be modified and updated across multiple nodes in a running ROS system.


Installation
------------
Kineverse depends on a number of different libraries, some of which need to be installed manually. The installation has been tested for Ubuntu 20.04 and ROS Noetic.

### Kineverse
Go to a ROS workspace and clone Kineverse.

```bash
cd ros_ws/src  # Go to a ROS workspace

git clone https://github.com/ARoefer/kineverse.git
git clone https://github.com/ARoefer/kineverse_msgs.git
```

### Dependencies
Kineverse depends on a number of external packages, some of which can be installed using pip. From within the `kineverse` directory, run:

```bash
pip install -r requirements.txt
```


### QP-Oases
For the time being, Kineverse contains a small, QP-based motion generation module. This module requires the QP-Oases ROS package to be installed. 

```bash
cd ros_ws/src  # Go to a ROS workspace
git clone https://github.com/SemRoCo/qpOASES.git
cd qpOASES
git checkout noetic
```

Once you are done cloning the package, you can rebuild and re-source your workspace. 

### Installing Bullet
Kineverse includes a small geometry query system which uses a custom wrapper for the Bullet Physics Engine. Unfortunately the installation is quite manual and requires a modification of the `PYTHONPATH` variable.

```bash
sudo apt install python-dev 
pip install 'pytest<5.0'

cd libs  # Go to a directory where you want to store the bullet code
git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python2.7
make install -j4

git clone https://github.com/ARoefer/bullet3.git
cd bullet3
./build_cmake_pybullet_3.8_double.sh

export PYTHONPATH=$PYTHONPATH:$PWD/build_cmake/better_python:$PWD/examples/pybullet  # Extend the python path
```

If you do not want to keep adding the library to your path manually, I advise adding the export of `PYTHONPATH` to your `.bashrc` file.

If everything worked, you should be able to import the wrapper in a Python environment. 

```python
import betterpybullet as bpb
```

If that does not work, try restarting your system.
