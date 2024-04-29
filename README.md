Kineverse
=========

Kineverse is a framework for building models of articulated objects/robots using symbolic mathematical expressions and constraints. 
It is meant to be used in a ROS environment and features a client-server implementation which allows articulation models to be modified and updated across multiple nodes in a running ROS system.

Version 2
---------
Kineverse has gotten (or is in the progress of getting) a full re-write as [kv-lite](https://github.com/ARoefer/kv_lite.git), as this version made a good first step, but is very difficult to use.
The second version is meant to be more streamlined and tries as much as possible to behave like `numpy` to be easier to approach.
This repository will remain for the sake of posterity. 

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
Kineverse includes a small geometry query system which uses a custom wrapper for the Bullet Physics Engine. Unfortunately, the installation has to be done from source. The `kineverse` package includes a `build_pybullet.sh` script which can be run to perform the entire installation automatically. The script expects a directory as argument to which it will clone both `pybind11` and `bullet3`. It will build and install both from there. The script will ask for `sudo` rights as a part of the process.

```bash
# Ensure you have the Python 3 development tools
sudo apt install python3-dev 

# Run installation
./build_pybullet.sh /my/awesome/library/dir
# Wait...
```

If everything worked, you should be able to import the wrapper in a Python environment. 

```python
import betterpybullet as bpb
```

If that does not work, try restarting your system.
