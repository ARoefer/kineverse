#!/bin/sh

if [ $# -ne 1 ]; then 
    echo "Need directory to clone pybullet and pybind11 to"
    exit 0
fi

cd $1

git clone https://github.com/pybind/pybind11.git
cd pybind11
mkdir build
cd build
if [ -z ${ROS_DISTRO} ] || [ ${ROS_DISTRO} = "noetic" ]; then
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=3 -DPYBIND11_TEST=OFF
else
    cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_PYTHON_VERSION=2 -DPYBIND11_TEST=OFF
fi

sudo make install -j4

cd ../..
git clone https://github.com/ARoefer/bullet3.git

cd bullet3
git checkout kineverse_new

if [ -z ${ROS_DISTRO} ] || [ ${ROS_DISTRO} = "noetic" ]; then
    ./build_cmake_pybullet_3.8_double.sh Release
else
    ./build_cmake_pybullet_2.7_double.sh Release
fi

echo "export PYTHONPATH=${PYTHONPATH}:${PWD}/build_cmake/better_python:${PWD}/build_cmake/examples/pybullet" >> ~/.bashrc
