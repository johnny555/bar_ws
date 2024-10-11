#!/bin/bash
source /opt/ros/jazzy/local_setup.bash
source install/setup.bash
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install,ignore:easy_install command is deprecated::setuptools.command.easy_install"; 
export PYTHONWARNINGS
export SDF_PATH=/workspace/install/krytn/share:/workspace/install/krytn/:/opt/ros/jazzy/share/
export GZ_SIM_RESOURCE_PATH=$SDF_PATH
export RCUTILS_COLOURIZE_OUTPUT=1

colcon build --symlink-install --merge-install
