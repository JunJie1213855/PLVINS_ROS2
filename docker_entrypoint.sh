#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/ros/plvins_ws/install/setup.bash
exec "$@"
