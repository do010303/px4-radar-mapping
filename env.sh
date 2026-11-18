# Source this before using gz tools, RViz or the scripts against ./run_sitl.sh:
#   source env.sh
source /opt/ros/humble/setup.bash

# run_sitl.sh keeps its Gazebo in a partition of its own on loopback, apart
# from any other simulation on this machine; tools must use the same.
export GZ_PARTITION="${GZ_PARTITION:-px4_mapping}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
