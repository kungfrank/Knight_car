source ~/duckietown/catkin_ws/devel/setup.bash
export robot=oreo
export robot=$HOSTNAME

export ROS_HOSTNAME=$robot.local
export ROS_MASTER_URI=http://$robot.local:11311/
export ROSLAUNCH_SSH_UNKNOWN=1
export DUCKIETOWN_ROOT=$HOME/duckietown
