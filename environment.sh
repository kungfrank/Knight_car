echo "Activating ROS"
source /opt/ros/indigo/setup.bash
echo "Setting up PYTHONPATH"
export PYTHONPATH=/home/ubuntu/duckietown/catkin_ws/src:$PYTHONPATH
echo "Activating development workspace"
source ~/duckietown/catkin_ws/devel/setup.bash
echo "setting ROS_MASTER_URI"
export ROS_MASTER_URI=http://`hostname`.local:11311
echo ROS_MASTER_URI=$ROS_MASTER_URI

export ROS_HOSTNAME=`hostname`.local
echo ROS_HOSTNAME=$ROS_HOSTNAME