echo "Activating ROS"
source /opt/ros/indigo/setup.bash
echo "Setting up PYTHONPATH"
export PYTHONPATH=/home/ubuntu/duckietown/catkin_ws/src:$PYTHONPATH
echo "Activating development"
source ~/duckietown/catkin_ws/devel/setup.bash