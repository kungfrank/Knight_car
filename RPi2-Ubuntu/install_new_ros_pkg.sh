# Install and build ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm geometry_msgs --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-custom_ros.rosinstall

#
wstool merge -t src indigo-custom_ros.rosinstall
wstool update -t src