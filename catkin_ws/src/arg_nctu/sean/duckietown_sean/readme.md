Wheel odometry

To run this node, make sure you have done the following:

1. Download the python RPi.GPIO module on raspberry pi. Some references available on Google.

2. In my case, my rpi2 with model B and motors have two-phase output encoders, I stuck the state 1 output of the left wheel to rpi 23 GPIO in BCM and right to 27. State 2 of left to 24 and right to 27, respectively. (Make sure your rpi model, different model may have different BCM pin numbers.) Feel free to change the pin to suit your case while remember to change the code.

3. After setup, run command line: 
   $ sudo su
   $ source /opt/ros/indigo/setup.bash
   $ source ~/duckietown/catkin_ws/devel/setup.sh
   $ roslaunch duckietown_sean joystick_with_wheel_odom.launch
   
PS. I noticed that in dagu_car/launch/wheels_driver_node.launch were missed the local argument which caused above .launch error (Try to use SSH to connect to itself with root which I cannot solve, so I revive the .launch file.)

