# rosberrypi_cam

ROS driver for RaspberryPi camera module

## Installation

Make sure ROS is installed on the Raspberry Pi.

Install raspicam:

```
$ cd ~/catkin_ws
$ mkdir external_src
$ cd external_src
$ wget http://sourceforge.net/projects/raspicam/files/raspicam-0.1.3.zip/download
$ mv download raspicam-0.1.3.zip
$ unzip raspicam-0.1.3.zip
$ cd raspicam-0.1.3
$ mkdir build
$ cd build
$ cmake ..
```

Now download and build rosberrypi_cam:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/japonophile/rosberrypi_cam.git
$ cd ~/catkin_ws
$ catkin_make
```

## Usage

Run roscore locally on the Pi or remotely (set the ROS_MASTER_URI accordingly).
Run the rosberrypi_cam_node

```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun rosberrypi_cam rosberrypi_cam_node
```

Copyright (c) 2015, Dan Lazewatsky

README (and small tweaks) added by Antoine Choppin

