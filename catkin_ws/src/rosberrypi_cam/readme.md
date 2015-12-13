# `rosberrypi_cam` package

## Dependencies

### catkin 
* `roscpp`
* `image_transport`
* `cv_bridge`
* `camera_info_manager`

### other
* raspicam: see `catkin_ws/external_src/raspicam-0.1.3` for further details

# Node: `rosberrypi_cam_node`
This node captures frames from the PiCamera and publishes as a `sensor_msgs/Image`

## Parameters
* `fps`: FPS as int. default 30.
* `height`: image hight in pixels. Default 200
* `width`:  image width in pixels. Default 320
* `brightness`: [0,100] . Default 50.
* `contrast`: [0,100] . Default 50.
* `saturation`: [0,100] . Default 50.
* `gain`: [0,100] . Default 50.
* `exposure`: [0,100]. -1 for auto. Default 50.
* `white_balance_red_v`: [0,100], -1 for auto. Default 50.
* `white_balance_blue_u`: [0,100], -1 for auto. Default 50.

## Subscribe Topics
None

## Publish Topics
* `~camera_info`: `sensor_msgs/CameraInfo`. For calibraiton data.
* `~image_raw`: `sensor_msgs/Image`. Camera captured image.

## Services
None

