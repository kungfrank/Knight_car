# Cone_detector
ROS + OpenCV 2.4.8 integration.  Examples of object detection using  template matching, countour finding, and morphology with rospy and roscpp


### Examples
Video of using stuff


### Verify Installation Requirements 
To use this software you need ros-indigo, OpenCV 2.4.8, and python. 

* Verify OpenCV release files exist for 2.4.8:
```
cat /usr/share/OpenCV/OpenCVModules-release.cmake | grep 2.4.8
```
You should see a lot of printouts like this:
```
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8"
  ...
```

*  Check the OpenCV2 python bindings exist:
```
 >>> import cv2

```
* Compile the package. Move the code directory into your `catkin_ws/src` and call `catkin_make` 

    *  relevant ros packages: roscpp, rospy, std_msgs, image_transport, cv_bridge, sensor_msgs, geometry_msgs


### Overview of files in Obj_Detector Repository:

* package.xml
* CMakeLists.txt
* example.bag
* launch/
    * cone_detector.launch
    * ...
* src/

    * echo.py
    * echo.cpp
    * image_matching.py
    * morphology_object_tracking.cpp
    * contour.py
    * blob.png
    * image.png



---
#### example.bag
A very tiny bag file that shows an orange cone.  you can test all of the code with this bag file and the .png images in the src directory.
```
rosbag play example.bag -a
```

#### CMakeLists

`CMakeLists.txt` finds and links the OpenCV libraries and ros packages.  If you create more .cpp files you need to declare a c++ executable and add dependencies to it. 

Add the bottom of CMakeLists.txt we added the following to make `echo`:
```
add_executable(echo src/echo.cpp)
add_dependencies(echo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo ${OpenCV_LIBS} ${catkin_LIBRARIES}  )
```

#### Launch/cone_detector.launch
This package creates ros subscribers that subscribe to `~image_raw` to obtain images from a camera feed.  The launch file shows how to remap `~image_raw` to `/camera/rgb/image_rect_color`.  Modify this file to subscribe to a different image feed.


---
### Source Files Explanations

#### echo.py and echo.cpp  (CV_BRIDGE Examples)
##### Usage:
```
roslaunch cone_detector echo_py.launch 
roslaunch cone_detector echo_cpp.launch 

```
Both echo programs subscribe to our image topic and publish the image they recieved.  Before publishing the image message they convert the message to opencv type using cv_bridge.

In python:
```python
#convert sensor_msgs.msg/Image.msg  to opencv
self.bridge.imgmsg_to_cv2(image_msg)

#convert opencv image to sensor_msgs.msg/Image.msg:
self.bridge.cv2_to_imgmsg(image_cv, "bgr8")
```

In cpp:
```
// convert sensor_msgs.msg/Image.msg to opencv Mat
image = cv_bridge::toCvCopy(msg, "bgr8")->image;

// convert Mat to sensor_mgs.msg/Image.msg
 msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", image).toImageMsg();

```

##### Visualization:
You can use rqt to visualize the echo image stream. 
``` 
rqt_image_view
```
Select the topic `/echo_image` or `/echo/echo_image` for cpp and python, respectively.


---
#### image_matching.py (Template Matching Example in Python)
##### Usage:
This program uses a template image and tries to find it inside the camera feed.
To run this program, you must include the template image location:

```
roslaunch cone_detector cone_detector.launch image:=/home/ari/catkin_ws/src/cone/cone_detector/src/blob.png 
```

This program defines two classes: `ConeDetector` and `TemplateMatcher`.

* ConeDetector should look very similiar to Echo.py. It has an additional ros publisher for the `cone_ibvs` topic.  This publishes a signal from [-1,1] of the cones relative location and was used for the TA's solution of IBVS parking at a cone.

* TemplateMatcher uses OpenCV's `matchTemplate()` function to compare the template image to the raw image frame.   You can read about it more on their website here: http://docs.opencv.org/master/d4/dc6/tutorial_py_template_matching.html#gsc.tab=0

As you'll see many vision algorithms have different qualities.  Template matching is very susceptible to scale and rotation error.  An upright orange cone is symmetrical and will not have rotation error, however the scale will change as the robot moves toward and away the cone. We have implemented pyramid matching functionality that resizes the template image for comparison.

##### Visualization:
You can use rqt to visualize the detected cone.
```
rqt_image_view
```
Select the topic `/cone_detector/cone_detection`.


---
#### morphology_object_tracking.cpp  (color segementation in cpp)

##### Usage:
```
roslaunch cone_detector morphology_object_tracking.launch 
```


This code was adapted from here: http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html .

This code will track the cone and update its location over time.  Three opencv windows are created: control, original, and thresholded image.  In control you can change the hue, value, and saturation thresholds we are using to detect the orange cone.  The original window will display the original image with a line tracing its location over time. Lastly, threshold should display a mostly black window with the orange cone in white. If you cannot see the cone you can modify the threshold values.
##### Visualization:
Running the code should automatically display three cv windows.

---
#### contour.py (Contour image detection in python)

##### Usage:
```
python contour.py image.png
```

countour.py is a simple example that does not integrate with ROS.  It simply shows the opencv functionality for finding contours. It detects the contours in the image that are with in the color min and max thresholds:
```
COLOR_MIN = np.array([0, 80, 80],np.uint8)
COLOR_MAX = np.array([22, 255, 255],np.uint8)
```

Note that the image was first converted into HSV colorspace. The color orange has a hue of around 0 - 22, which is why the thresholds was set this way.

This program plots a box around the largest contour within our color specified thresholds.


##### Visualization:
Running the code should automatically display one cv window. 
