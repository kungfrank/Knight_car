
catkin_ws := catkin_ws
scuderia := scuderia.yaml
machines := $(catkin_ws)/src/duckietown/machines

all: $(machines)


$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)

fix-time:
	echo "Calling ntpdate to fix time"
	sudo ntpdate -u us.pool.ntp.org 

catkin-clean:
	rm -rf $(catkin_ws)/build

build-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"
	
build:
	catkin_make -C $(catkin_ws) 

# Unit tests
# Teddy: make it so "make unittests" runs all unit tests

unittests-environment:
	bash -c "source environment.sh; python setup/sanity_checks"

unittests:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"


unittests-anti_instagram:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; rosrun anti_instagram annotation_tests.py"

# HW testing 

test-camera:
	echo "Testing Camera HW by taking a picture (smile!)."
	raspistill -t 1000 -o test-camera.jpg


test-led: 
	echo "Calibration blinking pattern"
	bash -c "source environment.sh; rosrun rgb_led blink test_all_1"



# Basic demos

vehicle_name=$(shell hostname)

demo-joystick: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown joystick.launch veh:=$(vehicle_name)"

demo-joystick-camera: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name)"

demo-line_detector: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name)"

demo-joystick-perception: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos master.launch fsm_file_name:=joystick"
  

demo-led-fancy1: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy1"

demo-led-fancy2: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy2"

demo-led-blink-%: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led blink $*"

