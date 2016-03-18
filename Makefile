
catkin_ws := catkin_ws
scuderia := scuderia.yaml
machines := $(catkin_ws)/src/duckietown/machines

all: $(machines)

fix-time:
	sudo ntpdate -u us.pool.ntp.org 

$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)
 
catkin-clean:
	rm -rf $(catkin_ws)/build

build-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"
build:
	catkin_make -C $(catkin_ws) 
