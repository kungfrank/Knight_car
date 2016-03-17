
catkin_ws := catkin_ws
scuderia := scuderia.yaml
machines := $(catkin_ws)/src/duckietown/machines

all: $(machines)


$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)
 
catkin-clean:
	rm -rf $(catkin_ws)/build

build:
	catkin_make -C $(catkin_ws) --make-args "-j4"
