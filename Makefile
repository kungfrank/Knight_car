
scuderia := scuderia.yaml
machines := catkin_ws/src/duckietown/machines

all: $(machines)


$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)
 


