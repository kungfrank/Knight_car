
scuderia := scuderia.yaml
machines := catkin_ws/src/duckietown/machines

all: $(machines)


$(machines): $(scuderia)
	python create-config.py $(scuderia) > $(machines)
 


