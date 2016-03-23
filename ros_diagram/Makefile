all: Duckietown_ROS_Diagram.pdf

%.dot: %.dot.in
	python prepare.py < $< > $@

%.pdf: %.dot
	dot $< -Tpdf -o $@