all: Duckietown_ROS_Diagram.pdf

fsm: default.pdf

%.dot: %.dot.in
	python prepare.py < $< > $@

%.pdf: %.dot
	dot $< -Tpdf -o $@


update-continuous: 
	python recreate_on_changes.py  Duckietown_ROS_Diagram.dot.in Duckietown_ROS_Diagram.dot

upload-andrea:
	cp Duckietown_ROS_Diagram.pdf /Users/andrea/a.censi@gmail.com/1601-Duckietown/duckietown-public/design/Duckietown_ROS_Diagram.pdf
	
	
