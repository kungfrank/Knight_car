#!/usr/bin/env bash

#Setup Google Drive Root 
read -e -p "Please enter path to Google Drive root: " -i ~/grive GRIVE_ROOT
export SUB_FOLDER=Duckietown/Duckietown-Public/Duckietown_design

echo "GRIVE_ROOT set to $GRIVE_ROOT"

echo "Generating pdf file..."
dot -O Duckietown_ROS_Diagram.dot -Tpdf

echo "Copying Duckietown_ROS_Diagram.* to $GRIVE_ROOT/$SUB_FOLDER"
cp Duckietown_ROS_Diagram.* $GRIVE_ROOT/$SUB_FOLDER/

echo "Removing Duckietown_ROS_Diagram.dot.pdf"
rm Duckietown_ROS_Diagram.dot.*

if [ "$(uname)" == "Linux" ]; then
    echo "Calling grive to sync to google drive if on Linux."
    cd $GRIVE_ROOT
    grive -V  
fi
