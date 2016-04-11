#!/usr/bin/env bash

#Setup Google Drive Root 
read -e -p "Please enter path to Google Drive root: " -i ~/grive GRIVE_ROOT
# export SUB_FOLDER=Duckietown/Duckietown-Public/Duckietown_design
export SUB_FOLDER=duckietown-public/design

echo "GRIVE_ROOT set to $GRIVE_ROOT"

echo "Generating pdf file..."
make
#Duckietown_ROS_Diagram.pdf

echo "Copying Duckietown_ROS_Diagram.dot.pdf to $GRIVE_ROOT/$SUB_FOLDER"

cp Duckietown_ROS_Diagram.pdf $GRIVE_ROOT/$SUB_FOLDER/Duckietown_ROS_Diagram.pdf

echo "Removing Duckietown_ROS_Diagram.dot.pdf..."
rm Duckietown_ROS_Diagram.pdf

if [ "$(uname)" == "Linux" ]; then
    echo "Calling grive to sync to google drive if on Linux."
    cd $GRIVE_ROOT
    grive -V  
    cd -
fi
