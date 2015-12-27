#!/bin/bash
set -e
set -x

mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
