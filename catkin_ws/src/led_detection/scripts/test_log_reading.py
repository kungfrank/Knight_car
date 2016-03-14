#!/usr/bin/env python
import rosbag
import sys

args = sys.argv[1:]

if len(args) != 1:
    print('Usage: \n\n\t\t test_log_reading <bag>')
    sys.exit(1)

filename = sys.argv[1]

print('filename: %s' % filename)

from duckietown_utils import d8n_read_all_images
data = d8n_read_all_images(filename)

