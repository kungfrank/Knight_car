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


print data.shape # (928,)
print data.dtype # [('timestamp', '<f8'), ('rgb', 'u1', (480, 640, 3))]

num_images = data.shape[0]
for k in range(num_images):
    
    timestamp = data[k]['timestamp']
    rgb = data[k]['rgb']
    
    assert rgb.shape == (480, 640, 3)
    
# fps

timestamps = data[:]['timestamp']
period = (timestamps[-1] - timestamps[0]) / num_images
fps = 1.0 / period

print('fps: %.2f' % fps)
