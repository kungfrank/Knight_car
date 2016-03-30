from api import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray
from led_detection import logger
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from math import floor, ceil

__all__ = ['LEDDetector']

class LEDDetector(LEDDetector):
    """ A mockup """

    def __init__(self):
        pass

    # ~~~~~~~~~~~~~~~~~~~ Downsample ~~~~~~~~~~~~~~~~~~~~~~~~

    def downsample(self, channel, cell_width=20, cell_height=20):
        W = channel.shape[2]
        H = channel.shape[1]

        print('Original shape: {0}'.format(channel[0].shape))

        # crop image around borders to get integer submultiples
        ncells_x = floor(1.0*W/cell_width)
        ncells_y = floor(1.0*H/cell_height)
        rest_x = W%cell_width
        rest_y = H%cell_height
        imgs_cropped = channel[:, ceil(.5*rest_y):H-floor(.5*rest_y), ceil(.5*rest_x):W-floor(.5*rest_x)]
        print('ncells_x: %s, ncells_y: %s' % (ncells_x, ncells_y))
        print('rest_x: %s, rest_y: %s' % (rest_x, rest_y))
        print('Cropped shape: {0}'.format(imgs_cropped.shape))
        assert imgs_cropped.shape[1]%cell_height == 0
        assert imgs_cropped.shape[2]%cell_width == 0

        # Split images into rectangles
        cell_values = np.array(np.split(imgs_cropped,ncells_x, axis=2))
        print('First split: {0}'.format(cell_values.shape))
        cell_values = np.mean(cell_values, axis=3)
        print('First reduction: {0}'.format(cell_values.shape))
        cell_values = np.array(np.split(cell_values,ncells_y, axis=2))
        print('Second split: {0}'.format(cell_values.shape))
        cell_values = np.mean(cell_values, axis=3)
        print('Second reduction: {0}'.format(cell_values.shape))
        cell_values = np.swapaxes(cell_values, 2, 0)
        cell_values = np.swapaxes(cell_values, 2, 1)
        return cell_values

    # ~~~~~~~~~~~~~~~~~~~ Select candidates ~~~~~~~~~~~~~~~~~~~~

    def get_candidate_cells(self, cell_values, threshold):
        variance = np.var(cell_values, axis=0)

        plt.figure()
        plt.imshow(variance, cmap=cm.gray, interpolation="nearest")
        plt.title('Variance map')
        plt.show()
        pass
    
    # ~~~~~~~~~~~~~~~~~~~ Detect LEDs ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def detect_led(self,
                   images,
                   mask,
                   frequencies_to_detect,
                   min_distance_between_LEDs_pixels):

        assert len(images.shape) == 1
        n = images.shape[0]
        if n == 0:
            raise ValueError('No images provided')

        timestamps = images['timestamp']
        rgb = images['rgb']

        rgb0 = rgb[0]
        if not mask.shape == rgb0.shape:
            raise ValueError('Invalid mask')
        
        logger.info(mask)

        if not isinstance(frequencies_to_detect, list):
            raise ValueError(frequencies_to_detect)

        if not min_distance_between_LEDs_pixels > 0:
            raise ValueError(min_distance_between_LEDs_pixels)

        channel = images['rgb'][:,:,:,0] # just using first channel

        cell_vals = self.downsample(channel, 10, 10)

        self.get_candidate_cells(cell_vals, 0)

