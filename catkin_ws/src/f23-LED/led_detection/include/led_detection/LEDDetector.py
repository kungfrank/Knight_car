from api import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray
from led_detection import logger
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.cm as cm
from math import floor, ceil

# image filters
from scipy.ndimage.filters import maximum_filter
from scipy.ndimage.morphology import generate_binary_structure, binary_erosion

# fft
import scipy.fftpack

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

    def detect_peaks(self, image):
        """
        Takes an image and detect the peaks usingthe local maximum filter.
        Returns a boolean mask of the peaks (i.e. 1 when
        the pixel's value is the neighborhood maximum, 0 otherwise)
        """
        neighborhood = generate_binary_structure(2,2)
        local_max = maximum_filter(image, 5)==image
        background = (image==0)
        eroded_background = binary_erosion(background, structure=neighborhood, border_value=1)
        detected_peaks = local_max - eroded_background
        return detected_peaks

    # ~~~~~~~~~~~~~~~~~~~ Select candidates ~~~~~~~~~~~~~~~~~~~~
    def get_candidate_cells(self, cell_values, threshold):
        variance = np.var(cell_values, axis=0)
        peaks_mask = self.detect_peaks(variance)
        threshold_mask = variance>threshold

        plt.figure()
        plt.imshow(variance,cmap=cm.gray, interpolation="nearest")
        plt.title('Variance map')

        plt.figure()
        plt.imshow(peaks_mask*threshold_mask)
        plt.title('Peaks')

        plt.show()
        return peaks_mask*threshold_mask
        
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

        cell_vals = self.downsample(channel, 15, 15)

        candidates_mask = self.get_candidate_cells(cell_vals, 100)
        candidate_cells = [(i,j) for (i,j) in np.ndindex(candidates_mask.shape) if candidates_mask[i,j]]

        T = 1.0/30 # expectin 30 fps
        f = np.linspace(0.0, 1.0/(2.0*T), n/2)

        print('f.shape: {0}'.format(f.shape))

        for (i,j) in candidate_cells:
            signal = cell_vals[:,i,j]
            signal = signal-np.mean(signal)
            print('signal.shape: {0}'.format(signal.shape))
            # TODO resample!!
            signal_f = scipy.fftpack.fft(signal)
            print('signal_f.shape: {0}'.format(signal_f.shape))
            y_f =  2.0/n * np.abs(signal_f[:n/2])

            fig, ax1 = plt.subplots()
            ax1.plot(signal)
            fig, ax2 = plt.subplots()
            ax2.plot(f,y_f)
            plt.show()


