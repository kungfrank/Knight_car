from api import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray
from led_detection import logger
from math import floor, ceil
import numpy as np

# plotting 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Rectangle

# image filters
from scipy.ndimage.filters import maximum_filter
from scipy.ndimage.morphology import generate_binary_structure, binary_erosion

# fft
import scipy.fftpack

__all__ = ['LEDDetector']

class LEDDetector(LEDDetector):
    """ The LEDDetector class """

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
        return (cell_values, [ceil(.5*rest_y), ceil(.5*rest_x)])

    # ~~~~~~~~~~~~~~~~~~~ Find local maxima ~~~~~~~~~~~~~~~~~~~~

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

        #plt.figure()
        #plt.imshow(variance,cmap=cm.gray, interpolation="nearest")
        #plt.title('Variance map')

        #plt.figure()
        #plt.imshow(peaks_mask*threshold_mask)
        #plt.title('Peaks')

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

        if not isinstance(frequencies_to_detect, list):
            raise ValueError(frequencies_to_detect)

        if not min_distance_between_LEDs_pixels > 0:
            raise ValueError(min_distance_between_LEDs_pixels)

        channel = images['rgb'][:,:,:,0] # just using first channel

        cell_width = 15
        cell_height = 15

        (cell_vals, crop_offset) = self.downsample(channel, cell_width, cell_height)

        candidates_mask = self.get_candidate_cells(cell_vals, 100)
        candidate_cells = [(i,j) for (i,j) in np.ndindex(candidates_mask.shape) if candidates_mask[i,j]]

        # Create result object
        result = LEDDetectionArray()

        # Detect frequencies and discard non-periodic signals
        ts_tolerance = 0.1
        T = timestamps[-1]-timestamps[0]
        min_num_periods = 5

        for (i,j) in candidate_cells:
            signal = cell_vals[:,i,j]
            signal = signal-np.mean(signal)
            print('signal.shape: {0}'.format(signal.shape))

            zero_crossings = np.where(np.diff(np.sign(signal)))[0]
            zero_crossings_t = timestamps[zero_crossings]
            led_img_coords = Vector2D((0.5+j)*cell_width+crop_offset[1], (0.5+i)*cell_height+crop_offset[0])       
            logger.info('Coords: %s, %s'% (led_img_coords.x,led_img_coords.y))
            logger.info('Zero crossings: %s'%zero_crossings_t)

            diffs = [b-a for a, b in zip(zero_crossings_t, zero_crossings_t[1:])]
            logger.info('Diffs: %s'%diffs)
            logger.info('Measured freq %s'% (0.5/np.mean(diffs)))

            print(frequencies_to_detect)
            for f in frequencies_to_detect:
                if(len(zero_crossings)<min_num_periods):
                    logger.info('Not an LED, discarded\n')
                    break
                if(all(d-ts_tolerance <= 0.5/f <= d+ts_tolerance for d in diffs)):
                    logger.info('Confirmed LED with frequency %s\n'%f)
                   # recover coordinates of centroid
                    result.detections.append(LEDDetection(timestamps[0], timestamps[-1],
                    led_img_coords, f, '', -1)) # -1...confidence not implemented
                    break

            # Plot all signals
            #fig, ax1 = plt.subplots()
            #ax1.plot(timestamps, signal)
            #plt.show()

        plt.imshow(rgb0)
        ax = plt.gca()

        font = {'family': 'serif',
                'color':  'red',
                'weight': 'bold',
                'size': 16,
                }
        # Plot all results
        for r in result.detections:
            pos = r.pixels_normalized
            ax.add_patch(Rectangle((pos.x-0.5*cell_width, pos.y-0.5*cell_height), cell_width, cell_height, edgecolor="red", linewidth=3, facecolor="none"))
            plt.text(pos.x-0.5*cell_width, pos.y-cell_height, str(r.frequency), fontdict=font)
        #    pass

        plt.show()

        return result


