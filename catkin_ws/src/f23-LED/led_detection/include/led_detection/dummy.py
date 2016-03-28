from api import LEDDetector

__all__ = ['DummyLEDDetector']

class DummyLEDDetector(LEDDetector):
    """ A mockup """

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

        #tuneable parameters
        partitions_x = 10
        partitions_y = 10
        intensity_variance_threshold = 10 #TODO: this is arbitrary

        #should be 480x640?
        W = rgb0.shape[0]
        H = rgb0.shape[1]
        partition_width = W / partitions_x
        partition_height = H / partitions_y

        # jump by partition
        for x in range(0, W, partition_width):
            for y in range(0, H, partition_height):
                # look at same partition across multiple images
                average_intensities = []
                for image in images['rgb']:
                    intensities = []
                    # loop through pixels in each partition
                    for x_coord in range(x, x+partition_width):
                        for y_coord in range(y,y+partition_height):
                            if x_coord > W or y_coord > H:
                                continue
                            pixel = image[x_coord][y_coord]
                            # a simple approach from http://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
                            #could also use matplotlib's rgb_to_hsv?
                            #TODO: check if pixel order is RGB or BGR (assuming RGB)
                            intensity = 0.299*pixel[0] + 0.587*pixel[1] + 0.114*pixel[2]
                            intensities.append(intensity)

                    average_intensities.append(numpy.mean(intensities))
                variance = numpy.var(average_intensities)
                if variance > intensity_variance_threshold:
                    #threshold into "on" and "off" states
                    pass
                #determine frequency
        return []
