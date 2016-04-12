import cv2
import numpy as np

#from PIL import Image as pimg
#import jpeg4py as jpeg


def image_cv_from_jpg(data):
    """ Returns an OpenCV image from a string """
    image_cv = cv2.imdecode(np.fromstring(data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
    return image_cv



def image_cv_from_jpg_fn(fn):
    with open(fn) as f:
        return image_cv_from_jpg(f.read())





# with PIL Image
# image_cv = jpeg.JPEG(np.fromstring(image_msg.data, np.uint8)).decode()

# with libjpeg-turbo
# Convert from uncompressed image message
# image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
