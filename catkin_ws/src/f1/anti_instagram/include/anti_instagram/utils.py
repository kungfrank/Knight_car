# candidates to be part of "duckietown_utils"

def L2_image_distance(a, b):
    import numpy as np
    return np.mean(np.square(a * 1.0 - b * 1.0))


def random_image(h, w):
    import numpy as np
    return np.array(np.random.random((h, w, 3)) * 255, dtype=np.uint8)

def load_image(f):
    return read_file(f)

def read_file(filename):
    import cv2
    from duckietown_utils.expand_variables import expand_environment

    filename = expand_environment(filename)
    img = cv2.imread(filename)
    if  img is None:
        msg = 'Cannot read filename %r.' % filename
        raise ValueError(msg)
    return img


def wrap_test_main(f):
    from . import logger
    import traceback, sys
    try:
        f()
    except Exception as e:
        logger.error(traceback.format_exc(e))
        logger.error('Exiting with error code 1')
        sys.exit(1)
    except:  # another weird exception
        logger.error('Exiting with error code 2')
        sys.exit(2)
    else:
        logger.info('Success.')
        sys.exit(0)

