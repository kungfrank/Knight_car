import numpy as np
from . import logger
from duckietown_utils.expand_variables import expand_environment
import os

__all__ = [
    'd8n_read_images_interval',
    'd8n_read_all_images',
]

def d8n_read_images_interval(filename, t0, t1):
    """
        Reads all the RGB data from the bag,
        in the interval [t0, t1], where t0 = 0 indicates
        the first image.
    
    """
    data = d8n_read_all_images(filename, t0, t1)
    logger.info('Read %d images from %s.' % (len(data), filename))
    timestamps = data['timestamp']
    # normalize timestamps
    first = data['timestamp'][0]
    timestamps -= first
    logger.info('Sequence has length %.2f seconds.' % timestamps[-1])
    selection = np.logical_and(timestamps >= t0, timestamps <= t1)
    nselected = np.sum(selection)

    if nselected == 0:
        msg = 'The slice of time is empty.'
        raise ValueError(msg)
    
    selected = data[selection]
    return selected
    
def d8n_read_all_images(filename, t0=None, t1=None):
    """ 
    
        Raises a ValueError if not data could be read.
    
        Returns a numpy array. 
        
        data = d8n_read_all_images(bag)
                
        print data.shape # (928,)
        print data.dtype # [('timestamp', '<f8'), ('rgb', 'u1', (480, 640, 3))]

    """
    import rosbag  # @UnresolvedImport
    filename = expand_environment(filename)
    if not os.path.exists(filename):
        msg = 'File does not exist: %r' % filename
        raise ValueError(msg)
    bag = rosbag.Bag(filename)
    that_topic = get_image_topic(bag)

    data = []
    first_timestamp = None
    with rosbag.Bag(filename, 'r') as bag:
        for j, (topic, msg, t) in enumerate(bag.read_messages()):
            if topic == that_topic:
                float_time = t.to_sec()
                if first_timestamp is None:
                    first_timestamp = float_time
                
                rel_time = float_time - first_timestamp
                if t0 is not None:
                    if rel_time < t0:
                        continue
                if t1 is not None:
                    if rel_time > t1:
                        continue
                rgb = numpy_from_ros_compressed(msg)
                data.append({'timestamp': float_time, 'rgb': rgb})

                if j % 10 == 0:
                    print('Read %d images from topic %s' % (j, topic))

    print('Returned %d images' % len(data))
    if not data:
        raise ValueError('no data found')
    
    H, W, _ = rgb.shape  # (480, 640, 3)
    print('Detected image shape: %s x %s' % (W, H))
    n = len(data)
    dtype = [
        ('timestamp', 'float'),
        ('rgb', 'uint8', (H, W, 3)),
    ]
    x = np.zeros((n,), dtype=dtype)
    for i, v in enumerate(data):
        x[i]['timestamp'] = v['timestamp']
        x[i]['rgb'][:] = v['rgb']

    return x

def get_image_topic(bag):
    topics = bag.get_type_and_topic_info()[1].keys()
    for t in topics:
        if 'camera_node/image/compressed' in t:
            return t 
    msg = 'Cannot find the topic: %s' % topics
    raise ValueError(msg)

def numpy_from_ros_compressed(msg):
    if 'CompressedImage' in msg.__class__.__name__:
        return rgb_from_pil(pil_from_CompressedImage(msg))
    assert False, msg.__class__.__name__

def pil_from_CompressedImage(msg):
    from PIL import ImageFile  # @UnresolvedImport
    parser = ImageFile.Parser()
    parser.feed(msg.data)
    res = parser.close()
    return res

def rgb_from_pil(im):
    return np.asarray(im).astype(np.uint8)


