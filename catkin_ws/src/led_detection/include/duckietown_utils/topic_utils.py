import numpy as np


def d8n_read_all_images(filename):
    import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(filename)
    that_topic = get_image_topic(bag)

    data = []
    with rosbag.Bag(filename, 'r') as bag:
        for j, (topic, msg, t) in enumerate(bag.read_messages()):
            if topic == that_topic:
                float_time = t.to_sec()
                rgb = numpy_from_ros_compressed(msg)
                data.append({'timestamp': float_time, 'rgb': rgb})

                if j % 10 == 0:
                    print('Read %d images at %s' % (j, topic))

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


