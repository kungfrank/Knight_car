#!/usr/bin/env python

from anti_instagram import  logger
import timeit
from anti_instagram.utils import get_rospkg_root
import os
from duckietown_utils.jpg import image_cv_from_jpg
 


def get_test_image():
    package_root = get_rospkg_root('anti_instagram')
    return os.path.join(package_root, 'tests', 'frame.jpg')


def setup():
    with open(get_test_image(), 'rb') as f:
        return f.read()

def decode1(data):
    return image_cv_from_jpg(data)

def decode2(data):
    return image_cv_from_jpg(data)

def decode3(data):
    return image_cv_from_jpg(data)


if __name__ == '__main__':
    n = 10
    methods = ['decode1', 'decode2', 'decode3']
    for m in methods:
        tn = timeit.timeit(stmt='from __main__ import %s; %s(data)' % (m, m),
                       setup='from __main__ import setup; data=setup()',
                       number=n
                       )
        t = tn / n
        logger.info("method %s, avg over %d tries: %.1f ms " % (m, n, t * 1000))

