#!/usr/bin/env python
from anti_instagram import wrap_test_main, logger
import numpy as np
from anti_instagram.utils import random_image, L2_image_distance, L1_image_distance
from anti_instagram.kmeans import scaleandshift, SASParams

def anti_instagram_test_correctness():
    logger.info('This is going to test that algorithm 1 and 2 give same results')
    
    img = random_image(480, 640)
    scale = np.random.rand(3)
    shift = np.random.rand(3)
    
    SASParams.algorithm = 1
    img1 = scaleandshift(img, scale, shift)

    SASParams.algorithm = 2
    img2 = scaleandshift(img, scale, shift)

    diff_L2 = L2_image_distance(img1, img2)
    diff_L1 = L1_image_distance(img1, img2)


    logger.info('diff_L2: %f' % diff_L2)
    logger.info('diff_L1: %f' % diff_L1)

    assert diff_L1 <= 0.01

if __name__ == '__main__':
    wrap_test_main(anti_instagram_test_correctness) 
