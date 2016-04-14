#!/usr/bin/env python
from anti_instagram import AntiInstagram, logger, wrap_test_main
from anti_instagram.AntiInstagram import ScaleAndShift, calculate_transform
from duckietown_utils.expand_variables import expand_environment
from duckietown_utils.jpg import (image_clip_255, image_cv_from_jpg_fn,
    make_images_grid)
from duckietown_utils.locate_files_impl import locate_files
from line_detector.LineDetector import LineDetector
from line_detector.LineDetector2 import LineDetector2
import cv2
import os
import scipy.io
import IPython

def examine_dataset(dirname, out):
    logger.info(dirname)
    dirname = expand_environment(dirname)

    jpgs = locate_files(dirname, '*.jpg')
    mats = locate_files(dirname, '*.mat')

    logger.debug('I found %d jpgs and %d mats' % (len(jpgs), len(mats)))

    if len(jpgs) == 0:
        msg = 'Not enough jpgs.'
        raise ValueError(msg)

#     if len(mats) == 0:
#         msg = 'Not enough mats.'
#         raise ValueError(msg)

    first_jpg = sorted(jpgs)[0]
    logger.debug('Using jpg %r to learn transformation.' % first_jpg)

    first_jpg_image = image_cv_from_jpg_fn(first_jpg)


    success, health, parameters = calculate_transform(first_jpg_image)

    s = ""
    s += 'success: %s\n' % str(success)
    s += 'health: %s\n' % str(health)
    s += 'parameters: %s\n' % str(parameters)
    w = os.path.join(out, 'learned_transform.txt')
    with open(w, 'w') as f:
        f.write(s)
    logger.info(s)
    
    transform = ScaleAndShift(**parameters)
    
    for j in jpgs:
        shape = (200, 160)
        interpolation = cv2.INTER_NEAREST
        name = 'line_detector'
 
        summaries =[]
        LineDetectorClass = LineDetector
        s =run_detection(transform, j, out, shape=shape,
                      interpolation=interpolation, name=name,
                      LineDetectorClass=LineDetectorClass)
        summaries.append(s)

        name = 'line_detector2'
        LineDetectorClass = LineDetector2
        s=run_detection(transform, j, out, shape=shape,
                      interpolation=interpolation, name=name,
                      LineDetectorClass=LineDetectorClass)
        summaries.append(s)
        
        together = make_images_grid(summaries, cols=1, pad=10, bgcolor=[.5, .5, .5])

        # 
        bn = os.path.splitext(os.path.basename(j))[0]
        fn = os.path.join(out, '%s.all.png' % (bn))
        cv2.imwrite(fn, zoom_image(together, 4))

    for m in mats:
        logger.debug(m)
        jpg = os.path.splitext(m)[0] + '.jpg'
        if not os.path.exists(jpg):
            msg = 'JPG %r for mat %r does not exist' % (jpg, m)
            logger.error(msg)
        else:
            test_pair(transform, jpg, m, out)
        
def zoom_image(im, zoom):
    zoom = 4
    s = (im.shape[1] * zoom, im.shape[0] * zoom)
    imz = cv2.resize(im, s, interpolation=cv2.INTER_NEAREST)
    return imz

def run_detection(transform, jpg, out, shape, interpolation,
                  name, LineDetectorClass):
    image = image_cv_from_jpg_fn(jpg)

    image = cv2.resize(image, shape, interpolation=interpolation)
#     bgr = bgr[bgr.shape[0] / 2:, :, :]

    image_detections = line_detection(LineDetectorClass, image)
    transformed = transform(image)

    transformed_clipped = image_clip_255(transformed)
    transformed_detections = line_detection(LineDetectorClass, transformed_clipped)

    if not os.path.exists(out):
        os.makedirs(out)
    bn = os.path.splitext(os.path.basename(jpg))[0]

    def write(postfix, im):
        fn = os.path.join(out, '%s.%s.%s.png' % (bn, name, postfix))
        cv2.imwrite(fn, zoom_image(im, 4))

    together = make_images_grid([image,  # transformed,
                                 merge_masks_res(image_detections),
                                 gray2rgb(image_detections['edges']),
                                 image_detections['annotated'],
                                 
                                 transformed_clipped,
                                 merge_masks_res(transformed_detections),
                                 gray2rgb(transformed_detections['edges']),
                                 transformed_detections['annotated'],
                       ], 
                                
                                cols=4, pad=10, bgcolor=[1, 1, 1])
    write('together', together)
    return together

def merge_masks_res(res):
    return merge_masks(res['area_white'], res['area_red'], res['area_yellow'])

def merge_masks(area_white, area_red, area_yellow):
    B, G, R = 0, 1, 2
    def white(x):
        x = gray2rgb(x)
        return x
    def red(x):
        x = gray2rgb(x)
        x[:,:,R] *= 1
        x[:,:,G] *= 0
        x[:,:,B] *= 0
        return x
    def yellow(x):
        x = gray2rgb(x)
        x[:,:,R] *= 1
        x[:,:,G] *= 1
        x[:,:,B] *= 0
        return x
    h, w = area_white.shape
    orig = [area_white, area_red, area_yellow]
    masks = [white(area_white), red(area_red), yellow(area_yellow)]
        
    res = np.zeros(shape=masks[0].shape, dtype=np.uint8)
    
    for i, m in enumerate(masks):
        nz = (orig[i] > 0) * 1.0
        assert nz.shape == (h, w), nz.shape
        
        for j in [0, 1, 2]:
            res[:,:,j] = (1-nz) * res[:,:,j].copy() + (nz) * m[:,:,j]
    
    return res
    

def test_pair(transform, jpg, mat, out):
    """ 
        jpg = filename
        mat = filename
    """

    data = scipy.io.loadmat(mat)
    regions = data['regions'].flatten()
    for r in regions:
        logger.info('region')
        x = r['x'][0][0].flatten()
        y = r['y'][0][0].flatten()
        mask = r['mask'][0][0]
        print 'x', x
        print 'y', y
        print 'mask shape', mask.shape
        print 'type', r['type'][0][0][0][0] # type in 1- based / matlab-based indices from the list of region types (i.e road, white, yellow, red, or what ever types were annotated) 
        print 'color', r['color'][0] # color in [r,g,b] where [r,g,b]are between 0 and 1
        # print 'guy look here'
        # IPython.embed()
        # XXX: to finish

def line_detection(LineDetectorClass, bgr):
    detector = LineDetectorClass()
    detector.setImage(bgr)

    if isinstance(detector, LineDetector):
        # detect lines and normals
        lines_white, normals_white, area_white = detector.detectLines('white')
        lines_yellow, normals_yellow, area_yellow = detector.detectLines('yellow')
        lines_red, normals_red, area_red = detector.detectLines('red')

        # draw lines
        detector.drawLines(lines_white, (0, 0, 0))
        detector.drawLines(lines_yellow, (255, 0, 0))
        detector.drawLines(lines_red, (0, 255, 0))
    
    elif isinstance(detector, LineDetector2):
        # detect lines and normals
        lines_white, normals_white, centers_white, area_white = detector.detectLines2('white')
        lines_yellow, normals_yellow, centers_yellow, area_yellow = detector.detectLines2('yellow')
        lines_red, normals_red, centers_red, area_red = detector.detectLines2('red')

        # draw lines
        detector.drawLines(lines_white, (0, 0, 0))
        detector.drawLines(lines_yellow, (255, 0, 0))
        detector.drawLines(lines_red, (0, 255, 0))
        # draw normals
        #detector.drawNormals2(centers_yellow, lines_yellow, normals_yellow)
        #detector.drawNormals2(centers_white, lines_white, normals_white)
        #detector.drawNormals2(centers_red, lines_red, normals_red)
    
        print('XXX Hang Zhao please fix')
        # draw normals
#         detector.drawNormals2(centers_yellow, lines_yellow, normals_yellow)
#         detector.drawNormals2(centers_white, lines_white, normals_white)
#         detector.drawNormals2(centers_red, lines_red, normals_red)
        
    res = {}
    res['annotated'] = detector.getImage()
    res['area_white'] = area_white
    res['area_red'] = area_red
    res['area_yellow'] = area_yellow
    res['edges'] = detector.edges
    return res

#    cv2.imwrite('lines_with_normal.png', detector.getImage())

import numpy as np
def gray2rgb(gray):
    ''' 
        Converts a H x W grayscale into a H x W x 3 RGB image 
        by replicating the gray channel over R,G,B. 
        
        :param gray: grayscale
        :type  gray: array[HxW](uint8),H>0,W>0
        
        :return: A RGB image in shades of gray.
        :rtype: array[HxWx3](uint8)
    '''
#    assert_gray_image(gray, 'input to gray2rgb')

    rgb = np.zeros((gray.shape[0], gray.shape[1], 3), dtype='uint8')
    for i in range(3):
        rgb[:, :, i] = gray
    return rgb


def anti_instagram_annotations_test():
    base = "${DUCKIETOWN_DATA}/phase3-misc-files/so1/"

    base = expand_environment(base)
    dirs = locate_files(base, '*.iids1', alsodirs=True)

    for d in dirs:
        import getpass
        uname = getpass.getuser()
        out = os.path.join(os.path.dirname(d), uname, os.path.basename(d) + '.v')
        if not os.path.exists(out):
            os.makedirs(out)
        examine_dataset(d, out)


if __name__ == '__main__':
    wrap_test_main(anti_instagram_annotations_test) 
