#!/usr/bin/env python
from anti_instagram import logger, wrap_test_main, AntiInstagram
from duckietown_utils.expand_variables import expand_environment
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.jpg import image_cv_from_jpg_fn
import scipy.io
import os
from anti_instagram.AntiInstagram import calculate_transform, ScaleAndShift

def examine_dataset(dirname, out):
    logger.info(dirname)
    dirname = expand_environment(dirname)

    jpgs = locate_files(dirname, '*.jpg')
    mats = locate_files(dirname, '*.mat')

    logger.debug('I found %d jpgs and %d mats' % (len(jpgs), len(mats)))

    if len(jpgs) == 0:
        msg = 'Not enough jpgs.'
        raise ValueError(msg)

    if len(mats) == 0:
        msg = 'Not enough mats.'
        raise ValueError(msg)

    first_jpg = sorted(jpgs)[0]
    logger.debug('Using jpg %r to learn transformation.' % first_jpg)

    first_jpg_image = image_cv_from_jpg_fn(first_jpg)


    success, health, parameters = calculate_transform(first_jpg_image)
    logger.info('success: %s' % str(success))
    logger.info('health: %s' % str(health))
    logger.info('parameters: %s' % str(parameters))

    transform = ScaleAndShift(**parameters)
    
    for m in mats:
        logger.debug(m)
        jpg = os.path.splitext(m)[0] + '.jpg'
        if not os.path.exists(jpg):
            msg = 'JPG %r does not exist'
            raise ValueError(msg)
        
        test_pair(transform, jpg, m, out)
        
       
        
def test_pair(transform, jpg, mat, out):
    """ 
        jpg = filename
        mat = filename
    """ 
    
    image_cv = image_cv_from_jpg_fn(jpg)
    transformed = transform(image_cv)

    if not os.path.exists(out):
        os.makedirs(out)
    import cv2
    bn = os.path.splitext(os.path.basename(jpg))[0]
    cv2.imwrite(os.path.join(out, '%s.orig.jpg' % bn), image_cv)
    cv2.imwrite(os.path.join(out, '%s.transformed.jpg' % bn), transformed)

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
        print 'type', r['type']
        print 'color', r['color']




def anti_instagram_annotations_test():
    datasets = [
        "${DUCKIETOWN_DATA}/phase3-misc-files/so1/annotations_format_example/agirard/",
    ]
    
    for d in datasets:
        examine_dataset(d, 'out')


if __name__ == '__main__':
    wrap_test_main(anti_instagram_annotations_test) 
