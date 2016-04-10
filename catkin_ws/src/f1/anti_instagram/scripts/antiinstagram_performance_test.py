#!/usr/bin/env python
from anti_instagram import wrap_test_main, SASParams, logger, random_image, AntiInstagram
import timeit

class Params:
    shape = None

def anti_instagram_test_performance():
    logger.info('This is going to test the performance of algorithm 1 and 2')

    for i in [1,2]:
        SASParams.algorithm = i    
        shapes = [ (480, 640), (240, 320), (120, 160) ]

        for Params.shape in shapes:
            res = applyTransformOnRandomImg()
            print('algo: %d Shape: %s   -> %1.f ms' % (i, str(Params.shape), 1000*res))
    
    for Params.shape in shapes:
        print('Shape: %s' % str(Params.shape))
        res = calcuateTransformOnRandomImg()

def setup():
    img = random_image(Params.shape[0], Params.shape[1])
    ai = AntiInstagram()
    return ai, img

def applyTransformOnRandomImg():
    n = 10
    tn = timeit.timeit(stmt='ai.applyTransform(img)',
                       setup='from __main__ import setup; ai,img=setup()',
                       number=n
                       )
    t = tn / n
    #logger.info("Average Apply Transform Took: %.1f ms " % (t * 1000))
    return t

def calcuateTransformOnRandomImg():
    n = 10
    tn = timeit.timeit(stmt='ai.calculateTransform(img,True)',
                       setup='from __main__ import setup; ai,img=setup()',
                       number=n)

    t = tn / n
    logger.info("Average Calculate Transform Took: %.1f ms" % (t * 1000))

if __name__ == '__main__':
    wrap_test_main(anti_instagram_test_performance) 
