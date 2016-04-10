from . import logger
import numpy as np

class SASParams:
    algorithm = 2

def scaleandshift(img, scale, shift):
    logger.info('scale: %s' % scale)
    logger.info('shift: %s' % shift)

    assert img.shape[2] == 3
    assert len(scale) == 3, scale
    assert len(shift) == 3, shift

    if SASParams.algorithm == 1:
        return scaleandshift1(img, scale, shift)

    if SASParams.algorithm == 2:
        return scaleandshift2(img, scale, shift)

    assert False

def scaleandshift2(img, scale, shift):
    img_shift = np.empty_like(img)
    for i in range(3):
        img_shift[:, :, i] = scale[i] * img[:, :, i] + shift[i]

    return img_shift

def scaleandshift1(img, scale, shift):
    h = img.shape[0]
    w = img.shape[1]

    img_scale = np.reshape(img, [h * w, 3])
    img_scale = np.reshape(img_scale * np.array(scale), [h, w, 3])

    img_shift = np.reshape(img_scale, [h * w, 3])
    img_shift = np.reshape(img_shift + np.array(shift), [h, w, 3])

    return img_shift
