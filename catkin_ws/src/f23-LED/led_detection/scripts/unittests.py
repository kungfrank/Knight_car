#!/usr/bin/env python
# colored logging
from duckietown_utils import col_logging  # @UnusedImport
from led_detection import logger

from led_detection.dummy import DummyLEDDetector
from led_detection.unit_tests import load_tests
import os
import sys
from duckietown_utils.wildcards import expand_string
from duckietown_utils.wrap_main import wrap_main

def main():
    script_name = os.path.basename(sys.argv[0])
    args = sys.argv[1:]
    if len(args) != 2:
        msg = """
Usage:

    rosrun led_detection <script> <tests> <algorithms>
    
where:

    <tests> = comma separated list of algorithms. May use "*".
    <algorithms> = comma separated list of algorithms. May use  "*".
    
For example, this runs all tests on all algorithms:


    rosrun led_detection <script> '*' '*'
    
"""

        msg = msg.replace('<script>', script_name)
        logger.error(msg)
        sys.exit(2)

    which_tests0 = sys.argv[1]
    which_estimators0 = sys.argv[2]

    root = os.environ['DUCKIETOWN_ROOT']
    dirname = 'catkin_ws/src/f23-LED/led_detection/scripts/'
    filename = '20160312-allblinking_test1-argo.led_detection_test.yaml'
    filename = os.path.join(root, dirname, filename)

    alltests = load_tests(filename)
    estimators = {'dummy': DummyLEDDetector()}
    
    which_tests = expand_string(which_tests0, list(alltests))
    which_estimators = expand_string(which_estimators0, list(estimators))

    logger.info('     tests: %r |-> %s' % (which_tests0, which_tests))
    logger.info('estimators: %r |-> %s' % (which_estimators0, which_estimators))

    # which tests to execute
    for id_test in which_tests:
        for id_estimator in which_estimators:
            test_results = run_test(id_test, alltests[id_test],
                                    id_estimator, estimators[id_estimator])

   
def find_match(detection, expected):
    # scan the list of expected detections and see if you find one
    # return first match
    # TODO this can probably be done in a more python style
    for i in range(0, len(expected)):
        e = expected[i]
        if(abs(detection.pixels_normalized.x-e['image_coordinates'][0])<e['image_coordinates_margin']\
           and abs(detection.pixels_normalized.y-e['image_coordinates'][1])<e['image_coordinates_margin']):
            return i

    return -1

def run_test(id_test, test, id_estimator, estimator):
    logger.info('     id_test: %s' % id_test)
    logger.info('id_estimator: %s' % id_estimator)
    from led_detection.unit_tests import LEDDetectionUnitTest
    assert isinstance(test, LEDDetectionUnitTest)
    query = test.get_query()
    result = estimator.detect_led(**query)
    logger.info(test.expected[0])
    match_count = [0]*len(test.expected)
    for r in result.detections:
        m = find_match(r, test.expected) 
        if(m != -1):
            match_count[m]+=1

    logger.info(match_count)

    # TODO before return, output which LED was expected and not detected 

    return not 0 in match_count

if __name__ == '__main__':
    wrap_main(main)

