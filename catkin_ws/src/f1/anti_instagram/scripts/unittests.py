#!/usr/bin/env python
# colored logging
import os
import sys

from duckietown_utils import col_logging 
from anti_instagram import logger
from anti_instagram import AntiInstagram
from anti_instagram.unit_tests import load_tests
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
    dirname = 'catkin_ws/src/f1/anti_instagram/scripts/'
    filename = 'all_tests.yaml'
    filename = os.path.join(root, dirname, filename)

    alltests = load_tests(filename)
    estimators = {'anti_instagram' : AntiInstagram}
    
    which_tests = expand_string(which_tests0, list(alltests))
    which_estimators = expand_string(which_estimators0, list(estimators))

    logger.info('     tests: %r |-> %s' % (which_tests0, which_tests))
    logger.info('estimators: %r |-> %s' % (which_estimators0, which_estimators))

    # which tests to execute
    test_results = []
    for id_test in which_tests:
        for id_estimator in which_estimators:
            test_results.append(run_test(id_test, alltests[id_test],
                                    id_estimator, estimators[id_estimator]))

    if(test_results.count(False)==0):
        logger.info('All tests passed')
    else:
        logger.error('Some tests failed')

def is_match(detection, expected):
    # Determines whether a detection matches with an expectation
    # if either the frequency or the position match but something
    # else doesn't, it warns about what it is
    predicates = dict({
    'position': abs(detection.pixels_normalized.x-expected['image_coordinates'][0])<expected['image_coordinates_margin']
    and abs(detection.pixels_normalized.y-expected['image_coordinates'][1])<expected['image_coordinates_margin'],
    'frequency': detection.frequency == expected['frequency'],
    #'timestamps': abs(detection.timestamp1-expected['timestamp1'])<0.1 and
    #              abs(detection.timestamp2-expected['timestamp2'])<0.1
    })

    unsatisfied = [n for n in predicates if not predicates[n]]
    if(unsatisfied and (predicates['position'] or predicates['frequency'])):
        logger.warning('\nAlmost a match - (%s mismatch) - between detection: \n%s \nand expectation: \n%s'
                        % (unsatisfied, detection, expected))

    return not unsatisfied

def find_match(detection, expected_set):
    # return index (in expected) of the first match to detection
    # or -1 if there is no match
    try:
        return (n for n in range(len(expected_set)) if \
        (is_match(detection, expected_set[n]))).next()
    except StopIteration:
        return -1

def run_test(id_test, test, id_estimator, estimator):
    logger.info('     id_test: %s' % id_test)
    logger.info('id_estimator: %s' % id_estimator)
    from anti_instagram.unit_tests import AntiInstagramUnitTest
    assert isinstance(test, AntiInstagramUnitTest)
    query = test.get_query()
    result = estimator.detect_led(**query)

    # We are testing whether the expected detections are a subset of 
    # the returned ones, we will accept duplicate detections of the
    # same LED 
    match_count = [0]*len(test.expected)
    for r in result.detections:
        m = find_match(r, test.expected) 
        if(m != -1):
            match_count[m]+=1
   
    missedLEDs = [test.expected[i] for i in range(0,  len(match_count)) if match_count[i]==0]
    if(missedLEDs):
        logger.error('missed LED detections (%s): \n %s' % (len(missedLEDs),missedLEDs))

    return not 0 in match_count

if __name__ == '__main__':
    wrap_main(main)

