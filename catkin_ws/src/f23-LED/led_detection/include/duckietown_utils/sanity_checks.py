from. import logger

def check_years():
    import datetime
    now = datetime.datetime.now()
    if now.year < 2016:
        msg = 'The date is not set correctly. This will screw up building.'
        raise Exception(msg)
    return '%s' % now.year

def check_failure():
    raise ValueError('error')

def do_all_checks():
    """ Returns the names of the failures  """

    checks = [
        check_years,
        check_failure,
    ]

    results = []
    for c in checks:
        logger.debug('Checking %s...' % c.__name__)
        try:
            res = c()
            results.append((True, res))
        except Exception as e:
            r = False, str(e)
            results.append(r)
            logger.error(e)

    failures = []
    for c, (status, s) in zip(checks, results):
        if status:
            mark = ' OK'
        else:
            mark = 'XXX'

        if len(s) > 25:
            s = s[:22] + ' [...]'
        s = '%20s - %s - %s' % (c.__name__, mark, s)
        if not status:
            failures.append(c.__name__)
            logger.error(s)
        else:
            logger.info(s)
    return failures
