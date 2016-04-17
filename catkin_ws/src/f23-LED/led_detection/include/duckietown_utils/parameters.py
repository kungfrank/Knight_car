import yaml

class Configurable():
    
    def __init__(self, param_names, configuration):

        # check that we set all parameters
        given = list(configuration)
        
        required = list(param_names)
        
        extra = set(given) - set(required)
        missing = set(required) - set(given)
        if extra or missing:
            msg = ('Error while loading configuration for %r from %r.' % 
                   (self, configuration))
            msg += '\n'
            msg += 'Extra parameters: %r\n' % extra
            msg += 'Missing parameters: %r\n' % missing
            raise ValueError(msg)

        assert set(given) == set(required)
        for p in param_names:
            setattr(self, p, stuff[p])