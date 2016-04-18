

def ipython_if_guy():
    """ 
    Use like this:

        from duckietown_utils.ipython import ipython_if_gui
        
        
        ipython_if_guy()
    
    """
    import getpass
    user = getpass.getuser()

    return user in ['guy']
