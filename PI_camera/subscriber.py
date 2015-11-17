
from serialization import python_from_wire
import time
import zmq

__all__ = ['PICameraSource']
 

class PICameraSource():
    F_JPG = 'jpg'
    F_NUMPY = 'Numpy'
    F_PIL = 'PIL'
    def __init__(self, address, data_format='jpg', stats=True):
        allowed = [PICameraSource.F_JPG,
                   PICameraSource.F_NUMPY,
                   PICameraSource.F_PIL]
        if not data_format in allowed:
            msg = 'Need one of %s, got %r.' % (allowed, data_format)
            raise ValueError(msg)
        self.address = address
        self.socket = None
        self.stats = stats
        self.data_format = data_format
        
    def reset(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)  # @UndefinedVariable
        # important: set CONFLATE before connecting
        socket.setsockopt(zmq.CONFLATE, 1)  # @UndefinedVariable
        socket.setsockopt(zmq.RCVHWM, 1)  # @UndefinedVariable
        socket.connect(self.address)
        socket.setsockopt(zmq.SUBSCRIBE, '')  # @UndefinedVariable

        self.socket = socket
        self.first = time.time()
        self.nreceived = 0
        self.nmissed_total = 0
        self.last_counter = -1

    def get(self, block=True, timeout=None):
        if self.socket is None:
            raise ValueError('Forgot to call reset().')
        if not block:
            raise NotImplementedError

        r = self.socket.recv()
        msg = python_from_wire(r)

        timestamp = msg['timestamp']
        resolution = msg['resolution']
        framerate = msg['framerate_requested']
        data = msg['data']

        counter = msg['counter']
        
        age = time.time() - timestamp
        nmissed = (counter - self.last_counter) - 1

        if not nmissed >= 0:
            print('Detected new instance of camera driver.')

        if self.nreceived > 0:
            self.nmissed_total += nmissed
        self.last_counter = counter

        cur = time.time()
        fps = self.nreceived / (cur - self.first)

        if self.stats:
            from procgraph_optimize.utils import friendly_quantity
            if self.nreceived % 100 == 1:
                print(('nrec: %5d nmiss: %5d Fps: %8.2f' % (self.nreceived, self.nmissed_total, fps))
                       + ' age: %s' % friendly_quantity(age, 's')
                       )

        self.nreceived += 1
        
        if self.data_format == PICameraSource.F_JPG:
            res = data 
        elif self.data_format == PICameraSource.F_PIL:
            from cStringIO import StringIO
            from PIL import Image  # @UnresolvedImport
            res = Image.open(StringIO(data))
        elif self.data_format == PICameraSource.F_NUMPY:
            from cStringIO import StringIO  # @Reimport
            import numpy as np
            from PIL import Image  # @UnresolvedImport @Reimport
            im = Image.open(StringIO(data))
            res = np.array(im)
        else:
            assert False

        return timestamp, res


if __name__ == '__main__':
    address = "tcp://maryam.local:11223"
    s = PICameraSource(address, stats=True)
    s.reset()
    n = 0
    while True:
        timestamp, data = s.get()
#         age = friendly_quantity(time.time() - timestamp, 's')
#         if n % 100 == 1:
#             pass
        # print('Received n = %d (last age: %s)' % (n, age))
        n += 1
    
    


