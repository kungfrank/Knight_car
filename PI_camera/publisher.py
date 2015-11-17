import time
from serialization import wire_from_python

__all__ = [
    'picamera_zmq_publisher',
]

def picamera_zmq_publisher(resolution=(320, 240), framerate=80,
                           address='tcp://*:11223',
                           SNDHWM=1, warmup=2.0):

    import picamera  # @UnresolvedImport
    import zmq

    context = zmq.Context()
    sock = context.socket(zmq.PUB)  # @UndefinedVariable
    sock.setsockopt(zmq.SNDHWM, SNDHWM)  # @UndefinedVariable
    sock.bind(address)

    with picamera.PiCamera() as camera:
        camera.resolution = resolution
        camera.framerate = framerate
        print('Warming up...')
        time.sleep(warmup)
        print('Capturing...')
        gen = outputs(sock, stats=True, block=True)
        camera.capture_sequence(gen, 'jpeg', use_video_port=True)


def outputs(sock, stats=True, block=True):
    import io
    stream = io.BytesIO()
    first = time.time()
    last = time.time()
    n = 0

    while True:
        yield stream

        stream.seek(0)
        data = stream.read()
        msg = dict(data=data, timestamp=time.time(), resolution=resolution,
                   framerate_requested=framerate, counter=n)

        m = wire_from_python(msg)

        if block:
            sock.send(m)
        else:
            sock.send(m, zmq.NOBLOCK)  # @UndefinedVariable
        cur = time.time()
        if stats and (n % 100 == 1):
            delta = cur - last
            fps = n / (cur - first)
            print("Avg: %8.2fps instantaneous throughput: %s = %s Hz" %
                  (fps, delta, 1 / delta))

        stream.seek(0)
        stream.truncate()
        last = cur
        n += 1

if __name__ == '__main__':
    resolution = (320, 240)
    framerate = 80
    framerate = 10
    address = 'tcp://*:11223'
    SNDHWM = 1
    picamera_zmq_publisher(resolution, framerate, address, SNDHWM)
