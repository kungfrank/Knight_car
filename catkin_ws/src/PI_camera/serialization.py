from msgpack import PackValueError
import msgpack_numpy as m  # @UnresolvedImport
m.patch()
import cPickle
import msgpack

__all__ = [
    'wire_from_python',
    'python_from_wire',
]


def wire_from_python(data):
    try:
        wire = msgpack.packb(data)  # , use_bin_type=True
    except (TypeError, PackValueError) as e:
        print('Falling back on pickle: %s' % e)
        # PackValueError: for recursive structures
        # print('falling back on pickle')
        data = ('__pickle', cPickle.dumps(data))
        wire = msgpack.packb(data)  # , use_bin_type=True

    return wire

def python_from_wire(data):
    resp = msgpack.unpackb(data, use_list=False)  # False: allows tuples
    # print('obtained %s ' % str(resp))
    if isinstance(resp, tuple) and len(resp) == 2 and resp[0] == '__pickle':
        resp = cPickle.loads(resp[1])

    return resp
