import os
import yaml
import numpy as np
import time, timeit

def timer(func):
    def wrapper(*args, **kwargs):
        t0 = timeit.default_timer()
        result = func(*args, **kwargs)
        elapsed = timeit.default_timer() - t0
        # name = func.__name__
        # arg_str = ', '.join(repr(arg) for arg in args)
        # print('[%0.8fs] %s(%s) -> %r' % (elapsed, name, arg_str, result))
        print('predict time: %0.8fs' % elapsed)
        return result
    return wrapper


def read_yaml(filename):
    with open(filename, 'r') as f:
        data = yaml.load(f)
    return data

def save_yaml(filename, data):
    with open(filename, 'w') as f:
        yaml.dump(data, f)

def interp2d(corners, r, c):
    if not isinstance(corners, np.ndarray):
        corners = np.array(corners)
    assert corners.shape[0] == 4

    x1 = corners[0]
    x2 = corners[1]
    x3 = corners[2]
    x4 = corners[3]

    y1 = x1 + (r/9.)*(x3-x1)
    y2 = x2 + (r/9.)*(x4-x2)
    z = y1 + (c/8.)*(y2-y1)

    return z.tolist()

def order_points(pts):
    if not isinstance(pts, np.ndarray):
        pts = np.array(pts, dtype=np.float32)

    polygon = np.zeros((4, 2), dtype=np.float32)

    # the top-left point will have the smallest sum, whereas
    # the bottom-right point will have the largest sum
    s = pts.sum(axis=1)
    polygon[0] = pts[np.argmin(s)]
    polygon[2] = pts[np.argmax(s)]

    # now, compute the difference between the points, the
    # top-right point will have the smallest difference,
    # whereas the bottom-left will have the largest difference
    diff = np.diff(pts, axis=1)
    polygon[1] = pts[np.argmin(diff)]
    polygon[3] = pts[np.argmax(diff)]

    # return the ordered coordinates
    return polygon.tolist()

def rc_to_code(r, c):
    code_str = 'abcdefghi'
    code = '{}{}'.format(code_str[c], r)
    return code

def round_int(x):
    return int(round(x))

def kill():
    os._exit(0)