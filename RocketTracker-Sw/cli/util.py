import math
import numpy as np
import lib.proto.protocol_pb2 as protocol
import argparse


def quat2euler(Q):
    w, x, y, z = Q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return np.rad2deg(roll_x), np.rad2deg(pitch_y), np.rad2deg(yaw_z)


def vector_print_just(vector):
    return ','.join([f"{v:9.4f}" for v in vector])


def calculate_2_pt_calibration(reference_a, value_a, reference_b, value_b):
    # reference_a = (value_a+offset)*scale
    # reference_b = (value_b+offset)*scale

    offset = (reference_a*value_b - reference_b*value_a) / \
        (reference_b - reference_a)

    scalar = 1.0
    if (value_a + offset != 0):
        scalar = reference_a/(value_a+offset)
    else:
        scalar = reference_b/(value_b+offset)

    return (offset, scalar)


def apply_2_pt_calibration(value, calib):
    offset, scalar = calib
    return (value + offset) * scalar


def configjson_get_2_pt_calibration(configjson, basekey, noscale=False):
    kscale, koffset = basekey+'_scale', basekey+'_offset'
    if noscale:
        return (configjson[koffset], 1.0)
    else:
        return (configjson[koffset], configjson[kscale])


def configjson_apply_single_calibration(configjson, basekey, value, noscale=False):
    return apply_2_pt_calibration(value, configjson_get_2_pt_calibration(configjson, basekey, noscale=noscale))


def configjson_apply_3D_calibration(configjson, basekey, value, noscale=False):
    out = value.copy()
    for i, ax in [(0, 'x'), (1, 'y'), (2, 'z')]:
        out[i] = configjson_apply_single_calibration(
            configjson, f"{basekey}.{ax}", value[i], noscale=noscale)
    return out


def config2dict(flat_dict: dict) -> dict:
    d = {}
    for k, v in flat_dict.items():
        path = k.split('.')[1:]

        cdp = d
        for sp in path[:-1]:
            if sp not in cdp:
                cdp[sp] = {}
            cdp = cdp[sp]

        cdp[path[-1]] = v

    return d


def dict2config(recursive_dict: dict):
    cfg_out = {}

    def worker(d, kp=[]):
        if isinstance(d, dict):
            for k, v in d.items():
                kp.append(k)
                worker(d[k], kp)
                kp.pop()
        else:
            cfg_out['config.' + '.'.join(kp)] = d

    worker(recursive_dict, kp=[])
    return cfg_out


def argparse_restricted_float(mi, ma):
    def worker(x):
        try:
            x = float(x)
        except ValueError:
            raise argparse.ArgumentTypeError(
                "%r is not a floating-point literal" % (x,))
        if x < mi or x > ma:
            raise argparse.ArgumentTypeError(
                f"{x} not in range [{mi}, {ma}]")
        return x
    return worker


def argparse_restricted_int(mi: int, ma: int):
    def worker(x):
        try:
            x = int(x)
        except ValueError:
            raise argparse.ArgumentTypeError(
                "%r is not a int literal" % (x,))
        if x < mi or x > ma:
            raise argparse.ArgumentTypeError(
                f"{x} not in range [{mi}, {ma}]")
        return x
    return worker


LOGGER_COBS_ESC = 0xAA
LOGGER_COBS_ESC_AA = 0x01
LOGGER_COBS_ESC_FF = 0x02


def log_cobs_decode(data: bytes) -> bytes:
    dout = bytes()
    diter = iter(data)
    while (d := next(diter, None)) is not None:
        if (d == LOGGER_COBS_ESC):
            dout += bytes([LOGGER_COBS_ESC if next(diter, None)
                          == LOGGER_COBS_ESC_AA else 0xFF])
        else:
            dout += bytes([d])
    return dout
