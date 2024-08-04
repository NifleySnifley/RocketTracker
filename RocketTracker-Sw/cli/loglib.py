import lib.proto.protocol_pb2 as protocol
from link import Datum, DatumTypeID, Frame
from ctypes import *
from collections import Counter
import base64
import pickle

LOGGER_COBS_ESC = 0xAA
LOGGER_COBS_ESC_AA = 0x01
LOGGER_COBS_ESC_FF = 0x02


def cobs_decode(data: bytes) -> bytes:
    dout = bytes()
    diter = iter(data)
    while (d := next(diter, None)) is not None:
        if (d == LOGGER_COBS_ESC):
            dout += bytes([LOGGER_COBS_ESC if next(diter, None)
                          == LOGGER_COBS_ESC_AA else 0xFF])
        else:
            dout += bytes([d])
    return dout


def getdict(struct):
    result = {}
    # print struct

    def get_value(value):
        if (type(value) not in [int, float, bool]) and not bool(value):
            # it's a null pointer
            value = None
        elif hasattr(value, "_length_") and hasattr(value, "_type_"):
            # Probably an array
            # print value
            value = get_array(value)
        elif hasattr(value, "_fields_"):
            # Probably another struct
            value = getdict(value)
        return value

    def get_array(array):
        ar = []
        for value in array:
            value = get_value(value)
            ar.append(value)
        return ar
    for f in struct._fields_:
        field = f[0]
        value = getattr(struct, field)
        # if the type is not a primitive and it evaluates to False ...
        value = get_value(value)
        result[field] = value
    return result


LOG_DEFAULT_FLAG_PRESS_FRESH = (1 << 0)
LOG_DEFAULT_FLAG_MAG_FRESH = (1 << 1)
LOG_DEFAULT_FLAG_LSM_ACC_FRESH = (1 << 2)
LOG_DEFAULT_FLAG_LSM_GYR_FRESH = (1 << 3)
LOG_DEFAULT_FLAG_ADXL_ACC_FRESH = (1 << 4)
LOG_DEFAULT_FLAG_GPS_FRESH = (1 << 5)


class LogDataDefaultRaw(Structure):
    _fields_ = [
        ("lps_press_raw", c_uint32),
        ("lis_mag_raw", c_int16*3),
        ("lsm_acc_raw", c_int16*3),
        ("lsm_gyr_raw", c_int16*3),
        ("adxl_acc_raw", c_int16*3),
        ("orientation_quat", c_float*4),
        ("filtered_altitude_m", c_float),
        ("gps_lat", c_float),
        ("gps_lon", c_float),
        ("gps_alt", c_float),
        ("flags", c_uint8),
    ]


class LogDataDefault():
    def __init__(self, data: LogDataDefaultRaw):
        pass

    def apply_calibration(self, configuration_dict):
        pass


LOG_FRAMETYPE_TO_CLASS = {
    0: LogDataDefaultRaw,
}


class LogFrame():
    def __init__(self, data: bytes):
        if (len(data) < 10):
            self = None
            return

        self.timestamp = int.from_bytes(data[0:8], 'little') * 1e-6

        dtype = int.from_bytes(data[8:10], 'little')

        if (dtype in LOG_FRAMETYPE_TO_CLASS):
            self.data = LOG_FRAMETYPE_TO_CLASS[dtype].from_buffer_copy(
                data[10:])
        else:
            self.data = None


class RTRKLog():
    def __init__(self):
        self.frames = []

    def load_raw_pickle(self, filename: str) -> bool:
        segments = pickle.load(open("./format_v2_test.pickle", 'rb'))
        segments, ack = segments[:-1], segments[-1]
        crc_xor = ack['log_crc16']

        xor = 0

        for s in segments:
            xor ^= s['segment_crc16']
        if (xor != crc_xor):
            # CRC Fail!
            return False

        databuffer = bytes()
        for s in segments:
            databuffer += base64.b64decode(s['data'])

        self.frames = [
            LogFrame(cobs_decode(f))
            for f in databuffer.split(b'\xFF')
        ]
        # print(Counter([len(f) for f in frames_dec]))


if __name__ == "__main__":
    log = RTRKLog()
    log.load_raw_pickle("./gps_10hz.pickle")

    print(len(log.frames))
    print(getdict(log.frames[0].data))
