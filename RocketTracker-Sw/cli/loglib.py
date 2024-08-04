import lib.proto.protocol_pb2 as protocol
from link import Datum, DatumTypeID, Frame
from ctypes import *
from collections import Counter
import base64
import pickle
from util import *
import json
import csv

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


def split_dict_vectors(d: dict):
    outd = {}
    for k, v in d.items():
        if isinstance(v, list):
            if (len(v) == 3):
                # XYZ Vector
                for i, ax in enumerate(list("xyz")):
                    outd[f"{k}:{ax}"] = v[i]
            elif (len(v) == 4):
                # Quaternion
                for i, ax in enumerate(list("wxyz")):
                    outd[f"{k}:{ax}"] = v[i]
        else:
            outd[k] = v
    return outd


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
    _fields_ = [
        ("lps_press", c_float),
        ("lis_mag", c_float*3),
        ("lsm_acc", c_float*3),
        ("lsm_gyr", c_float*3),
        ("adxl_acc", c_float*3),
        ("orientation_quat", c_float*4),
        ("filtered_altitude_m", c_float),
        ("gps_lat", c_float),
        ("gps_lon", c_float),
        ("gps_alt", c_float),

        ("lps_press_fresh", c_bool),
        ("lis_mag_fresh", c_bool),
        ("lsm_acc_fresh", c_bool),
        ("lsm_gyr_fresh", c_bool),
        ("adxl_acc_fresh", c_bool),
        ("gps_fresh", c_bool),
    ]

    def __init__(self, data: LogDataDefaultRaw):
        self.lps_press = data.lps_press_raw / 1048576.0
        self.lis_mag = [1000.0*(v/6842.0) for v in data.lis_mag_raw]
        self.lsm_acc = [0.001*(v * 0.488) for v in data.lsm_acc_raw]
        self.lsm_gyr = [0.001*(v * 70.0) for v in data.lsm_gyr_raw]
        self.adxl_acc = [v * 49 * 0.001 for v in data.adxl_acc_raw]
        self.orientation_quat = data.orientation_quat
        self.filtered_altitude_m = data.filtered_altitude_m
        self.gps_lat = data.gps_lat
        self.gps_lon = data.gps_lon
        self.gps_alt = data.gps_alt
        # self.flags = data.flags
        self.lps_press_fresh = bool(data.flags & LOG_DEFAULT_FLAG_PRESS_FRESH)
        self.lis_mag_fresh = bool(data.flags & LOG_DEFAULT_FLAG_MAG_FRESH)
        self.lsm_acc_fresh = bool(data.flags & LOG_DEFAULT_FLAG_LSM_ACC_FRESH)
        self.lsm_gyr_fresh = bool(data.flags & LOG_DEFAULT_FLAG_LSM_GYR_FRESH)
        self.adxl_acc_fresh = bool(
            data.flags & LOG_DEFAULT_FLAG_ADXL_ACC_FRESH)
        self.gps_fresh = bool(data.flags & LOG_DEFAULT_FLAG_GPS_FRESH)
        pass

    def calibrate(self, config: str | dict):
        cjson = {}

        if isinstance(config, dict):
            cjson = config
        else:
            with open(config, 'r') as f:
                cjson = dict2config(json.load(f))

        self.lps_press = self.lps_press + \
            cjson['config.calibration.lps22.offset']

        self.lis_mag = configjson_apply_3D_calibration(
            cjson, "config.calibration.lis3mdl", self.lis_mag)

        self.lsm_acc = configjson_apply_3D_calibration(
            cjson, "config.calibration.lsm6dsm.accel", self.lsm_acc)

        self.lsm_gyr = configjson_apply_3D_calibration(
            cjson, "config.calibration.lsm6dsm.gyro", self.lsm_gyr, noscale=True)

        self.adxl_acc = configjson_apply_3D_calibration(
            cjson, "config.calibration.adxl375", self.adxl_acc)


LOG_FRAMETYPE_TO_CLASS = {
    0: LogDataDefaultRaw,
}


class LogFrame():
    def __init__(self, data: bytes):
        if (len(data) < 10):
            self.valid = False
            return

        self.valid = True
        self.timestamp = int.from_bytes(data[0:8], 'little') * 1e-6

        dtype = int.from_bytes(data[8:10], 'little')

        if (dtype in LOG_FRAMETYPE_TO_CLASS):
            self.data = LOG_FRAMETYPE_TO_CLASS[dtype].from_buffer_copy(
                data[10:])
            if isinstance(self.data, LogDataDefaultRaw):
                self.data = LogDataDefault(self.data)
        else:
            self.data = None


class RTRKLog():
    def __init__(self):
        self.frames = []

    def load_raw_pickle(self, filename: str) -> bool:
        segments = []
        with open(filename, 'rb') as f:
            segments = pickle.load(f)
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
        self.frames = [f for f in self.frames if f.valid]
        self.frames.sort(key=lambda f: f.timestamp)
        # print(Counter([len(f) for f in frames_dec]))

    def calibrate(self, calibfilepath):
        cjson = {}
        with open(calibfilepath, 'r') as f:
            cjson = dict2config(json.load(f))
        for f in self.frames:
            if callable(getattr(f.data, 'calibrate', None)):
                # print("Calib")
                f.data.calibrate(cjson)

    def export_json(self, filename):
        dv = [getdict(f) for f in self.frames]

        with open(filename, 'w') as f:
            json.dump(dv, f)
            f.flush()

    def export_csv(self, filename):
        dictrep = [{"time_seconds": f.timestamp} |
                   split_dict_vectors(getdict(f.data)) for f in self.frames]
        allkeys = set().union(*(d.keys() for d in dictrep))

        with open(filename, 'w', newline='', encoding='utf-8') as f:
            cw = csv.DictWriter(f, allkeys)
            cw.writeheader()
            cw.writerows(dictrep)


if __name__ == "__main__":
    log = RTRKLog()
    log.load_raw_pickle("./logs/gps_10hz.pickle")
    print(getdict(log.frames[0].data))

    log.calibrate("./logs/tracker2.config.json")
    # log.export_csv("./logs/gps_10hz.csv")
