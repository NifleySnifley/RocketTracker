import lib.proto.protocol_pb2 as protocol
from link import Datum, DatumTypeID, Frame
from ctypes import *
from collections import Counter
from io import BufferedReader
import base64
import pickle
from util import *
import json
import csv
import gpxpy
import gpxpy.gpx
import datetime


def getdict(struct):
    result = {}

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


def convert_bools(d: dict):
    return {k: (int(v) if isinstance(v, bool) else v) for k, v in d.items()}


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


LOGDATA_EVENT_BOOT = 1
LOGDATA_EVENT_LIFTOFF = 2
LOGDATA_EVENT_BURNOUT = 3
LOGDATA_EVENT_APOGEE = 4
LOGDATA_EVENT_LANDING = 5
LOGDATA_EVENT_STATE = 6
LOGEVENT2STR = {
    1: "BOOT",
    2: "LIFTOFF",
    3: "BURNOUT",
    4: "APOGEE",
    5: "LANDING",
    6: "STATE",
}
LOGEVENT2STR_HUMAN = {
    1: "Device boot",
    2: "Liftoff",
    3: "Engine burnout",
    4: "Apogee",
    5: "Landed",
    6: "Log rate",
}

LOGSTATE_LOGGING_STOPPED = 0
LOGSTATE_LOGGING_AUTO_ARMED = 1,
LOGSTATE_LOGGING_AUTO_LIFTOFF = 2,
LOGSTATE_LOGGING_AUTO_FLIGHT = 3,
LOGSTATE_LOGGING_AUTO_LANDED = 4,
LOGSTATE_LOGGING_MANUAL_HZ = 5


class LogDataEventRaw(Structure):
    _fields_ = [
        ("event_type", c_uint8),
        ("argument", c_uint16),
    ]

    def parse_state_argument(self):
        rate_hz = (self.argument & 0b1111111111)
        state = (self.argument >> 10) & 0b111111
        return (state, rate_hz)

    def __str__(self) -> str:
        s = f"{LOGEVENT2STR_HUMAN.get(self.event_type, '<unknown>')}"
        if (self.event_type == LOGDATA_EVENT_STATE):
            s += f" {self.parse_state_argument()[1]}Hz"
        return s


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

        ("orientation_pitch", c_float),
        ("orientation_yaw", c_float),
        ("orientation_roll", c_float),

        ("lps_press_fresh", c_bool),
        ("lis_mag_fresh", c_bool),
        ("lsm_acc_fresh", c_bool),
        ("lsm_gyr_fresh", c_bool),
        ("adxl_acc_fresh", c_bool),
        ("gps_fresh", c_bool),
    ]

    def __init__(self, data: LogDataDefaultRaw):
        # hPa
        self.lps_press = data.lps_press_raw / 1048576.0
        # mG - sort-of
        self.lis_mag = [1000.0*(v/6842.0) for v in data.lis_mag_raw]
        # g
        self.lsm_acc = [0.001*(v * 0.488) for v in data.lsm_acc_raw]
        # deg/sec
        self.lsm_gyr = [0.001*(v * 70.0) for v in data.lsm_gyr_raw]
        # g
        self.adxl_acc = [v * 49 * 0.001 for v in data.adxl_acc_raw]

        self.orientation_quat = data.orientation_quat
        self.orientation_roll, self.orientation_pitch, self.orientation_yaw = quat2euler(
            self.orientation_quat)

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


class LogDataTextLog():
    def __init__(self) -> None:
        self.text = ""

    def from_buffer_copy(buffer: bytes):
        s = LogDataTextLog()
        s.text = buffer.decode('ascii')
        return s

    def __repr__(self) -> str:
        return self.text


LOG_FRAMETYPE_TO_CLASS = {
    0: LogDataDefaultRaw,
    2: LogDataEventRaw,
    3: LogDataTextLog
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
            print(f"Unknown frametype")
            self.data = None

    def __str__(self) -> str:
        return f"{self.data} @ T{self.timestamp:+.2f}s"

# TODO: Log data resampling from freshness!!!


class RTRKLog():
    def __init__(self, frames=[]):
        self.frames = frames
        self.raw_size_bytes = 0
        self.data = bytes()

    def load_raw_pickle(self, filename):
        segments = []
        if not isinstance(filename, str):
            segments = pickle.load(filename)
        else:
            with open(filename, 'rb') as f:
                segments = pickle.load(f)

        self.load_segments(segments)

    def load_raw_binary(self, filename):
        data = None
        if not isinstance(filename, str):
            data = filename.read()
        else:
            with open(filename, 'rb') as f:
                data = f.read()

        self.load_data(data)

    def load_file_auto(self, filename):
        try:
            self.load_raw_pickle(filename)
        except Exception:
            self.load_raw_binary(filename)

    def load_data(self, data=None):
        if data is None:
            data = self.data
        else:
            self.data = data

        self.raw_size_bytes = len(data)
        self.frames = [
            LogFrame(log_cobs_decode(f))
            for f in data.split(b'\xFF')
        ]
        self.frames = [f for f in self.frames if f.valid]

    def load_segments(self, segments: list) -> bool:
        segments, ack = segments[:-1], segments[-1]
        crc_xor = ack['log_crc16']

        xor = 0
        for s in segments:
            xor ^= s['segment_crc16']
        if (xor != crc_xor):
            # CRC Fail!
            return False

        self.data = bytes()
        for s in segments:
            ddec = base64.b64decode(s['data'])
            self.data += ddec

        self.load_data()

    def split_logs(self):
        frame_buckets = [[]]

        def split():
            print("sp")
            frame_buckets.append([])

        clr = 0
        for f in self.frames:
            if isinstance(f.data, LogDataEventRaw):
                # print(LOGEVENT2STR[f.data.event_type], f.data.argument)
                ev = f.data

                if ev.event_type == LOGDATA_EVENT_BOOT:
                    split()
                    clr = 0
                elif ev.event_type == LOGDATA_EVENT_STATE:
                    state, hz = ev.parse_state_argument()
                    if clr != 0 and state in [LOGSTATE_LOGGING_STOPPED]:
                        split()
                    clr = hz

            frame_buckets[-1].append(f)

        logs_out = [RTRKLog(fb) for fb in frame_buckets if len(fb)]
        for l in logs_out:
            l.sort()
        return logs_out

    def normalize(self):
        t0 = min([f.timestamp for f in self.frames])
        liftoffs = [
            f for f in self.frames if
            (isinstance(f.data, LogDataEventRaw)
             and f.data.event_type == LOGDATA_EVENT_LIFTOFF)
        ]
        if len(liftoffs) == 1:
            # print('lo!')
            t0 = liftoffs[0].timestamp

        # print(f"t0 = {t0}")
        for f in self.frames:
            f.timestamp -= t0

    def sort(self):
        self.frames.sort(key=lambda f: f.timestamp)

    def calibrate(self, calibfilepath):
        cjson = {}
        f = None
        if isinstance(calibfilepath, str):
            f = open(calibfilepath, 'r')
        else:
            f = calibfilepath

        cjson = dict2config(json.load(f))
        f.flush()
        f.close()

        for f in self.frames:
            if callable(getattr(f.data, 'calibrate', None)):
                # print("Calib")
                f.data.calibrate(cjson)

    def export_json(self, filename):
        dv = [{"time_seconds": f.timestamp} |
              getdict(f.data) for f in self.frames]

        f = filename
        if isinstance(filename, str):
            f = open(filename, 'w', newline='', encoding='utf-8')
        json.dump(dv, f, indent=4)
        f.flush()
        f.close()

    def export_csv(self, filename):
        dictrep = [{"time_seconds": f.timestamp} |
                   convert_bools(split_dict_vectors(getdict(f.data))) for f in self.frames]
        allkeys = set().union(*(d.keys() for d in dictrep))
        allkeys = list(sorted(allkeys))

        f = filename
        if isinstance(filename, str):
            f = open(filename, 'w', newline='', encoding='utf-8')

        cw = csv.DictWriter(f, allkeys)
        cw.writeheader()
        cw.writerows(dictrep)

        f.flush()
        f.close()

    def export_gpx(self, filename):
        fresh_gps = [f for f in self.frames if isinstance(
            f.data, LogDataDefault) and f.data.gps_fresh]

        gpx = gpxpy.gpx.GPX()
        gpx_track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(gpx_track)
        gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)

        def log2gpxpoint(frame: LogFrame):
            data: LogDataDefault = frame.data
            return gpxpy.gpx.GPXTrackPoint(latitude=data.gps_lat, longitude=data.gps_lon, elevation=data.gps_alt, time=datetime.datetime.fromtimestamp(frame.timestamp))
        gpx_segment.points += [log2gpxpoint(f) for f in fresh_gps]

        if isinstance(filename, str):
            with open(filename, 'w') as f:
                f.write(gpx.to_xml())
        else:
            filename.write(gpx.to_xml())

    def get_log_duration_sec(self):
        tss = [f.timestamp for f in self.frames]
        return (max(tss) - min(tss))

    def get_events(self):
        return [f for f in self.frames if isinstance(f.data, LogDataEventRaw)]


if __name__ == "__main__":
    log = RTRKLog()
    log.load_raw_pickle("./logs/gps_walk_events.pickle")
    log.calibrate("./logs/tracker2.config.json")

    individual_logs = log.split_logs()
    print(f"Contains {len(individual_logs)} individual logs")
    individual_logs[0].export_gpx("./logs/gps_walk_events.gpx")
