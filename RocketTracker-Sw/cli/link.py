from typing import List
from lib.proto.protocol_pb2 import *
from google.protobuf.json_format import MessageToJson, ParseDict, MessageToDict
from lib.proto.protocol_pb2 import DatumTypeID, LinkID, TalkerID
import lib.proto.protocol_pb2 as protocol
from lib.comms_lib.crctabgen import crc16

# define USB_SERIAL_BUF_SIZE 1024*5
# define USB_SER_ESC 0xFF
# define USB_SER_ESC_ESC 0x01
# define USB_SER_ESC_NULL 0x02
# define USB_TIMEOUT_MS 500

COBS_ESC = 0xFF
COBS_ESC_ESC = 0x01
COBS_ESC_00 = 0x02


def cobs_encode(data: bytes) -> bytes:
    out = bytes()
    for b in data:
        if b == COBS_ESC:
            out += bytes([COBS_ESC, COBS_ESC_ESC])
        elif b == 0x00:
            out += bytes([COBS_ESC, COBS_ESC_00])
        else:
            out += bytes([b])
    return out


def cobs_decode(data: bytes) -> bytes:
    dout = bytes()
    diter = iter(data)
    while (d := next(diter, None)) is not None:
        if (d == COBS_ESC):
            dout += bytes([COBS_ESC if next(diter, None)
                          == COBS_ESC_ESC else 0x00])
        else:
            dout += bytes([d])
    return dout


DATUMTYPEID_TO_CLASS = {
    INFO_Blank: None,
    INFO_Raw: None,
    INFO_Battery: Battery,
    INFO_GPS: GPS,
    INFO_Altitude: Altitude,
    INFO_Orientation: Orientation,
    INFO_Alert: Alert,
    INFO_LogStatus: LogStatus,

    STATUS_RadioRxStatus: RadioRxStatus,

    CMD_Ping: Command_Ping,
    RESP_Ping: Resp_Ping,

    CMD_ConfigureLogging: Command_ConfigureLogging,
    RESP_ConfigureLogging: Resp_BasicError,

    CMD_EraseLog: Command_EraseLog,
    RESP_EraseLog: Resp_BasicError,

    CMD_DownloadLog: None,  # IMPLEMENT!!!!
    RESP_DownloadLog: Resp_BasicError,  # IMPLEMENT!!!!
    RESP_DownloadLog_Segment: Resp_DownloadLog_Segment,
    ACK_Download_Complete: Acknowledgement_Download_Complete,
    # ACK_DownloadLog_Segment: None,  # IMPLEMENT!!!!

    CMD_LogStatus: LogStatus,
}


class Datum:
    def __init__(self, typeid: DatumTypeID = None, data=None) -> None:
        self.typeid = typeid if (typeid is not None) else protocol.INFO_Blank
        self.length = 0
        self.data = bytes()
        if (data is not None):
            self.set_data(data)

    def set_data(self, data: bytes):
        self.length = len(data)
        self.data = data

    def get_bytes(self) -> bytes:
        out = bytes()
        # 8-bit DatumTypeID
        out += int.to_bytes(int(self.typeid), 1, byteorder='little')
        # 16-bit length
        out += int.to_bytes(len(self.data), 2, byteorder='little')
        # Payload
        out += self.data

        return out

    def load_bytes(self, data: bytes) -> int:
        i = 0
        self.typeid = int.from_bytes(data[i:i+1], byteorder='little')
        i += 1
        self.length = int.from_bytes(data[i:i+2], byteorder='little')
        i += 2
        self.data = data[i:i+self.length]
        i += self.length
        return i

    def __repr__(self):
        d = self.to_dict()
        if d is not None:
            return f"{DatumTypeID.DESCRIPTOR.values_by_number[self.typeid].name} ({self.length}): {d}"
        else:
            return f"{DatumTypeID.DESCRIPTOR.values_by_number[self.typeid].name} ({self.length}): {self.data.hex(':')}"

    def load_protobuf(self, protostruct):
        self.set_data(protostruct.SerializeToString())

    def to_protobuf(self):
        _class = DATUMTYPEID_TO_CLASS[self.typeid]
        if _class is None:
            return None

        _obj = _class()
        _obj.ParseFromString(self.data)
        return _obj

    def to_dict(self):
        p = self.to_protobuf()
        return MessageToDict(self.to_protobuf(), preserving_proto_field_name=True, use_integers_for_enums=True) if p is not None else None


# Basically fmgr-lite
class Frame:
    def __init__(self, encoded=None) -> None:
        self.id = protocol.Basestation
        self.crc = None
        self.datums: List[Datum] = []

        if (encoded is not None):
            self.load_from_bytes(encoded)

    def add_datum(self, datum: Datum):
        self.datums.append(datum)

    def get_payload_bytes(self):
        data = bytes()
        for d in self.datums:
            data += d.get_bytes()
        return data

    def get_bytes(self) -> bytes:
        encoded = bytes()
        data = self.get_payload_bytes()

        # 16-bit ID
        encoded += int.to_bytes(self.id, 2, byteorder='little')
        # 16-bit CRC
        encoded += int.to_bytes(crc16(data), 2, byteorder='little')
        # Data bytes
        encoded += data

        return encoded

    # Return true on success
    def load_from_bytes(self, data: bytes) -> bool:
        self.datums.clear()
        i = 0
        self.id = int.from_bytes(data[i:i+2], byteorder='little')
        i += 2
        self.crc = int.from_bytes(data[i:i+2], byteorder='little')
        i += 2

        while (i < len(data)):
            d = Datum()
            n = d.load_bytes(data[i:])
            self.datums.append(d)
            i += n

        return crc16(data) == self.crc
        # TODO: Check CRC!!!
