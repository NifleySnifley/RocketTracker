from gen.protocol_pb2 import *
from google.protobuf import message

MTMAP = {
    CMD_Ping: "CMD_Ping",
    RESP_Ping: "RESP_Ping",
    CMD_StartLog: "CMD_StartLog",
    RESP_StartLog: "RESP_StartLog",
    CMD_StopLog: "CMD_StopLog",
    RESP_StopLog: "RESP_StopLog",
    STATUS_RadioRxStatus: "RX_RadioStatus",
    INFO_Alert: "INFO_Alert",
    INFO_Altitude: "INFO_Altitude",
    INFO_Battery: "INFO_Battery",
    INFO_Blank: "INFO_Blank",
    INFO_GPS: "INFO_GPS",
    INFO_Orientation: "INFO_Orientation",
    INFO_Raw: "INFO_Raw",
}


def msgtype_str(t: DatumTypeID):
    try:
        return MTMAP[t]
    except KeyError:
        return None


def msgtype_fromstr(S: str):
    try:
        return [t for t, s in MTMAP.items() if s == S][0]
    except KeyError:
        return None


MTID_TO_TYPE = {
    msgtype_str(INFO_GPS): GPS,
    msgtype_str(INFO_Alert): Alert,
    msgtype_str(INFO_Altitude): Altitude,
    msgtype_str(INFO_Battery): Battery,
    msgtype_str(INFO_Orientation): Orientation,
    msgtype_str(INFO_Raw): Raw,
    msgtype_str(STATUS_RadioRxStatus): RadioRxStatus,
    msgtype_str(INFO_Blank): None
}


def has_all_fields(obj: message.Message):
    return all([obj.HasField(f) for f in obj.DESCRIPTOR.fields_by_name])
