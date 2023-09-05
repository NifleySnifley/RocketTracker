from gen.protocol_pb2 import *
from google.protobuf import message

MTMAP = {
    CMD_Ping: "CMD_Ping",
    CMD_StartLog: "CMD_StartLog",
    CMD_StopLog: "CMD_StopLog",
    RX_RadioStatus: "RX_RadioStatus",
    TLM_Alert: "TLM_Alert",
    TLM_Altitude_Info: "TLM_Altitude_Info",
    TLM_Battery_Info: "TLM_Battery_Info",
    TLM_Blank: "TLM_Blank",
    TLM_GPS_Info: "TLM_GPS_Info",
    TLM_Orientation_Info: "TLM_Orientation_Info",
    TLM_Raw: "TLM_Raw",
}


def msgtype_str(t: MessageTypeID):
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
    msgtype_str(TLM_GPS_Info): GPS_Info,
    msgtype_str(TLM_Alert): Alert,
    msgtype_str(TLM_Altitude_Info): Altitude_Info,
    msgtype_str(TLM_Battery_Info): Battery_Info,
    msgtype_str(TLM_Orientation_Info): Orientation_Info,
    msgtype_str(TLM_Raw): Raw,
    msgtype_str(RX_RadioStatus): Receiver_RadioStatus,
    msgtype_str(TLM_Blank): None
}


def battery_percent(voltage: float):
    return (voltage/5.0) * 100


def has_all_fields(obj: message.Message):
    return all([obj.HasField(f) for f in obj.DESCRIPTOR.fields_by_name])
