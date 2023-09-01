from gen.protocol_pb2 import *
from receiver import msgtype_str

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
