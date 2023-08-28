from qtpy.QtWidgets import *
from qtpy.QtCore import *
from typing import Deque
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen.protocol_pb2 import *


def msgtype_str(t: MessageTypeID):
    return {
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
    }[t]


def alert_str(t: AlertType):
    return {
        ALT_Apogee: "ALT_Apogee",
        ALT_Error: "ALT_Error",
        ALT_Landed: "ALT_Landed",
        ALT_Liftoff: "ALT_Liftoff",
        ALT_Log50: "ALT_Log50",
        ALT_LogFull: "ALT_LogFull",
        ALT_LowBatt: "ALT_LowBatt",
    }[t]


RADIOCFG_DEFAULT = {
    "FRF": 915,
    "SF": 7,
    "CR": 5,
    "POW": 20,
    "BW": 125
}  # TODO: Have a config file for the app so stuff like this is persistent...


class Receiver(QWidget):
    datumProcessed = Signal(int, object)

    def __init__(self, tlm: QSerialPort, vgps: QSerialPort, frf: QDoubleSpinBox, bw: QComboBox, pow: QSpinBox, *args, **kwargs):
        super(Receiver, self).__init__(*args, **kwargs)

        self.port_vgps = vgps
        self.port_tlm = tlm
        self.frf_inp = frf
        self.bw_inp = bw
        self.pow_inp = pow

        self.frf_inp.setDisabled(True)
        self.bw_inp.setDisabled(True)
        self.pow_inp.setDisabled(True)

        self.rx_buf = []
        self.rx_len = 0

        self.frf_inp.valueChanged.connect(self.set_frf)
        self.bw_inp.currentTextChanged.connect(self.set_bw)
        self.pow_inp.valueChanged.connect(self.set_pow)

        self.port_tlm.readyRead.connect(self.tlm_rx)

    def onconnect(self):
        self.frf_inp.setDisabled(False)
        self.bw_inp.setDisabled(False)
        self.pow_inp.setDisabled(False)
        pass

    def ondisconnect(self):
        self.frf_inp.setDisabled(True)
        self.bw_inp.setDisabled(True)
        self.pow_inp.setDisabled(True)
        pass

    # TODO: Check the thing
    def send_config(self, param: str, value: str):
        txt = f"$P{param.upper()},{value}\n"
        if (self.port_vgps.isOpen()):
            self.port_vgps.write(
                bytes(txt, 'ascii'))
            print(txt)

    def set_frf(self):
        self.send_config("FRF", "{:.2f}".format(self.frf_inp.value()))

    def set_bw(self):
        self.send_config("BW", self.bw_inp.currentText()[:-3])

    def set_pow(self):
        self.send_config("POW", str(self.pow_inp.value()))

    def tlm_rx(self):
        bs = self.port_tlm.readAll().data()
        bval = 0x00

        biter = iter(bs)
        for b in biter:
            # 0x00 resets frame (delimiter)
            if (b == 0x00):
                # print("RESET")
                self.rx_buf.clear()
                self.rx_len == 0
                continue
            elif (b == 0xFF):
                bval = 0 if next(biter) == 0x02 else 0xFF
            else:
                bval = b

            # print(hex(bval))

            # Get length
            if (self.rx_len == 0):
                if (len(self.rx_buf) == 0):
                    self.rx_len = int(bval)
                    self.rx_buf.clear()
            else:
                self.rx_buf.append(bval)
                self.rx_len -= 1

                if (self.rx_len == 0):
                    # self.msg_queue.appendleft(self.rx_buf.copy())
                    # print(f"Msg Rx: {len(self.rx_buf)}")
                    # print(", ".join([hex(e) for e in self.rx_buf]))

                    # Get ID and sort
                    id = int.from_bytes(self.rx_buf[:2], 'little')
                    # TODO: handle ID

                    # Parse Datums
                    i = 2
                    while (i < len(self.rx_buf)):
                        dtype = self.rx_buf[i]
                        i += 1
                        slen = self.rx_buf[i]
                        i += 1
                        self.parse_datum(dtype,
                                         bytes(self.rx_buf[i:i+slen]))
                        i += slen
                    # Emit signal for datums

    def parse_datum(self, t: MessageTypeID, data):
        # print(f"Datum type {msgtype_str(t)} with {len(data)} bytes.")
        parsed: object = None
        if (t == TLM_GPS_Info):
            parsed = GPS_Info()
            parsed.ParseFromString(data)
        elif (t == RX_RadioStatus):
            parsed = Receiver_RadioStatus()
            parsed.ParseFromString(data)

        self.datumProcessed.emit(t, parsed)
