from qtpy.QtWidgets import *
from qtpy.QtCore import *
from typing import Deque
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen.protocol_pb2 import *
from google.protobuf.message import DecodeError
from util import has_all_fields, msgtype_str


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

    def __init__(self, tlm: QSerialPort, vgps: QSerialPort, frf: QDoubleSpinBox, bw: QComboBox, pow: QSpinBox, cprint, *args, **kwargs):
        super(Receiver, self).__init__(*args, **kwargs)

        self.printfn = cprint
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
        self.port_vgps.readyRead.connect(self.vgps_rx)

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
                    i = 2
                    crc = int.from_bytes(self.rx_buf[:2], 'little')
                    i += 2
                    # TODO: handle ID

                    # Parse Datums
                    while (i < len(self.rx_buf)):
                        dtype = self.rx_buf[i]
                        i += 1
                        slen = self.rx_buf[i]
                        i += 1
                        self.parse_datum(dtype,
                                         bytes(self.rx_buf[i:i+slen]))
                        i += slen
                    # Emit signal for datums

    def vgps_rx(self):
        # Detect and port swap if applicable
        bs = self.port_vgps.readAll().data()
        for b in bs:
            if b == 0x00:
                # There shouldn't be any null characters coming from the VGPS port!
                # The ports must be in the wrong order, so swap them
                self.port_tlm.readyRead.disconnect(self.tlm_rx)
                self.port_vgps.readyRead.disconnect(self.vgps_rx)

                tmp = self.port_vgps
                self.port_vgps = self.port_tlm
                self.port_tlm = tmp

                self.port_tlm.readyRead.connect(self.tlm_rx)
                self.port_vgps.readyRead.connect(self.vgps_rx)

    def parse_datum(self, t: MessageTypeID, data):
        if (msgtype_str(t) == None):
            # self.printfn(t)
            self.printfn("Error, malformed datum type!")
            return
        parsed: object = None
        try:
            if (t == TLM_GPS_Info):
                parsed = GPS_Info()
                parsed.ParseFromString(data)
            elif (t == RX_RadioStatus):
                parsed = Receiver_RadioStatus()
                parsed.ParseFromString(data)
            elif (t == TLM_Battery_Info):
                parsed = Battery_Info()
                parsed.ParseFromString(data)
            elif (t == TLM_Alert):
                parsed = Alert()
                parsed.ParseFromString(data)
            elif (t == TLM_Altitude_Info):
                parsed = Altitude_Info()
                parsed.ParseFromString(data)
            elif (t == TLM_Blank):
                parsed = None
            elif (t == TLM_Orientation_Info):
                parsed = Orientation_Info()
                parsed.ParseFromString(data)
            elif (t == TLM_Raw):
                parsed = Raw()
                parsed.ParseFromString(data)

        except DecodeError:
            self.printfn("Error, malformed datum protobuf!")
            return
        
        if (parsed == None):
            self.printfn("Unknown datum type! (None)")
            return
        
        # if (not has_all_fields(parsed)):
        #     self.printfn(f"Missing field! {msgtype_str(t)}")
        #     return 

        self.datumProcessed.emit(t, parsed)
