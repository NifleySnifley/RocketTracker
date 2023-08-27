from qtpy.QtWidgets import *
from qtpy.QtCore import *
from typing import Deque
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen import protocol_pb2

RADIOCFG_DEFAULT = {
    "FRF": 915,
    "SF": 7,
    "CR": 5,
    "POW": 20,
    "BW": 125
}  # TODO: Have a config file for the app so stuff like this is persistent...


class Receiver(QWidget):
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

        self.msg_queue = Deque()
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

    def send_config(self, param: str, value: str):
        txt = f"$P{param.upper()},{value}\r\n"
        if (self.port_vgps.isOpen()):
            self.port_vgps.write(
                bytes(txt, 'ascii'))

    def set_frf(self):
        self.send_config("FRF", "{:.2f}".format(self.frf_inp.value()))

    def set_bw(self):
        self.send_config("BW", self.bw_inp.currentText()[:-3])

    def set_pow(self):
        self.send_config("POW", str(self.pow_inp.value()))

    def tlm_rx(self):
        bs = self.port_tlm.readAll().data()
        for b in bs:
            if (self.rx_len == 0):
                if (len(self.rx_buf) > 0):
                    # self.msg_queue.appendleft(self.rx_buf.copy())
                    print([hex(e) for e in self.rx_buf])
                    print("msgrx")
                self.rx_len = int(b)
                self.rx_buf.clear()
            else:
                self.rx_buf.append(b)
                self.rx_len -= 1
        pass
