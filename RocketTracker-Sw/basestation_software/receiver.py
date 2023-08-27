from qtpy.QtWidgets import *
from qtpy.QtCore import *
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo

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

        self.frf_inp.valueChanged.connect(self.set_frf)
        self.bw_inp.currentTextChanged.connect(self.set_bw)
        self.pow_inp.valueChanged.connect(self.set_pow)

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
