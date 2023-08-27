import sys
from qtpy.QtWidgets import *
from qtpy.QtCore import *
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen.mainwindow import Ui_MainWindow
import gen.protocol_pb2
from receiver import Receiver

CONSOLE_SCROLLBACK = 256


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.map_tab: QWidget
        self.port_selector: QComboBox
        self.connect_btn: QPushButton
        self.console: QPlainTextEdit
        self.port_rfsh_btn: QToolButton
        self.avail_ports: list[QSerialPortInfo] = []
        self.setupUi(self)

        self.tlm_port = QSerialPort()
        self.vgps_port = QSerialPort()

        # TODO: This class handles decoding, make signal for messages!
        self.receiver = Receiver(
            self.tlm_port, self.vgps_port, self.rx_cfg_frf, self.rx_cfg_bw, self.rx_cfg_pow)

        self.mapWidget = MapWidget()
        self.maplayout = QVBoxLayout()
        self.maplayout.addWidget(self.mapWidget)
        self.maplayout.setContentsMargins(0, 0, 0, 0)

        # Working with the maps with pyqtlet
        self.map = L.map(self.mapWidget)
        self.map.setView([0, 0], 10)
        L.tileLayer(
            'https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Gt').addTo(self.map)
        self.marker = L.circleMarker(
            [0, 0], options={"fillOpacity": 1.0, "color": "#ff0000"})
        self.marker.bindPopup('Maps are a treasure.')
        self.map.addLayer(self.marker)

        self.map_tab.setLayout(self.maplayout)

        # Refresh button
        self.refreshports()
        self.port_rfsh_btn.clicked.connect(self.refreshports)
        self.connect_btn.clicked.connect(self.connectports)

    def print(self, *items, sep=' '):
        ls = self.console.toPlainText().splitlines()
        if (len(ls) > CONSOLE_SCROLLBACK):
            del ls[0]
        ls.append(sep.join([str(e) for e in items]))
        self.console.setPlainText("\n".join(ls))

    def find_vgps_port(self):
        tlm = self.port_selector.currentIndex()
        if (tlm < len(self.avail_ports) - 1):
            print(self.avail_ports[tlm+1].portName())
            return self.avail_ports[tlm+1]
        else:
            self.print(
                "Error: Could not connect to VGPS port\n    Be sure to select the lower numbered port")
            return None

    def connectports(self):
        if (self.tlm_port.isOpen()):
            self.tlm_port.close()
            self.vgps_port.close()
            self.connect_btn.setText("Connect")
            self.receiver.ondisconnect()
        else:
            self.tlm_port.setPort(
                self.avail_ports[self.port_selector.currentData()])
            gpp = self.find_vgps_port()
            if (gpp != None):
                self.vgps_port.setPort(gpp)
            else:
                self.tlm_port.close()
                self.vgps_port.close()
                return

            for p in [self.vgps_port, self.tlm_port]:
                p.setBaudRate(QSerialPort.BaudRate.Baud115200)
                p.setParity(QSerialPort.Parity.NoParity)
                p.setDataBits(QSerialPort.DataBits.Data8)
                p.setFlowControl(QSerialPort.FlowControl.NoFlowControl)
                p.setStopBits(QSerialPort.StopBits.OneStop)

            if (self.tlm_port.open(QIODevice.ReadWrite) and self.vgps_port.open(QIODevice.ReadWrite)):
                self.connect_btn.setText("Disconnect")
                self.print(
                    f"Connected to {self.tlm_port.portName()} and {self.vgps_port.portName()}")
                self.receiver.onconnect()

                # self.tlm_port.write(bytes([0x02, 0x00, 0x00]))
            else:
                self.tlm_port.disconnect()
                self.vgps_port.disconnect()

    def tlm_read(self):
        print(self.tlm_port.readAll())

    def refreshports(self):
        self.avail_ports = (sorted(
            QSerialPortInfo.availablePorts(), key=lambda p: p.portName()))
        self.port_selector.clear()
        for i, port in enumerate(self.avail_ports):
            self.port_selector.addItem(
                f"{port.portName()} - {port.description()}", userData=i)
            # self.print(port.portName(), port.productIdentifier(),
            #            port.description())


app = QApplication(['', '--no-sandbox'])

window = MainWindow()
window.show()
app.exec()
