
import os
import sys
from io import TextIOWrapper
from qtpy.QtWidgets import *
from qtpy.QtGui import QPalette, QColor
from qtpy.QtCore import *
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen.mainwindow import Ui_MainWindow
from gen.protocol_pb2 import *
from google.protobuf.json_format import MessageToJson
from util import battery_percent, MTID_TO_TYPE
from receiver import Receiver, msgtype_str
from typing import Deque
import json

CONSOLE_SCROLLBACK = 256


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.battery_bar: QProgressBar
        self.battery_voltage: QLabel
        self.rssi_disp: QLCDNumber
        self.snr_disp: QLCDNumber
        self.map_tab: QWidget
        self.port_selector: QComboBox
        self.connect_btn: QPushButton
        self.console: QPlainTextEdit
        self.port_rfsh_btn: QToolButton
        self.avail_ports: list[QSerialPortInfo] = []
        self.setupUi(self)

        self.logfile: TextIOWrapper = open("tracker.log", 'w')

        self.tlm_port = QSerialPort()
        self.vgps_port = QSerialPort()

        palette = self.rssi_disp.palette()

        # foreground color
        palette.setColor(palette.WindowText, QColor(85, 85, 255))

        self.rssi_disp.setPalette(palette)
        self.snr_disp.setPalette(palette)

        # TODO: This class handles decoding, make signal for messages!
        self.receiver = Receiver(
            self.tlm_port, self.vgps_port, self.rx_cfg_frf, self.rx_cfg_bw, self.rx_cfg_pow, self.print)

        self.gps_track = Deque()

        if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
            # self.print('running in a PyInstaller bundle')
            self.mapWidget = MapWidget(
                use_file_absolute_path=False, alternative_base_path=os.path.abspath(os.path.dirname(__file__)))
        else:
            # self.print('running in a normal Python process')
            self.mapWidget = MapWidget()
        print(os.path.abspath(os.path.dirname(__file__)))
        self.maplayout = QVBoxLayout()
        self.maplayout.addWidget(self.mapWidget)
        self.maplayout.setContentsMargins(0, 0, 0, 0)

        # Working with the maps with pyqtlet
        self.map = L.map(self.mapWidget)
        self.map.setView([0, 0], 10)
        L.tileLayer(
            'https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Gt').addTo(self.map)
        self.marker = L.marker(
            [0, 0])
        self.map_track_line = L.polyline(
            [], {"color": 'lightblue'}).addTo(self.map)

        self.map_tab.setLayout(self.maplayout)

        # Refresh button
        self.refreshports()
        self.port_rfsh_btn.clicked.connect(self.refreshports)
        self.connect_btn.clicked.connect(self.connectports)
        self.receiver.datumProcessed.connect(self.datum_cb)

    def datum_cb(self, t: MessageTypeID, datum: object):
        if (self.logfile != None and not self.logfile.closed):
            try:
                vtype = (MTID_TO_TYPE[msgtype_str(t)])
                if (vtype != None):
                    cls: vtype = datum
                    self.logfile.write(json.dumps({
                        "type": msgtype_str(t),
                        "datum": json.loads(MessageToJson(cls, preserving_proto_field_name=True))
                    }) + '\n')
                    self.logfile.flush()
            except Exception:
                self.print("Logging error!")

        print(f"Type: {msgtype_str(t)}")

        if (t == TLM_GPS_Info):
            datum: GPS_Info = datum

            self.gps_track.append((datum.lat, datum.lon))
            if (len(self.gps_track) > 4096):
                self.gps_track.popLeft()

            self.map_track_line.latLngs = [[a, b] for a, b in self.gps_track]
            self.map_track_line.removeFrom(self.map).addTo(self.map)

            self.marker.latLng = [datum.lat, datum.lon]
            self.marker.removeFrom(self.map).addTo(self.map)

            # if ()
            self.map.flyTo([datum.lat, datum.lon], 18)

        if (t == RX_RadioStatus):
            datum: Receiver_RadioStatus

            self.rssi_disp.display(datum.RSSI)
            self.snr_disp.display(datum.SNR)

        if (t == TLM_Battery_Info):
            datum: Battery_Info
            self.battery_voltage.setText(f"({datum.battery_voltage})")
            self.battery_bar.setValue(
                int(battery_percent(datum.battery_voltage)))

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
                self.tlm_port.clear()
                self.vgps_port.clear()

                self.receiver.onconnect()

                # self.tlm_port.write(bytes([0x02, 0x00, 0x00]))
            else:
                self.tlm_port.disconnect()
                self.vgps_port.disconnect()

    def refreshports(self):
        self.avail_ports = [p for p in (sorted(
            QSerialPortInfo.availablePorts(), key=lambda p: p.portName())) if p.vendorIdentifier() == 0xCafe]
        self.port_selector.clear()
        for i, port in enumerate(self.avail_ports):
            self.port_selector.addItem(
                f"{port.portName()} - {port.description()}", userData=i)
            # self.print(port.portName(), port.productIdentifier(),
            #            port.description())

    def closeEvent(self, event):
        if (self.logfile != None):
            self.logfile.flush()
            self.logfile.close()
        event.accept()  # let the window close


os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
# os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "2"
os.environ["QT_SCALE_FACTOR"] = "1.25"
app = QApplication(['', '--no-sandbox'])
# app.setAttribute(Qt.AA_EnableHighDpiScaling)
window = MainWindow()
window.show()
app.exec()
