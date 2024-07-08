import asyncio
import os
import sys
from io import TextIOWrapper
from qtpy.QtWidgets import *
from qtpy.QtGui import QPalette, QColor, QIcon
from qtpy.QtCore import *
from qtpy import QtWidgets
from pyqtlet2 import L, MapWidget
from qtpy.QtSerialPort import QSerialPort, QSerialPortInfo
from gen.mainwindow import Ui_MainWindow
from gen import logdialog
from gen.protocol_pb2 import *
from google.protobuf.json_format import MessageToJson, ParseDict
from util import MTID_TO_TYPE, msgtype_str, msgtype_fromstr
from receiver import Receiver
from typing import Deque
import json
from threading import Thread
from cacheserver import proxy_main
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
import matplotlib
matplotlib.use('QTAgg')
import time


CONSOLE_SCROLLBACK = 64


# Dialog for configuring logging to a file
class LogDialog(QDialog):
    """Employee dialog."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = logdialog.Ui_Dialog()
        self.ui.setupUi(self)
        self.filedialog = QFileDialog(self)

        self.log_new = False
        self.log_cancel = False
        self.log_filename = ""

        self.ui.buttonBox.button(
            QDialogButtonBox.StandardButton.Open).clicked.connect(self.open_btn)
        self.ui.buttonBox.button(
            QDialogButtonBox.StandardButton.Ok).clicked.connect(self.ok_btn)
        self.ui.buttonBox.button(
            QDialogButtonBox.StandardButton.Cancel).clicked.connect(self.cancel_btn)
        self.ui.currentLogFileLineEdit.textChanged.connect(self.fname_edit)

        # self.ui.currentLogFileLabel.setText(self.log_filename)

        self.filedialog.setFileMode(QFileDialog.FileMode.AnyFile)
        # self.filedialog.setNameFilter("Log files (*.txt *.log)")
        self.filedialog.setViewMode(QFileDialog.Detail)

    def fname_edit(self):
        self.log_filename = self.ui.currentLogFileLineEdit.text()

    def open_btn(self):
        self.log_filename, tp = self.filedialog.getSaveFileName(
            self, "Log File", "./tracker.log", "Log files (*.txt *.log)")
        self.ui.currentLogFileLineEdit.setText(self.log_filename)

    def cancel_btn(self):
        self.log_cancel = True
        self.accept()

    def ok_btn(self):
        self.log_new = len(self.log_filename) != 0
        self.accept()


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, *args, obj=None, **kwargs):
        # Window setup
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setWindowIcon(QIcon('icon.svg'))

        # Type definitions... annoying
        self.battery_bar: QProgressBar
        self.battery_voltage: QLabel
        self.rssi_disp: QLCDNumber
        self.snr_disp: QLCDNumber
        self.map_tab: QWidget
        self.graph_tab: QWidget
        self.port_selector: QComboBox
        self.connect_btn: QPushButton
        self.console: QPlainTextEdit
        self.port_rfsh_btn: QToolButton
        self.stats_frame: QFrame

        self.setupUi(self)
        self.session_start_t = time.time()

        # Set default UI states, disabled, etc.
        self.stats_frame.setDisabled(True)

        # Action (menu) buttons
        self.actionLog_Config: QtWidgets.QWidgetAction
        self.actionClear_Session: QtWidgets.QWidgetAction
        self.actionLoad_Log: QtWidgets.QWidgetAction
        self.actionFollow: QtWidgets.QWidgetAction

        # Logging
        self.logfile: TextIOWrapper = None  # open("tracker.log", 'w')

        # Ports
        self.avail_ports: list[QSerialPortInfo] = []
        self.tlm_port = QSerialPort()
        self.vgps_port = QSerialPort()

        # Colors of stats 7seg displays
        palette = self.rssi_disp.palette()
        palette.setColor(palette.WindowText, QColor(85, 85, 255))
        self.rssi_disp.setPalette(palette)
        self.snr_disp.setPalette(palette)

        # Message parsing, etc.
        self.receiver = Receiver(
            self.tlm_port, self.vgps_port, self.rx_cfg_frf, self.rx_cfg_bw, self.rx_cfg_pow, self.print)


        # Logs for visualization
        self.gps_track = Deque()
        self.alt_log = []

        # Map setup
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
            'http://127.0.0.1:8085/mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Gt').addTo(self.map)
        self.marker = L.marker(
            [0, 0])
        self.map_track_line = L.polyline(
            [], {"color": 'lightblue'}).addTo(self.map)

        self.map_tab.setLayout(self.maplayout)

        # Graph setup!
        self.graphlayout = QVBoxLayout()

        self.graphfig = Figure()        
        self.graphax = self.graphfig.add_subplot(111)
        self.graphcanvas = FigureCanvasQTAgg(self.graphfig, )
        self.graphcanvas.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.graphcanvas.updateGeometry()
        
        self.graphfig.tight_layout()

        self.graphlayout.setContentsMargins(0,0,0,0)
        self.graphlayout.addWidget(self.graphcanvas)
        self.graphlayout.addWidget(NavigationToolbar2QT(self.graphcanvas, self))
        self.graph_tab.setLayout(self.graphlayout)
        
        # Refresh button
        self.refreshports()
        self.port_rfsh_btn.clicked.connect(self.refreshports)
        self.connect_btn.clicked.connect(self.connectports)
        self.receiver.datumProcessed.connect(self.datum_cb)

        # Action (menu) buttons
        self.actionLog_Config.triggered.connect(self.logfile_menu_act)
        self.actionClear_Session.triggered.connect(self.clearsession_menu_act)
        self.actionLoad_Log.triggered.connect(self.loadlog_menu_act)

    def loadlog_menu_act(self):
        fsel = QFileDialog(self)
        loadfname, tp = fsel.getOpenFileName(
            self, "Log File", "./tracker.log", "Log files (*.txt *.log)")

        if (len(loadfname) > 0 and os.path.isfile(loadfname)):
            if (self.logfile is not None):
                self.logfile.close()

            # Clear current session
            self.clearsession_menu_act()

            with open(loadfname, 'r') as f:
                for line in f.readlines():
                    fjson = json.loads(line)
                    dtype = msgtype_fromstr(fjson['type'])
                    ddata = fjson['datum']

                    if (dtype is not None and ddata is not None):
                        self.datum_cb(dtype, ParseDict(
                            ddata, MTID_TO_TYPE[msgtype_str(dtype)]))

    def logfile_menu_act(self):
        dlg = LogDialog(self)
        dlg.exec()

        if (dlg.log_new):
            self.logfile = open(dlg.log_filename, 'w')
        elif (dlg.log_cancel and self.logfile != None and not self.logfile.closed):
            self.logfile.close()

    def clearsession_menu_act(self):
        self.stats_frame.setDisabled(True)
        
        self.alt_log.clear()
        self.update_alt_plot()
        
        self.session_start_t = time.time()
        
        self.gps_track.clear()
        self.map_track_line.latLngs = [[g[1],g[2]] for g in self.gps_track]
        self.map_track_line.removeFrom(self.map).addTo(self.map)
        
    def session_time(self):
        return time.time() - self.session_start_t

    def datum_cb(self, t: DatumTypeID, datum: object):
        self.stats_frame.setDisabled(False)  # We're getting messages!

        if (self.logfile != None and not self.logfile.closed):
            try:
                vtype = (MTID_TO_TYPE[msgtype_str(t)])
                if (vtype != None):
                    cls: vtype = datum
                    self.logfile.write(json.dumps({
                        "type": msgtype_str(t),
                        "datum": json.loads(MessageToJson(cls, preserving_proto_field_name=True)),
                        "timestamp": self.session_time()
                    }) + '\n')
                    self.logfile.flush()
            except Exception:
                self.print("Logging error!")

        # print(f"Type: {msgtype_str(t)}")

        if (t == INFO_GPS):
            datum: GPS = datum

            # TODO: Use CRC and more tracker testing to remove the need for this!!!
            if (len(self.gps_track) > 0 and False):
                if ((abs(self.gps_track[-1][0] - datum.lat) + abs(self.gps_track[-1][1] - datum.lon) > 0.1)):
                    return  # Reject ridiculous gps errors

            self.gps_track.append((self.session_time(), datum.lat, datum.lon, datum.alt))
            if (len(self.gps_track) > 86400): # OK I hope this never happens
                self.gps_track.popLeft()

            print(datum.fix_status)
            if (datum.fix_status < 2):
                return  # Nothing to do with invalid GPS fix

            self.map_track_line.latLngs = [[g[1],g[2]] for g in self.gps_track]
            self.map_track_line.removeFrom(self.map).addTo(self.map)

            self.marker.latLng = [datum.lat, datum.lon]
            self.marker.removeFrom(self.map).addTo(self.map)

            if (self.actionFollow.isChecked()):
                self.map.flyTo([datum.lat, datum.lon], 18)
                
            self.update_alt_plot()

        if (t == STATUS_RadioRxStatus):
            datum: RadioRxStatus

            self.rssi_disp.display(datum.RSSI)
            self.snr_disp.display(datum.SNR)

        if (t == INFO_Battery):
            datum: Battery
            # Reject obviously wrong values
            if (datum.battery_voltage > 24.0 or datum.battery_voltage < 0.0):
                return

            self.battery_voltage.setText(f"({datum.battery_voltage:.2f})")
            self.battery_bar.setValue(
                int(datum.percentage))
            
        if (t == INFO_Altitude):
            datum: Altitude
            self.alt_log.append((self.session_time(), datum.alt_m))
            self.update_alt_plot()
            
    def update_alt_plot(self):
        # print("EEE")
        self.graphax.clear()
        self.graphax.plot([l[0] for l in self.alt_log], [l[1] for l in self.alt_log], label="Barometric Altitude (m)")
        self.graphax.plot([g[0] for g in self.gps_track], [g[3] for g in self.gps_track], label="GPS Altitude (m)")
        # self.graphax.set_ylabel('Altitude (m)')
        self.graphax.legend()
        
        self.graphcanvas.draw() 
        pass

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
            if (self.port_selector.currentData() is None):
                self.print("Error, no ports available!")
                return

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
        if (self.logfile != None and not self.logfile.closed):
            self.logfile.flush()
            self.logfile.close()
        event.accept()  # let the window close


app_stop = False


def proxy_proc():
    global app_stop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    proxy_main(loop)

    # if (sys.platform not in ["linux", "linux2"]):
    while not app_stop:
        loop.run_until_complete(asyncio.sleep(1))
    pass


proxy = Thread(target=proxy_proc)
proxy.start()

try:
    # Make it big
    os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "1"
    os.environ["QT_SCALE_FACTOR"] = "1.25"
    app = QApplication(['', '--no-sandbox'])
    window = MainWindow()
    window.show()
    app.exec()
finally:
    app_stop = True
    proxy.join()
