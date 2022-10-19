from vispy.app import use_app
from vispy import scene, app, gloo
from vispy.scene import visuals
from vispy.util.transforms import perspective, translate, rotate

from PyQt5 import QtWidgets, QtCore

import serial

import numpy as np
import time
import threading
import csv
import GSGenerator as generator

IMAGE_SHAPE = (2000, 2000) # (height, width)
CANVAS_SIZE = (2000, 2000) # (width, height)
SIZE_MAX = 80
LINE_COLORS = ['red', 'white', 'yellow']
LINE_WIDTH = 2
DEFAULT_PADDING = 10
COM = "COM5"
TEST_COM = "COM6"

missionTime = "00:00:00.00"
packetCount = "0"
swState = "LAUNCH READY"
# STATES: LAUNCH READY, ASCENDING, RELEASING, DESCENDING, LANDED
plState = "N"
# STATES: N for not released, R for released
voltage = 6.0
altData = []
tempData = []
locData = []
rotData = (0.0, 0.0, 0.0)
flag = True

class GroundStationWindow(QtWidgets.QMainWindow):

    closing = QtCore.pyqtSignal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        central_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout()

        self.canvas_wrapper = CanvasWrapper(self)
        main_layout.addWidget(self.canvas_wrapper.canvas.native)
        self.side_bar = GSSidebar()
        main_layout.addWidget(self.side_bar)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
    
    def closeEvent(self, event):
        print("Closing main window!")
        self.closing.emit()
        return super().closeEvent(event)

class GSSidebar(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()

        # team name & ID label
        self.teamLabel = QtWidgets.QLabel("Ocean's 8 - 1004")
        layout.addWidget(self.teamLabel)

        # mission time label
        self.timeLabel = QtWidgets.QLabel(f"Mission Time: {missionTime}")
        layout.addWidget(self.timeLabel)

        # packet count label
        self.packetLabel = QtWidgets.QLabel(f"Packet Count: {packetCount}")
        layout.addWidget(self.packetLabel)

        # software state label
        self.softwareLabel = QtWidgets.QLabel(f"Software State: {swState}")
        layout.addWidget(self.softwareLabel)

        # payload state label
        self.payloadLabel = QtWidgets.QLabel(f"Payload State: {plState}")
        layout.addWidget(self.payloadLabel)

        # voltage label
        self.voltageLabel = QtWidgets.QLabel(f"Voltage: {voltage}V")
        layout.addWidget(self.voltageLabel)

        # manual release button
        self.releaseBtn = QtWidgets.QPushButton("Manual Release")
        self.releaseBtn.clicked.connect(manualRelease)
        layout.addWidget(self.releaseBtn)

        # ping button
        self.pingBtn = QtWidgets.QPushButton("Ping CanSat")
        self.pingBtn.clicked.connect(ping)
        layout.addWidget(self.pingBtn)

        self.startBtn = QtWidgets.QPushButton("Start")
        self.startBtn.clicked.connect(start)
        layout.addWidget(self.startBtn)

        self.debugLabel = QtWidgets.QLabel("Toggle Debug")
        self.debugLabel.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.debugLabel)

        self.debugToggle = QtWidgets.QCheckBox()
        self.debugToggle.clicked.connect(self.updateDebug)
        layout.addWidget(self.debugToggle)
        
        layout.addStretch(1)
        self.setLayout(layout)

    def updateLabels(self):
        self.timeLabel.setText(f"Mission Time: {missionTime}")
        self.packetLabel.setText(f"Packet Count: {packetCount}")
        self.softwareLabel.setText(f"Payload State: {swState}")
        self.payloadLabel.setText(f"Payload State: {plState}")
        self.voltageLabel.setText(f"Voltage: {voltage}V")
        print("Labels updated.")

    def updateDebug(self):
        global debug
        if self.debugToggle.isChecked():
            print("Debug Mode ON")
            debug = True
        else:
            print("Debug Mode OFF")
            debug = False

class CanvasWrapper(QtCore.QObject):

    update_side = QtCore.pyqtSignal()
    
    def __init__(self, parent):
        super().__init__(parent=parent)
        self.canvas = scene.SceneCanvas(size=CANVAS_SIZE)
        self.master_grid = self.canvas.central_widget.add_grid(margin=10)
        self.master_grid.spacing = 100

        # ALTITUDE GRAPH
        self.alt_grid = self.master_grid.add_grid(margin=100, row=0, col=0)
        self.alt_grid.spacing = 100

        alt_title = scene.Label("Altitude", color='white')
        alt_title.height_max = 30
        self.alt_grid.add_widget(alt_title, row=0, col=1, col_span=-2)

        alt_axis = scene.AxisWidget(
            orientation='left',
            axis_label='Altitude (m)',
            axis_font_size=9,
            axis_label_margin=30,
            tick_label_margin=5
        )
        alt_axis.width_max = SIZE_MAX
        self.alt_grid.add_widget(alt_axis, row=1, col=0)

        alt_time_axis = scene.AxisWidget(
            orientation='bottom',
            axis_label='Time (s)',
            axis_font_size=9,
            axis_label_margin= 30,
            tick_label_margin=20
        )
        alt_time_axis.height_max = SIZE_MAX
        self.alt_grid.add_widget(alt_time_axis, row=2, col=1)

        right_alt_padding = self.alt_grid.add_widget(row=1, col=2)
        right_alt_padding.width_max = 50

        alt_view = self.alt_grid.add_view(row=1, col=1, border_color='white')
        alt_data = np.array([[0,0]])
        self.alt_plot = visuals.Line(alt_data, parent=alt_view.scene, color=LINE_COLORS[0], width=LINE_WIDTH)
        alt_view.camera = 'panzoom'

        alt_axis.link_view(alt_view)
        alt_time_axis.link_view(alt_view)

        # TEMPERATURE GRAPH
        self.temp_grid = self.master_grid.add_grid(margin=100, row=0, col=1)
        self.temp_grid.spacing = 100

        temp_title = scene.Label("Temperature", color='white')
        temp_title.height_max = 30
        self.temp_grid.add_widget(temp_title, row=0, col=1, col_span=-2)

        temp_axis = scene.AxisWidget(
            orientation='left',
            axis_label='Temperature (deg C)',
            axis_font_size=9,
            axis_label_margin=30,
            tick_label_margin=5
        )
        temp_axis.width_max = SIZE_MAX
        self.temp_grid.add_widget(temp_axis, row=1, col=0)

        temp_time_axis = scene.AxisWidget(
            orientation='bottom',
            axis_label='Time (s)',
            axis_font_size=9,
            axis_label_margin=30,
            tick_label_margin=20
        )
        temp_time_axis.height_max = SIZE_MAX
        self.temp_grid.add_widget(temp_time_axis, row=2, col=1)

        right_temp_padding = self.temp_grid.add_widget(row=1, col=2)
        right_temp_padding.width_max = 50

        temp_view = self.temp_grid.add_view(row=1, col=1, border_color='white')
        temp_data = np.array([[0,0]])
        self.temp_plot = visuals.Line(temp_data, parent=temp_view.scene, color=LINE_COLORS[1], width=LINE_WIDTH)
        temp_view.camera = 'panzoom'

        temp_axis.link_view(temp_view)
        temp_time_axis.link_view(temp_view)

        # LOCATION GRAPH
        self.loc_grid = self.master_grid.add_grid(margin=100, row=1, col=0)
        self.loc_grid.spacing = 100

        loc_title = scene.Label("Location", color='white')
        loc_title.height_max = 30
        self.loc_grid.add_widget(loc_title, row=0, col=1, col_span=-2)

        lat_axis = scene.AxisWidget(
            orientation='left',
            axis_label='Latitude',
            axis_font_size=9,
            axis_label_margin=30,
            tick_label_margin=5
        )
        lat_axis.width_max = SIZE_MAX
        self.loc_grid.add_widget(lat_axis, row=1, col=0)

        long_axis = scene.AxisWidget(
            orientation='bottom',
            axis_label='Longitude',
            axis_font_size=9,
            axis_label_margin=30,
            tick_label_margin=20
        )
        long_axis.height_max = SIZE_MAX
        self.loc_grid.add_widget(long_axis, row=2, col=1)

        right_loc_padding = self.loc_grid.add_widget(row=1, col=2)
        right_loc_padding.width_max = 50

        loc_view = self.loc_grid.add_view(row=1, col=1, border_color='white')
        loc_data = np.array([[0,0]])
        self.loc_plot = visuals.Line(loc_data, parent=loc_view.scene, color=LINE_COLORS[2], width=LINE_WIDTH)
        loc_view.camera = 'panzoom'

        lat_axis.link_view(loc_view)
        long_axis.link_view(loc_view)

        # ROTATION GRAPH
        self.rot_grid = self.master_grid.add_grid(margin=100, row=1, col=1)
        self.rot_grid.spacing = 100

        rot_title = scene.Label("Orientation", color='white')
        rot_title.height_max = 30
        self.rot_grid.add_widget(rot_title, row=0, col=1, col_span =-2)


        self.rot_roll = scene.Label(f"Roll: {rotData[0]}", color='white')
        self.rot_grid.add_widget(self.rot_roll, row=1, col=0)

        self.rot_pitch = scene.Label(f"Pitch: {rotData[1]}", color='white')
        self.rot_grid.add_widget(self.rot_pitch, row=1, col=1)

        self.rot_yaw = scene.Label(f"Yaw: {rotData[2]}", color='white')
        self.rot_grid.add_widget(self.rot_yaw, row=1, col=2)

    def updateData(self):
        global altData
        global tempData
        global locData
        global rotData
        print("Updating data...")
        self.alt_plot.set_data(np.asarray(altData))
        self.temp_plot.set_data(np.asarray(tempData))
        self.loc_plot.set_data(np.asarray(locData))

        self.rot_roll.text = f"Roll: {rotData[0]}"
        self.rot_pitch.text = f"Pitch: {rotData[1]}"
        self.rot_yaw.text = f"Yaw: {rotData[2]}"

        self.update_side.emit()

class DataSource(QtCore.QObject):

    new_data = QtCore.pyqtSignal()

    def __init__(self, num_iterations=1000, sim=False, parent=None):
        super().__init__(parent)
        self.shouldEnd = False
        self.isSim = sim
        self.numIters = num_iterations

    def generateData(self):
        global missionTime
        global packetCount
        global swState
        global plState
        global voltage
        global altData
        global tempData
        global locData
        global rotData

        print("Data creation is starting!")

        # connect xbee
        ser = connectSerial(COM)

        f = open("cansatdata.csv", "w")
        writer = csv.writer(f)

        count = 1

        if self.isSim:
            print("DEBUG TESTING")
            gen = generator.generateVals(self.numIters)
            test_ser = connectSerial(TEST_COM)
            count = 0

        while True:
            if self.shouldEnd or (self.isSim and self.numIters == count):
                print("Ground Station was told to stop!")
                break
            
            try:
                if self.isSim:
                    count += 1
                    test_ser.write(f'{next(gen)}\n'.encode(encoding="ascii", errors="replace"))
                # get data from serial
                print("Waiting for packet...")
                packet = ser.readline().strip().decode(encoding="ascii")
            except serial.serialutil.SerialException:
                # xbee disconnected
                print("XBee disconnected.")
                if self.isSim:
                    print("Check that com0com is active.")
                time.sleep(0.05)
                ser = connectSerial(COM)
            else:
                print(packet)

                ID, missionTime, packetCount, swState, plState, alt, temp, voltage, gpslat, gpslong, gyror, gyrop, gyroy = packet.split(',')
                cTime = timeToSeconds(missionTime)

                writer.writerow([ID, missionTime, packetCount, swState, plState, alt, temp, voltage, gpslat, gpslong, gyror, gyrop, gyroy]) # format packet

                altData += [[cTime, float(alt)]]
                tempData += [[cTime, float(temp)]]
                locData += [[float(gpslat), float(gpslong)]]
                rotData = (float(gyror), float(gyrop), float(gyroy))

                self.new_data.emit()
        print("Ground Station done receiving data.")

    def stop_data(self):
        print("Data source is quitting...")
        self.shouldEnd = True

def timeToSeconds(cTime):
    """
    Assume time is a string in the format HH:MM:SS

    Returns time in seconds (int)
    """
    hours, minutes, seconds = cTime.split(':')
    return float(hours)*3600 + float(minutes)*60 + float(seconds)

def connectSerial(usbport):
    try:
        ser = serial.Serial(
            port=usbport,
            baudrate=9600
        )
        return ser
    except:
        print("Could not connect to XBee.")
        return serial.Serial()

def manualRelease():
    global debug
    ser = connectSerial(COM)
    if debug:
        print("DEBUG RECEIVING")
        debugSer = connectSerial(TEST_COM)
    print("Sending manual release signal...")
    ser.write("Manual Release Signal\n".encode(encoding="ascii", errors="replace"))
    if debug:
        print(f"{debugSer.readline().strip().decode(encoding='ascii')}\n")
        print("Signal received.")
    return


def ping():
    global flag
    global debug
    ser = connectSerial(COM)
    if debug:
        debugSer = connectSerial(TEST_COM)
    else:
        debugSer = None

    begin = time.perf_counter()

    ser.write("Ping!\n".encode(encoding="ascii", errors="replace"))

    timer = threading.Event()
    t1 = threading.Thread(target=pong, args=(ser, timer, debugSer))
    t1.start()
    if timer.wait(10):
        end = time.perf_counter()
        print(f"Ping successful! {end - begin: 0.2f} seconds.")
    else:
        print("Ping unsuccessful. Please try again.")
    flag = False
    t1.join()
    flag = True
    print("Ping function finished.")

def pong(ser, timer, debugSer=None):
    global flag
    print("Waiting to receive pong...")
    if debugSer != None:
        print("DEBUG WRITING")
        time.sleep(4)
        debugSer.write("1004PING\n".encode(encoding="ascii", errors="replace"))
        print("SIGNAL SENT")
    while flag:
        if ser.readline().strip().decode(encoding="ascii") == "1004PING":
            print("Pong!")
            timer.set()
            return
        else:
            print("Invalid packet received.")

def start():
    global threadCreated
    global debug
    if threadCreated:
        return
    iters = lambda flag: 1000 if flag else 1000
    dataSource = DataSource(sim=debug, num_iterations=iters(debug))
    dataThread = threading.Thread(target=dataSource.generateData)
    print('Starting run!')
    threadCreated = True

    dataSource.new_data.connect(lambda: win.canvas_wrapper.updateData())
    win.closing.connect(lambda: dataSource.stop_data(), QtCore.Qt.DirectConnection)

    dataThread.start()

if __name__ == "__main__":
    threadCreated = False
    debug = False

    app = use_app("pyqt5")
    app.create()

    win = GroundStationWindow()
    
    win.canvas_wrapper.update_side.connect(lambda: win.side_bar.updateLabels())

    win.show()
    app.run()

    if threadCreated:
        print("Waiting for data source to close...")

        time.sleep(5)
