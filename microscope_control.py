import sys
import usb.core
import usb.util
import toupcam
import numpy as np
import cv2
import time
import serial
from PyQt5.QtCore import pyqtSignal, QTimer, Qt, QSignalBlocker, QThread
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QLabel, QApplication, QWidget, QCheckBox, QMessageBox, QPushButton, QComboBox, QSlider, QGroupBox, QGridLayout, QHBoxLayout, QVBoxLayout, QMenu, QAction, QLCDNumber

ser = serial.Serial('COM3', 115200, timeout=1, write_timeout=5)

class AutoFocus:
    def __init__(self):
        pass

    def rgb_to_gray(self, image):
        gray_image = np.dot(image[..., :3], [38, 75, 15]) >> 7
        return gray_image.astype(np.uint8)

    def laplacian_variance(self, gray_image):
        """Calculate the variance of the Laplacian, which indicates the sharpness of the image."""
        laplacian = cv2.Laplacian(gray_image, cv2.CV_64F)
        return laplacian.var()

class FocusThread(QThread):
    finished = pyqtSignal()

    def __init__(self, main_widget):
        super().__init__()
        self.main_widget = main_widget
        self.is_running = True

    def run(self):
        self.main_widget.climb_hill_focus(50.0, 10.0, 20)
        self.finished.emit()

    def stop(self):
        self.is_running = False

class PositionWorker(QThread):
    update_position = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.running = False
        self.device = usb.core.find(idVendor=0x054C, idProduct=0x0061)
        if self.device is None:
            raise ValueError('Device not found')
        usb.util.claim_interface(self.device, 0)
        self.device.set_configuration()
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.flag = 0  # 用于控制键盘输入的状态
        self.home = 0
        self.alarmed = 0  # True if the system becomes in alarm status. Need to press button to reset
        self.cancel = 0
        self.move = 0
        self.x_min = 2
        self.x_max = 982
        self.y_min = 46
        self.y_max = 1023
        self.z_min = 63
        self.z_max = 1023

    def run(self):
        self.running = True
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        # Set reporting mask to 1
        self.send_gcode_command('$10=1\n')
        
        # 8/11/2024 Yuan: make sure it stays ON at initialization
        self.send_keep_on()
        self.flag = 1
        time_step = 0.04 # Yuan 8/11
        last_time = time.time()
        time_started = last_time
        while self.running:
            data = self.read_hid_data()
            if data:
                keyboard_input = self.check_keyboard(data)
                if keyboard_input == 1 and self.flag != 1:
                    self.send_keep_on()
                    self.flag = 1
                    print('on1')
                elif keyboard_input == 2 and self.flag != 2:
                    self.send_keep_off()
                    self.flag = 2
                    print('off')
                
                if keyboard_input == 5 and self.home == 0:
                    self.send_homing_command()
                    self.home = 1

                if keyboard_input == 7 and self.alarmed == 1:
                    self.send_reset_alarm()
                    self.alarmed = 0

                x = self.parse_axis_data(data[0], data[1])
                y = self.parse_axis_data(data[2], data[3])
                z = self.parse_axis_data(data[4], data[5])

                x = self.x_map(x)
                y = self.y_map(y)
                z = self.z_map(z)

                # Jog cancellation logic
                if (abs(x - 0x0200) < 0x0020 and abs(y - 0x0200) < 0x0020 and abs(z - 0x0200) < 0x0010) or self.alarmed:
                    if self.cancel == 0:
                        ser.reset_input_buffer()
                        self.send_jog_cancel()
                        self.update_position_from_device() # update once jog has stopped
                        self.cancel = 1
                        time.sleep(0.1)
                else: 
                    print('move')
                    gcode_command = self.process_data(x, y, z)
                    # Once move, keep the motor on
                    if gcode_command:
                        if self.flag != 1:
                            self.send_keep_on()
                            self.flag = 1

                        self.send_gcode_command(gcode_command, False)
                        self.cancel = 0
                        self.move = 1
                        self.home = 0

                        status = self.send_status_request()
                        
                        if status.find('Pn:') != -1:
                            self.alarmed = 1
                        #print(status)

                    self.update_position.emit(self.x_pos, self.y_pos, self.z_pos)

            time_now = time.time()
            time_elapsed = (time_now - last_time)
            time.sleep(max(0, time_step - time_elapsed))
            last_time = time.time()

    def stop(self):
        self.running = False

    def read_hid_data(self):
        data = None
        while True:
            try:
                data = self.device.read(0x81, 10, 1)
            except usb.core.USBError as e:
                if data == None:
                    continue
                else:
                    break
        #print(data)
        return data

    def parse_axis_data(self, low_byte, high_byte):
        return (high_byte << 8) | low_byte

    def map_value(self, value, min_input, max_input, range):
        return 8 * range * ((value - (min_input + max_input) / 2) ** 3) / ((max_input - min_input) ** 3)

    def map_value_z(self, value, min_input, max_input, range):
        return 16 * range * ((value - (min_input + max_input) / 2) ** 4) / ((max_input - min_input) ** 4) * ((value - (min_input + max_input) / 2) / abs(value - (min_input + max_input) / 2))

    def process_data(self, x, y, z):
        x_speed = -self.map_value(x, 0x0000, 0x03FF, 4000)
        y_speed = self.map_value(y, 0x0000, 0x03FF, 4000)
        z_speed = self.map_value_z(z, 0x0000, 0x03FF, 31000)

        tot_speed = (x_speed ** 2 + y_speed ** 2 + z_speed ** 2) ** 0.5

        #Step size setting
        dt = 0.025 / 60
        dtz = 0.025 / 60

        x_disp = x_speed * dt
        y_disp = y_speed * dt
        z_disp = z_speed * dtz

        self.x_pos += x_disp
        self.y_pos += y_disp
        self.z_pos += z_disp

        gcode_command = f'$J=G21G91X{x_disp:.3f}Y{y_disp:.3f}Z{z_disp:.3f}F{tot_speed:.1f}\n'
        return gcode_command

    def send_gcode_command(self, command, need_ret=True):
        if need_ret:
            ser.read_all()

        ser.write(command.encode())
        
        if need_ret:
            ret = ser.readline()
            print('command=',command, 'ret=', ret)
            return ret

    def send_jog_cancel(self):
        ser.write(b'\x85')
        ser.flush()

    def send_keep_on(self):
        self.send_gcode_command('$1=255\n')
        self.send_gcode_command('G0G91X0.01\n') 
        self.send_gcode_command('G0G91X-0.01\n') 

    def send_keep_off(self):
        self.send_gcode_command('$1=100\n')
        self.send_gcode_command('G0G91X0.01\n') 
        self.send_gcode_command('G0G91X-0.01\n') 

    def send_homing_command(self):
        try:
            ser.write(f'$H\n'.encode())
            ser.flush()
            time.sleep(3) 
            self.update_position_from_device()
            time.sleep(0.5)
        except serial.SerialTimeoutException:
            print("Homing command timeout")

    def send_reset_alarm(self):
        self.send_gcode_command('$X\n')
        print('Reset alarm')

    def send_status_request(self):
        return self.send_gcode_command('?\n').decode().strip()

    def check_keyboard(self, data):
        if data[6] == 0x01:
            return 1
        elif data[6] == 0x02:
            return 2
        elif data[6] == 0x04:
            return 3
        elif data[6] == 0x08:
            return 4
        elif data[6] == 0x10:
            return 5
        elif data[6] == 0x20:
            return 6
        elif data[6] == 0x40:
            return 7
        elif data[6] == 0x80:
            return 8

    def x_map(self, x):
        if x < 512:
            return 512 - 512 * (512 - x) / (512 - self.x_min)
        if x > 512:
            return 512 + 512 * (512 - x) / (512 - self.x_max)
        return 512

    def y_map(self, y):
        if y < 512:
            return 512 - 512 * (512 - y) / (512 - self.y_min)
        if y > 512:
            return 512 + 512 * (512 - y) / (512 - self.y_max)
        return 512
    
    def z_map(self, z):
        if z < 512:
            return 512 - 512 * (512 - z) / (512 - self.z_min)
        if z > 512:
            return 512 + 512 * (512 - z) / (512 - self.z_max)
        return 512

    def update_position_from_device(self):
        response = self.send_status_request()
        if response.startswith('<'):
            positions = self.parse_position_response(response)
            self.update_position.emit(*positions)

    def parse_position_response(self, response):
        # The response should be in terms of '<Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000>'
        mpos_start = response.find('MPos:') + len('MPos:')
        mpos_end = response.find('|', mpos_start)
        mpos_str = response[mpos_start:mpos_end]
        mpos = list(map(float, mpos_str.split(',')))
        self.x_pos, self.y_pos, self.z_pos = mpos
        return self.x_pos, self.y_pos, self.z_pos


class MainWidget(QWidget):
    evtCallback = pyqtSignal(int)

    @staticmethod
    def makeLayout(lbl1, sli1, val1, lbl2, sli2, val2):
        hlyt1 = QHBoxLayout()
        hlyt1.addWidget(lbl1)
        hlyt1.addStretch()
        hlyt1.addWidget(val1)
        hlyt2 = QHBoxLayout()
        hlyt2.addWidget(lbl2)
        hlyt2.addStretch()
        hlyt2.addWidget(val2)
        vlyt = QVBoxLayout()
        vlyt.addLayout(hlyt1)
        vlyt.addWidget(sli1)
        vlyt.addLayout(hlyt2)
        vlyt.addWidget(sli2)
        return vlyt

    def __init__(self):
        super().__init__()
        self.setMinimumSize(1280, 800)
        self.hcam = None
        self.timer = QTimer(self)
        self.imgWidth = 0
        self.imgHeight = 0
        self.pData = None
        self.res = 0
        self.temp = toupcam.TOUPCAM_TEMP_DEF
        self.tint = toupcam.TOUPCAM_TINT_DEF
        self.count = 0
        self.last_image = None
        self.last_image_flag = 0  # Flag to monitor whether last_image is active
        self.focus_thread = None  # Initialize focus_thread attribute
        self.position_worker = None

        gboxres = QGroupBox("Resolution")
        self.cmb_res = QComboBox()
        self.cmb_res.setEnabled(False)
        vlytres = QVBoxLayout()
        vlytres.addWidget(self.cmb_res)
        gboxres.setLayout(vlytres)
        self.cmb_res.currentIndexChanged.connect(self.onResolutionChanged)

        gboxexp = QGroupBox("Exposure")
        self.cbox_auto = QCheckBox("Auto exposure")
        self.cbox_auto.setEnabled(False)
        self.lbl_expoTime = QLabel("0")
        self.lbl_expoGain = QLabel("0")
        self.slider_expoTime = QSlider(Qt.Horizontal)
        self.slider_expoGain = QSlider(Qt.Horizontal)
        self.slider_expoTime.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        self.cbox_auto.stateChanged.connect(self.onAutoExpo)
        self.slider_expoTime.valueChanged.connect(self.onExpoTime)
        self.slider_expoGain.valueChanged.connect(self.onExpoGain)
        vlytexp = QVBoxLayout()
        vlytexp.addWidget(self.cbox_auto)
        vlytexp.addLayout(self.makeLayout(QLabel("Time(us):"), self.slider_expoTime, self.lbl_expoTime, QLabel("Gain(%):"), self.slider_expoGain, self.lbl_expoGain))
        gboxexp.setLayout(vlytexp)

        gboxwb = QGroupBox("White balance")
        self.btn_autoWB = QPushButton("White balance")
        self.btn_autoWB.setEnabled(False)
        self.btn_autoWB.clicked.connect(self.onAutoWB)
        self.lbl_temp = QLabel(str(toupcam.TOUPCAM_TEMP_DEF))
        self.lbl_tint = QLabel(str(toupcam.TOUPCAM_TINT_DEF))
        self.slider_temp = QSlider(Qt.Horizontal)
        self.slider_tint = QSlider(Qt.Horizontal)
        self.slider_temp.setRange(toupcam.TOUPCAM_TEMP_MIN, toupcam.TOUPCAM_TEMP_MAX)
        self.slider_temp.setValue(toupcam.TOUPCAM_TEMP_DEF)
        self.slider_tint.setRange(toupcam.TOUPCAM_TINT_MIN, toupcam.TOUPCAM_TINT_MAX)
        self.slider_tint.setValue(toupcam.TOUPCAM_TINT_DEF)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        self.slider_temp.valueChanged.connect(self.onWBTemp)
        self.slider_tint.valueChanged.connect(self.onWBTint)
        vlytwb = QVBoxLayout()
        vlytwb.addLayout(self.makeLayout(QLabel("Temperature:"), self.slider_temp, self.lbl_temp, QLabel("Tint:"), self.slider_tint, self.lbl_tint))
        vlytwb.addWidget(self.btn_autoWB)
        gboxwb.setLayout(vlytwb)

        self.btn_open = QPushButton("Open")
        self.btn_open.clicked.connect(self.onBtnOpen)
        self.btn_snap = QPushButton("Snap")
        self.btn_snap.setEnabled(False)
        self.btn_snap.clicked.connect(self.onBtnSnap)
        self.btn_focus = QPushButton("Focus")
        self.btn_focus.clicked.connect(self.onBtnFocus)
        self.btn_homing = QPushButton("Homing")
        self.btn_homing.clicked.connect(self.onHoming)
        self.btn_kill_alarm = QPushButton("Kill Alarm")
        self.btn_kill_alarm.clicked.connect(self.onKillAlarm)
        self.btn_unlock_motor = QPushButton("Unlock Motor")
        self.btn_unlock_motor.clicked.connect(self.onUnlockMotor)

        vlytctrl = QVBoxLayout()
        vlytctrl.addWidget(gboxres)
        vlytctrl.addWidget(gboxexp)
        vlytctrl.addWidget(gboxwb)
        vlytctrl.addWidget(self.btn_open)
        vlytctrl.addWidget(self.btn_snap)
        vlytctrl.addWidget(self.btn_focus)
        vlytctrl.addWidget(self.btn_homing)
        vlytctrl.addWidget(self.btn_kill_alarm)
        vlytctrl.addWidget(self.btn_unlock_motor)
        vlytctrl.addStretch()
        wgctrl = QWidget()
        wgctrl.setLayout(vlytctrl)

        self.lbl_frame = QLabel()
        self.lbl_video = QLabel()
        self.lcd_position_x = QLCDNumber(self)
        self.lcd_position_y = QLCDNumber(self)
        self.lcd_position_z = QLCDNumber(self)

        vlytshow = QVBoxLayout()
        vlytshow.addWidget(self.lbl_video, 1)
        vlytshow.addWidget(self.lbl_frame)
        wgshow = QWidget()
        wgshow.setLayout(vlytshow)

        gmain = QGridLayout()
        gmain.setColumnStretch(0, 1)
        gmain.setColumnStretch(1, 4)
        gmain.addWidget(wgctrl, 0, 0, 2, 1)
        gmain.addWidget(wgshow, 0, 1, 1, 1)
        gmain.addWidget(QLabel("Position X:"), 1, 1, 1, 1, Qt.AlignRight)
        gmain.addWidget(self.lcd_position_x, 1, 2, 1, 1)
        gmain.addWidget(QLabel("Position Y:"), 2, 1, 1, 1, Qt.AlignRight)
        gmain.addWidget(self.lcd_position_y, 2, 2, 1, 1)
        gmain.addWidget(QLabel("Position Z:"), 3, 1, 1, 1, Qt.AlignRight)
        gmain.addWidget(self.lcd_position_z, 3, 2, 1, 1)
        self.setLayout(gmain)

        self.timer.timeout.connect(self.onTimer)
        self.evtCallback.connect(self.onevtCallback)

    def onTimer(self):
        if self.hcam:
            nFrame, nTime, nTotalFrame = self.hcam.get_FrameRate()
            self.lbl_frame.setText("{}, fps = {:.1f}".format(nTotalFrame, nFrame * 1000.0 / nTime))

    def closeCamera(self):
        if self.hcam:
            self.hcam.Close()
        self.hcam = None
        self.pData = None

        self.btn_open.setText("Open")
        self.timer.stop()
        self.lbl_frame.clear()
        self.cbox_auto.setEnabled(False)
        self.slider_expoGain.setEnabled(False)
        self.slider_expoTime.setEnabled(False)
        self.btn_autoWB.setEnabled(False)
        self.slider_temp.setEnabled(False)
        self.slider_tint.setEnabled(False)
        self.btn_snap.setEnabled(False)
        self.btn_focus.setEnabled(False)
        self.cmb_res.setEnabled(False)
        self.cmb_res.clear()

    def closeEvent(self, event):
        if self.focus_thread is not None and self.focus_thread.isRunning():
            self.focus_thread.stop()
            self.focus_thread.wait()
        self.closeCamera()
        event.accept()

    def onResolutionChanged(self, index):
        if self.hcam:  # step 1: stop camera
            self.hcam.Stop()

        self.res = index
        self.imgWidth = self.cur.model.res[index].width
        self.imgHeight = self.cur.model.res[index].height

        if self.hcam:  # step 2: restart camera
            self.hcam.put_eSize(self.res)
            self.startCamera()

    def onAutoExpo(self, state):
        if self.hcam:
            self.hcam.put_AutoExpoEnable(1 if state else 0)
            self.slider_expoTime.setEnabled(not state)
            self.slider_expoGain.setEnabled(not state)

    def onExpoTime(self, value):
        if self.hcam:
            self.lbl_expoTime.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoTime(value)

    def onExpoGain(self, value):
        if self.hcam:
            self.lbl_expoGain.setText(str(value))
            if not self.cbox_auto.isChecked():
                self.hcam.put_ExpoAGain(value)

    def onAutoWB(self):
        if self.hcam:
            self.hcam.AwbOnce()

    def wbCallback(nTemp, nTint, self):
        self.slider_temp.setValue(nTemp)
        self.slider_tint.setValue(nTint)

    def onWBTemp(self, value):
        if self.hcam:
            self.temp = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_temp.setText(str(value))

    def onWBTint(self, value):
        if self.hcam:
            self.tint = value
            self.hcam.put_TempTint(self.temp, self.tint)
            self.lbl_tint.setText(str(value))

    def startCamera(self):
        self.pData = bytes(toupcam.TDIBWIDTHBYTES(self.imgWidth * 24) * self.imgHeight)
        uimin, uimax, uidef = self.hcam.get_ExpTimeRange()
        self.slider_expoTime.setRange(uimin, uimax)
        self.slider_expoTime.setValue(uidef)
        usmin, usmax, usdef = self.hcam.get_ExpoAGainRange()
        self.slider_expoGain.setRange(usmin, usmax)
        self.slider_expoGain.setValue(usdef)
        self.handleExpoEvent()
        if self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0:
            self.handleTempTintEvent()
        try:
            self.hcam.StartPullModeWithCallback(self.eventCallBack, self)
        except toupcam.HRESULTException:
            self.closeCamera()
            QMessageBox.warning(self, "Warning", "Failed to start camera.")
        else:
            self.cmb_res.setEnabled(True)
            self.cbox_auto.setEnabled(True)
            self.btn_autoWB.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.slider_temp.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.slider_tint.setEnabled(self.cur.model.flag & toupcam.TOUPCAM_FLAG_MONO == 0)
            self.btn_open.setText("Close")
            self.btn_snap.setEnabled(True)
            self.btn_focus.setEnabled(True)
            bAuto = self.hcam.get_AutoExpoEnable()
            self.cbox_auto.setChecked(1 == bAuto)
            self.timer.start(1000)

    def openCamera(self):
        self.hcam = toupcam.Toupcam.Open(self.cur.id)
        if self.hcam:
            self.res = self.hcam.get_eSize()
            self.imgWidth = self.cur.model.res[self.res].width
            self.imgHeight = self.cur.model.res[self.res].height
            with QSignalBlocker(self.cmb_res):
                self.cmb_res.clear()
                for i in range(0, self.cur.model.preview):
                    self.cmb_res.addItem("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height))
                self.cmb_res.setCurrentIndex(self.res)
                self.cmb_res.setEnabled(True)
            self.hcam.put_Option(toupcam.TOUPCAM_OPTION_BYTEORDER, 0)  # Qimage use RGB byte order
            self.hcam.put_AutoExpoEnable(1)
            self.startCamera()
            # Start position worker
            self.start_position_worker()

    def onBtnOpen(self):
        if self.hcam:
            self.closeCamera()
            if self.position_worker is not None and self.position_worker.isRunning():
                self.position_worker.running = False
                self.position_worker.wait()
        else:
            arr = toupcam.Toupcam.EnumV2()
            if 0 == len(arr):
                QMessageBox.warning(self, "Warning", "No camera found.")
            elif 1 == len(arr):
                self.cur = arr[0]
                self.openCamera()
            else:
                menu = QMenu()
                for i in range(0, len(arr)):
                    action = QAction(arr[i].displayname, self)
                    action.setData(i)
                    menu.addAction(action)
                action = menu.exec(self.mapToGlobal(self.btn_open.pos()))
                if action:
                    self.cur = arr[action.data()]
                    self.openCamera()

    def onBtnSnap(self):
        if self.hcam:
            if 0 == self.cur.model.still:  # not support still image capture
                if self.pData is not None:
                    image = QImage(self.pData, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
                    self.count += 1
                    image.save("pyqt{}.jpg".format(self.count))
            else:
                menu = QMenu()
                for i in range(0, self.cur.model.still):
                    action = QAction("{}*{}".format(self.cur.model.res[i].width, self.cur.model.res[i].height), self)
                    action.setData(i)
                    menu.addAction(action)
                action = menu.exec(self.mapToGlobal(self.btn_snap.pos()))
                self.hcam.Snap(action.data())

    def onBtnFocus(self):
        if self.hcam:
            # 如果当前有运行中的线程，先停止它
            if self.focus_thread is not None and self.focus_thread.isRunning():
                self.focus_thread.stop()
                self.focus_thread.wait()

            # 停止 HID 设备控制
            if self.position_worker is not None and self.position_worker.isRunning():
                self.position_worker.running = False
                self.position_worker.wait()

            # 创建并启动新的Focus线程
            self.focus_thread = FocusThread(self)
            self.focus_thread.finished.connect(self.onFocusFinished)
            self.focus_thread.start()

    def onFocusFinished(self):
        print("Focus operation completed.")
        self.focus_thread = None  # Reset the thread variable

        self.start_position_worker()

    def onHoming(self):
        if self.position_worker is not None and self.position_worker.isRunning():
            self.position_worker.send_homing_command()

    def onKillAlarm(self):
        if self.position_worker is not None and self.position_worker.isRunning():
            self.position_worker.send_reset_alarm()

    def onUnlockMotor(self):
        if self.position_worker is not None and self.position_worker.isRunning():
            self.position_worker.send_keep_on()

    def climb_hill_focus(self, first_step, min_step, max_iteration):
        auto_focus = AutoFocus()
        z_displacement = first_step
        l_var = np.zeros((3,), dtype=np.float64)
        count = 0
        direction = 1

        image = self.last_image.copy()
        gray_image = auto_focus.rgb_to_gray(image)
        l_var[-1] = auto_focus.laplacian_variance(gray_image)
        print(f"Initial Laplacian Variance: {l_var[-1]:.6f}")

        if self.hcam:
            for i in range(max_iteration):
                if not self.focus_thread.is_running:
                    break

                if direction == 1:
                    self.send_gcommand(f'$J=G91Z{direction * z_displacement* 0.4}F30000\n')
                else:
                    self.send_gcommand(f'$J=G91Z{direction * z_displacement}F30000\n')
                
                time.sleep(0.3)

                start_time = time.time()
                image = self.last_image.copy()
                gray_image = auto_focus.rgb_to_gray(image)
                laplacian_variance = auto_focus.laplacian_variance(gray_image)
                l_var[:-1] = l_var[1:]
                l_var[-1] = laplacian_variance
                print(f"Iteration: {i}, Laplacian Variance: {laplacian_variance:.6f}")
                count += 1
                end_time = time.time()
                print(f"Time to calculate:{end_time-start_time:.6f}s")

                if count == 1:
                    if l_var[2] >= l_var[1]:
                        print("first step direction correct")
                    else:
                        direction = - direction
                        print("first step direction incorrect")
                else:
                    if l_var[1] >= l_var[0] and l_var[1] >= l_var[2]:
                        if abs(z_displacement) > min_step:
                            # self.send_gcommand(f'$J=G91Z{z_displacement}F30000\n')
                            # time.sleep(0.3)
                            # l_var[1], l_var[2] = l_var[2], l_var[1]
                            direction = - direction
                            z_displacement = z_displacement / 2
                            print("go back with halved step")
                            continue
                        if abs(z_displacement) <= min_step:
                            print(f"z_displacement = {z_displacement} um")
                            if direction == 1:
                                self.send_gcommand(f'$J=G91Z{direction * z_displacement* 0.4}F30000\n')
                            else:
                                self.send_gcommand(f'$J=G91Z{direction * z_displacement}F30000\n')
                            print("Focus operation completed")
                            return
                    # elif l_var[2] < l_var[1]:
                    #     direction = - direction
                        continue
                    else:
                        continue

    @staticmethod
    def eventCallBack(nEvent, self):
        '''callbacks come from toupcam.dll/so internal threads, so we use qt signal to post this event to the UI thread'''
        self.evtCallback.emit(nEvent)

    def onevtCallback(self, nEvent):
        '''this run in the UI thread'''
        if self.hcam:
            if toupcam.TOUPCAM_EVENT_IMAGE == nEvent:
                self.handleImageEvent()
            elif toupcam.TOUPCAM_EVENT_EXPOSURE == nEvent:
                self.handleExpoEvent()
            elif toupcam.TOUPCAM_EVENT_TEMPTINT == nEvent:
                self.handleTempTintEvent()
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.handleStillImageEvent()
            elif toupcam.TOUPCAM_EVENT_ERROR == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Generic Error.")
            elif toupcam.TOUPCAM_EVENT_STILLIMAGE == nEvent:
                self.closeCamera()
                QMessageBox.warning(self, "Warning", "Camera disconnect.")

    def handleImageEvent(self):
        try:
            self.hcam.PullImageV3(self.pData, 0, 24, 0, None)
        except toupcam.HRESULTException:
            pass
        else:
            image_copy = np.frombuffer(self.pData, dtype=np.uint8).reshape(self.imgWidth, self.imgHeight, 3).copy()
            self.display_image(image_copy)
            self.last_image = image_copy
            if (self.last_image is not None) and (self.last_image_flag == 0):  # Check if it is activated
                print('Last image activated')
                self.last_image_flag = 1

    def display_image(self, image):
        qimage = QImage(image, self.imgWidth, self.imgHeight, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self.lbl_video.setPixmap(pixmap)

    def handleExpoEvent(self):
        time = self.hcam.get_ExpoTime()
        gain = self.hcam.get_ExpoAGain()
        with QSignalBlocker(self.slider_expoTime):
            self.slider_expoTime.setValue(time)
        with QSignalBlocker(self.slider_expoGain):
            self.slider_expoGain.setValue(gain)
        self.lbl_expoTime.setText(str(time))
        self.lbl_expoGain.setText(str(gain))

    def handleTempTintEvent(self):
        nTemp, nTint = self.hcam.get_TempTint()
        with QSignalBlocker(self.slider_temp):
            self.slider_temp.setValue(nTemp)
        with QSignalBlocker(self.slider_tint):
            self.slider_tint.setValue(nTint)
        self.lbl_temp.setText(str(nTemp))
        self.lbl_tint.setText(str(nTint))

    def handleStillImageEvent(self):
        info = toupcam.ToupcamFrameInfoV3()
        try:
            self.hcam.PullImageV3(None, 1, 24, 0, info)  # peek
        except toupcam.HRESULTException:
            pass
        else:
            if info.width > 0 and info.height > 0:
                buf = bytes(toupcam.TDIBWIDTHBYTES(info.width * 24) * info.height)
                try:
                    self.hcam.PullImageV3(buf, 1, 24, 0, info)
                except toupcam.HRESULTException:
                    pass
                else:
                    image = QImage(buf, info.width, info.height, QImage.Format_RGB888)
                    self.count += 1
                    image.save("pyqt{}.jpg".format(self.count))

    def send_gcommand(self, command, need_ret=True):
        if need_ret:
            ser.read_all()  # Make sure there is nothing to read

        ser.write(command.encode())

        if need_ret:
            ret = ser.readline()
            print('command=', command, 'ret=', ret)
            return ret

    def start_position_worker(self):
        if self.position_worker is not None and self.position_worker.isRunning():
            self.position_worker.running = False
            self.position_worker.wait()

        self.position_worker = PositionWorker()
        self.position_worker.update_position.connect(self.update_position)
        self.position_worker.start()

    def update_position(self, x, y, z):
        self.lcd_position_x.display(x)
        self.lcd_position_y.display(y)
        self.lcd_position_z.display(z)


if __name__ == '__main__':
    toupcam.Toupcam.GigeEnable(None, None)
    app = QApplication(sys.argv)
    mw = MainWidget()
    mw.show()
    sys.exit(app.exec_())
