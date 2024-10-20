import asyncio
import csv
import json
import random
import shutil
import sys
import time
from datetime import datetime
from io import BytesIO
from pathlib import Path

import folium
import numpy as np
import paramiko
import requests
import serial
import vtk
import websockets
from PySide6.QtCore import (
    QRunnable, QThreadPool
)
from PySide6.QtCore import QThread, Signal, Slot, QTimer, Qt
from PySide6.QtGui import QIcon
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QSizePolicy, QFileDialog
)
from PySide6.QtWidgets import QWidget, QVBoxLayout, QGraphicsView, QGraphicsScene
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

from release_ui_form import Ui_MainWindow


class BlackboxCSV:
    def __init__(self, folder_path="335592_BlackBox"):
        self.folder_path = Path(folder_path)
        self.folder_path.mkdir(parents=True, exist_ok=True)
        self.backup_csv()
        self.backup_html()

    def write_csv(self, raw_data, filename="335592_csv_data.csv"):
        file_path = self.folder_path / filename
        header = ['Packet_count', 'Satellite_Status', 'Error_Code', 'Mission_Time', 'Pressure1', 'Pressure2',
                  'Altitude1', 'Altitude2', 'Altitude_Difference', 'Descent_rate', 'Temperature', 'Battery_Voltage',
                  'Gps_Latitude', 'Gps_Longitude', 'Gps_Altitude', 'Pitch', 'Roll', 'Yaw', 'LNLN', 'Iot_Data',
                  'Team_Number']

        write_header = not file_path.exists()
        try:
            with file_path.open("a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                if write_header:
                    writer.writerow(header)
                writer.writerow(raw_data.split(','))
        except Exception as e:
            print(f"Error writing data into CSV: {e}")

    def backup_file(self, filename, backup_folder_name):
        file_path = self.folder_path / filename
        if not file_path.exists():
            print(f"File {filename} does not exist. Skipping backup.")
            return

        backup_folder = self.folder_path / backup_folder_name
        backup_folder.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        new_file_name = f"{timestamp}_{filename}"
        try:
            shutil.copy(file_path, backup_folder / new_file_name)
            file_path.unlink()
            print(f"Backup created: {backup_folder / new_file_name}")
        except Exception as e:
            print(f"Error creating backup for {filename}: {e}")

    def backup_html(self, filename="335592_html_data.html"):
        self.backup_file(filename, "Backup HTML Files")

    def backup_csv(self, filename="335592_csv_data.csv"):
        self.backup_file(filename, "Backup CSV Files")

    def read_csv(self, filename="335592_csv_data.csv"):
        try:
            file_path = self.folder_path / filename
            with file_path.open("r", newline="") as f:
                reader = csv.reader(f)
                last_row = None
                for row in reader:
                    last_row = row
                return last_row
        except Exception as e:
            print(f"Error reading data from CSV file: {e}")


class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    def run(self):
        self.fn(*self.args, **self.kwargs)


class WebSocketClient(QThread):
    telemetry_received = Signal(str)

    def __init__(self, uri):
        super().__init__()
        self.uri = uri
        self.running = False
        self.reconnect_delay = 1  # delay in seconds between reconnection attempts

    async def websocket_connect(self):
        while self.running:
            try:
                async with websockets.connect(self.uri) as websocket:
                    while self.running:
                        try:

                            message = await websocket.recv()
                            self.telemetry_received.emit(message)
                        except Exception as e:
                            print("Error connecting to WebSocket:", e)
            except Exception as e:
                print("Error connecting to WebSocket:", e)
                await asyncio.sleep(1)  # Optional backoff before retrying the connection

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        asyncio.run(self.websocket_connect())


class IoTDataSendThread(QThread):
    transfer_complete = Signal(bool, str)

    def __init__(self, serial_port, baud_rate, ws_uri, ui):
        super().__init__()
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ws_uri = ws_uri
        self.running = False
        self.serial_connection = None
        self.main_window_ui = ui
        self.data_queue = asyncio.Queue()

    def run(self):
        asyncio.run(self.manage_threads())

    async def manage_threads(self):
        self.running = True
        serial_task = asyncio.create_task(self.read_serial_data())
        websocket_task = asyncio.create_task(self.send_websocket_data())

        await asyncio.gather(serial_task, websocket_task)

    async def read_serial_data(self):
        while self.running:
            try:
                self.connect_serial()
                while self.serial_connection and self.serial_connection.is_open:
                    if self.serial_connection.in_waiting > 0:
                        data = self.serial_connection.readline().decode('utf-8').strip()
                        print(data)
                        await self.data_queue.put(data)
                    await asyncio.sleep(0.1)  # To avoid tight loop
            except serial.SerialException as e:
                print(f"SerialException: {e}")
                self.connect_serial()
            except Exception as e:
                print(f"Error reading from serial: {e}")
                self.transfer_complete.emit(False, f"Error reading from serial: {e}")

    async def send_websocket_data(self):
        while self.running:  # Continue running while the system is active
            try:
                async with websockets.connect(self.ws_uri) as websocket:
                    try:
                        iot_data = await self.data_queue.get()
                        release_command = self.main_window_ui.ReleaseCommand
                        self.main_window_ui.ReleaseCommand = False
                        await websocket.send(f"{iot_data},{release_command}")
                        self.transfer_complete.emit(True, "Data transfer completed successfully.")
                    except Exception as e:
                        print(f"Error sending via websocket: {e}")
                        self.transfer_complete.emit(False, f"Error sending via websocket: {e}")
                        await asyncio.sleep(1)  # Optional backoff before retrying to send data
            except Exception as e:
                print(f"Error connecting to IoT Data WebSocket: {e}")
                self.transfer_complete.emit(False, f"Error connecting to IoT Data WebSocket: {e}")
                await asyncio.sleep(1)  # Optional backoff before retrying the connection

    def connect_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate)
            print("Serial connection established")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.serial_connection = None

    def stop(self):
        self.running = False
        self.data_queue = asyncio.Queue()  # Reset the data queue for future use
        print("Thread stopped, but UI still running")


class MapWidget(QWidget):
    def __init__(self, parent=None):
        super(MapWidget, self).__init__(parent)
        self.layout = None
        self.map_frame = None
        self.initUI()
        self.polyline_data = [[0, 0]]
        self.marker_coords = self.polyline_data[-1]
        self.circle_coords = self.polyline_data[-1]
        self.circle_radius = 5
        self.update_map()

    def initUI(self):
        self.layout = QVBoxLayout(self)
        self.map_frame = QWebEngineView()
        self.map_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.layout.addWidget(self.map_frame)

    def update_map(self):
        try:
            # Create a Folium map
            MAP_TILE_URL = """ADD YOUR OWN TILE ("https://api.tiles.mapbox.com/styles/v1/whoisjayd/clcrrua9d002e14o75f3w7gsl/tiles/{z}/{"
                            "x}/{"
                            "y}?access_token=YOUR TOKEN HERE)"""
            map_data = folium.Map(
                tiles=MAP_TILE_URL,
                zoom_start=17,
                location=self.marker_coords,
                attr=" ",
                max_zoom=100,
            )
            folium.PolyLine(self.polyline_data, color="blue", weight=2.5, opacity=1).add_to(map_data)
            folium.Marker(self.marker_coords, popup="").add_to(map_data)
            folium.Circle(self.circle_coords, radius=self.circle_radius).add_to(map_data)
            html = map_data.get_root().render()  # Render the Folium map to HTML

            # Update the map size dynamically using CSS
            html = f"<style>#map{{height:100%;width:100%;}}</style>{html}"
            with open("335592_BlackBox/335592_html_data.html", "w") as f:
                f.write(html)

            self.map_frame.setHtml(html)
        except Exception as e:
            print(f"Error updating map: {e}")

    def update_polyline(self, new_data):
        self.polyline_data.append(new_data)
        self.marker_coords = self.polyline_data[-1]
        self.circle_coords = self.polyline_data[-1]
        self.update_map()


class OrientationDisplayWidget(QWidget):
    def __init__(self, parent=None):
        super(OrientationDisplayWidget, self).__init__(parent)

        # Create a VTK widget
        self.vtk_widget = QVTKRenderWindowInteractor(self)

        self.setLayout(QVBoxLayout())  # Set layout for the widget
        self.layout().addWidget(self.vtk_widget)

        # Create a renderer, render window, and interactor
        self.renderer = vtk.vtkRenderer()
        self.render_window = self.vtk_widget.GetRenderWindow()
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()

        # Load STL file representing a cylinder
        self.reader = vtk.vtkSTLReader()
        self.reader.SetFileName("science.STL")
        self.reader.Update()

        # Get the output polydata from the reader
        self.camera = vtk.vtkCamera()
        self.polydata = self.reader.GetOutput()

        # Create a mapper and actor for the polydata
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputData(self.polydata)

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        # Set the center of rotation at the central axis of the cylinder
        bounds = self.polydata.GetBounds()
        self.actor.SetOrigin((bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2)

        # Add the actor to the renderer
        self.renderer.AddActor(self.actor)

        # Set the background color of the renderer
        self.renderer.SetBackground(8 / 255, 18 / 255, 44 / 255)

        # Add the renderer to the render window
        self.render_window.AddRenderer(self.renderer)

        # Initialize the interactor
        self.interactor.Initialize()

    def update_orientation(self, accel_x, accel_y, accel_z):
        self.actor.SetOrientation(0.0, 0.0, 0.0)
        self.actor.RotateX(accel_y)
        self.actor.RotateY(accel_x)
        self.actor.RotateZ(accel_z)
        self.vtk_widget.GetRenderWindow().Render()


def clean_data(data):
    if isinstance(data, np.ndarray):
        data = data[np.isfinite(data)]
    else:
        data = [x for x in data if np.isfinite(x)]
    return data


class DataProcessorThread(QThread):
    data_processed = Signal(dict)

    def __init__(self, graph_data):
        super().__init__()
        self.graph_data = graph_data

    def run(self):
        processed_data = {graph_name: clean_data(data) for graph_name, data in self.graph_data.items()}
        self.data_processed.emit(processed_data)


class VideoThread(QThread):
    change_pixmap_signal = Signal(QImage)

    def __init__(self, url):
        super().__init__()
        self.url = url
        self._run_flag = True

    def run(self):
        while self._run_flag:
            try:
                # Fetch the image data from the URL
                response = requests.get(self.url, stream=True)
                response.raise_for_status()
                image_data = BytesIO(response.content)

                # Create QImage from the image data
                qt_image = QImage()
                qt_image.loadFromData(image_data.read())

                # Emit the signal with the new image
                self.change_pixmap_signal.emit(qt_image)

            except requests.RequestException as e:
                print(f"Error fetching video frame: {e}")
                break

    def stop(self):
        self._run_flag = False
        self.wait()


class VideoWidget(QWidget):
    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url

        # Setup layout and graphics view
        layout = QVBoxLayout(self)
        self.view = QGraphicsView(self)
        layout.addWidget(self.view)
        layout.setContentsMargins(0, 0, 0, 0)

        # Setup scene and pixmap item
        self.scene = QGraphicsScene(self)
        self.view.setScene(self.scene)
        self.pixmap_item = self.scene.addPixmap(QPixmap())

        # Start video thread
        self.thread = VideoThread(self.url)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

        # Timer for refreshing the video feed
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(1000)  # Refresh every 1 second

    @Slot(QImage)
    def update_image(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        self.pixmap_item.setPixmap(pixmap)
        self.scene.setSceneRect(pixmap.rect())
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def closeEvent(self, event):
        self.timer.stop()
        self.thread.stop()
        self.thread.quit()
        super().closeEvent(event)

    def refresh(self, new_url=None):
        self.thread.stop()
        self.thread.quit()
        if new_url:
            self.url = new_url
        self.thread = VideoThread(self.url)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()


class FileTransferThread(QThread):
    transfer_complete = Signal(bool, float)

    def __init__(self, video_path, destination_path, hostname, username, password):
        super().__init__()
        self.video_path = video_path
        self.destination_path = destination_path
        self.hostname = hostname
        self.username = username
        self.password = password

    def run(self):
        try:
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh_client.connect(self.hostname, username=self.username, password=self.password)

            sftp_client = ssh_client.open_sftp()

            start_time = time.time()
            sftp_client.put(self.video_path, self.destination_path)
            end_time = time.time()

            sftp_client.close()
            ssh_client.close()

            transfer_time = round(end_time - start_time, 2)
            self.transfer_complete.emit(True, transfer_time)
        except Exception as e:
            print(f"Error occurred: {e}")
            self.transfer_complete.emit(False, f"Error occurred: {e}")


def onIOTTransferComplete(success, message):
    if success:
        print("Data transfer completed successfully.")
    else:
        print(f"Error: {message}")


class ButtonHandler:
    def __init__(self, main_window):
        self.file_transfer_thread = None
        self.main_window = main_window
        self.selected_file = None
        self.threadpool = QThreadPool()
        self.websocket_client = None
        self.iot_thread = None
        self.recovery_timer = None
        self.mission_state = {
            0: "Ready to flight",
            1: "Ascent",
            2: "Model satellite descent",
            3: "Release",
            4: "Science payload descent",
            5: "Recovery (Payload ground contact)"
        }

    def handle_select_file(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.ExistingFile)
        file_dialog.setNameFilter("Video Files (*.mp4 *.avi *.mov *.mkv *.flv)")
        if file_dialog.exec():
            self.selected_file = file_dialog.selectedFiles()[0]
            self.main_window.ui.SVSP_ToolButton.setText(self.selected_file[:60] + "...")
            print(f"Selected file: {self.selected_file}")

    def handle_send_file(self):
        if self.selected_file:
            self.main_window.ui.SVSP_SendButton.setEnabled(False)
            print(f"Sending file: {self.selected_file}")
            self.main_window.ui.SVSP_SendButton.setText("Sending...")

            hostname = "192.168.0.104"
            username = "dyaus"
            password = "1"
            destination_path = "/home/dyaus/video.mp4"

            self.file_transfer_thread = FileTransferThread(self.selected_file, destination_path, hostname, username,
                                                           password)
            self.file_transfer_thread.transfer_complete.connect(self.on_transfer_complete)
            self.file_transfer_thread.start()
        else:
            print("No file selected to send")

    @Slot(str)
    def on_telemetry_received(self, message):

        data = json.loads(message)
        print(data)
        packet_count = data.get("Packet", 100)
        lnln = data.get("Mech_Filter", "0N0N").strip().upper()
        satellite_status = int(data.get("Satellite_Status", "0"))
        error_code = data.get("Error_Codes", ''.join(random.choice('01') for _ in range(5)))
        mission_time = data.get("Mission_Time")
        pressure1 = data.get("Pressure1")
        altitude1 = data.get("Altitude1")
        rp2040_data = data.get("RP2040_Data")
        pressure2 = rp2040_data.get("Pressure2")
        altitude2 = rp2040_data.get("Altitude2")
        altitude_difference = round(data.get("Altitude_Difference", abs(altitude2 - altitude1)), 2)
        descent_rate = data.get("Descent_Rate")
        temperature = data.get("Temperature")
        battery_voltage = rp2040_data.get("Battery_Voltage", 3)
        gps_data = rp2040_data.get("GPS")
        gps_latitude = gps_data.get("Latitude")
        gps_longitude = gps_data.get("Longitude")
        gps_altitude = gps_data.get("Altitude")
        bno_data = data.get("BNO", {})
        pitch = bno_data.get("Pitch")
        roll = bno_data.get("Roll")
        yaw = bno_data.get("Yaw")
        iot_data_raw = data.get("IOT_Data")
        self.main_window.map_widget.update_polyline([gps_latitude, gps_longitude])
        iot_data = float(iot_data_raw)
        raw_data = (f"{packet_count}, {satellite_status}, {error_code}, {mission_time}, {pressure1}, {pressure2},"
                    f" {altitude1}, {altitude2}, {altitude_difference}, {descent_rate}, {temperature}, "
                    f"{battery_voltage}, {gps_latitude}, {gps_longitude}, {gps_altitude}, {yaw}, {pitch}, "
                    f"{roll}, {lnln}, {iot_data}, 335592")
        self.main_window.csv_data.write_csv(raw_data)
        self.main_window.science_payload_widget.update_orientation(roll, pitch, yaw)

        self.main_window.ui.MissionProgressBar.setValue(int(satellite_status))

        self.main_window.ui.RawTelemetryTextEdit.setText(raw_data)
        self.main_window.ui.RawTelemetryTextEdit.setAlignment(Qt.AlignCenter)
        # Update UI with received values
        self.main_window.ui.PacketCountLineEdit.setText(f"Packet Count: {packet_count}")
        self.main_window.ui.SatelliteStatusLineEdit.setText(f"Sat. Status: {satellite_status}")
        self.main_window.ui.MissionStateButton.setText(self.mission_state[satellite_status])
        self.main_window.ui.ErrorCodeLineEdit.setText(f"Err. Code: {error_code}")
        self.main_window.ui.MissionTimeLineEdit.setText(f"Mission Time: {mission_time}")

        # Convert to float and round to 2 decimal places
        self.main_window.ui.Pressure1LineEdit.setText(f"Pres. 1: {pressure1} pa")
        self.main_window.ui.Pressure2LineEdit.setText(f"Pres. 2: {pressure2} pa")
        self.main_window.ui.Altitude1LineEdit.setText(f"Alt. 1: {altitude1} m")
        self.main_window.ui.Altitude2LineEdit.setText(f"Alt. 2: {altitude2} m")
        self.main_window.ui.AltitudeDifferenceLineEdit.setText(f"Alt. Diff.: {altitude_difference} m")
        self.main_window.ui.DescentRate_LineEdit.setText(f"Descent Rate: {descent_rate} m/s")
        self.main_window.ui.TemperatureLineEdit.setText(f"Temp.: {temperature} °C")
        self.main_window.ui.BatteryVoltageLineEdit.setText(f"Battery Volt.: {battery_voltage} V")
        self.main_window.ui.GPSLatitudeLineEdit.setText(f"GPS Lat.: {gps_latitude} °")
        self.main_window.ui.GPSLongitudeLineEdit.setText(f"GPS Lon.: {gps_longitude} °")
        self.main_window.ui.GPSAltitudeLineEdit.setText(f"GPS Alt.: {gps_altitude} m")
        self.main_window.ui.PitchLineEdit.setText(f"Pitch: {yaw} °")
        self.main_window.ui.RollLineEdit.setText(f"Roll: {pitch} °")
        self.main_window.ui.YawLineEdit.setText(f"Yaw: {roll} °")
        self.main_window.ui.LNLNLineEdit.setText(f"LNLN: {lnln}")
        self.main_window.ui.IOTDataLineEdit.setText(f"IOT Data: {iot_data} °C")

        self.main_window.graph_data["TemperatureGraph"].append(temperature)
        self.main_window.graph_data["Pressure2Graph"].append(pressure2)
        self.main_window.graph_data["Pressure1Graph"].append(pressure1)
        self.main_window.graph_data["Altitude2Graph"].append(altitude2)
        self.main_window.graph_data["Altitude1Graph"].append(altitude1)
        self.main_window.graph_data["IOTDataGraph"].append(iot_data)
        error_code = list(error_code)

        button_map = {
            self.main_window.ui.ErrCodeVisual1: error_code[0],
            self.main_window.ui.ErrCodeVisual2: error_code[1],
            self.main_window.ui.ErrCodeVisual3: error_code[2],
            self.main_window.ui.ErrCodeVisual4: error_code[3],
            self.main_window.ui.ErrCodeVisual5: error_code[4],
        }

        # Update colors, text, and font size
        for button, code in button_map.items():
            button_style = "background-color: green; color: black;" if code == '0' else ("background-color: red; "
                                                                                         "color: black;")
            button.setText(code)
            button.setStyleSheet(button_style)

    @Slot(bool, str)
    def on_transfer_complete(self, success, message):
        print(success)
        if success:
            if message:
                try:
                    # Attempt to convert the message to float and update the button text
                    time_taken = round(float(message), 2)
                    self.main_window.ui.SVSP_SendButton.setText(f"Sent in {time_taken}s")
                except ValueError:
                    # Handle the case where conversion to float fails
                    self.main_window.ui.SVSP_SendButton.setText("Sent")
            else:
                # Handle the case where message is an empty string
                self.main_window.ui.SVSP_SendButton.setText("Sent")
        else:
            self.main_window.ui.SVSP_SendButton.setText("Send Failed")
            self.main_window.ui.SVSP_SendButton.setEnabled(True)

    @staticmethod
    async def send_telemetry_init_command(command):
        try:
            ws_uri = "ws://192.168.0.104:9004"
            async with websockets.connect(ws_uri) as websocket:
                try:
                    await websocket.send(command)
                except Exception as e:
                    print(f"Error sending via websocket: {e}")
        except Exception as e:
            print(f"Error connecting to WebSocket: {e}")

        try:

            url = f"http://192.168.0.104/335592_Flight/cmd_pipe.php?cmd=ca%20{1 if command == 'start' else 0}"
            response = requests.post(url)
            print(response)
        except Exception as e:
            print("Error in sending start video record")

    def handle_stop_telemetry(self):
        print("Stopping telemetry receiving process")
        if self.websocket_client:
            self.websocket_client.stop()
            self.websocket_client.wait()
            self.websocket_client = None
            asyncio.run(self.send_telemetry_init_command("stop"))

    def handle_start_telemetry(self, whichButton):
        print("Starting telemetry receiving process")
        if not self.websocket_client:
            self.websocket_client = WebSocketClient("ws://192.168.0.104:9003")
            self.websocket_client.telemetry_received.connect(self.on_telemetry_received)
            self.websocket_client.start()
            if whichButton == "StartButton":
                print("In startbutton process")
                asyncio.run(self.send_telemetry_init_command("start"))

    def handle_release_command(self):
        self.main_window.ReleaseCommand = True
        print("ReleaseCommand received and sent!")

    def handle_connect_arduino(self):
        if self.main_window.ui.ArduinoConnectButton.text() == "Connect":
            self.main_window.ArduinoIsConnected = True
            worker = Worker(self._connect_arduino)
            self.threadpool.start(worker)
        else:
            self._disconnect_arduino()

    def _connect_arduino(self):
        print("Connecting to Arduino")
        port = self.main_window.ui.ComComboBox.currentText()
        baud_rate = self.main_window.ui.PortComboBox.currentText()
        ws_uri = "ws://192.168.0.104:9001"
        try:
            self.iot_thread = IoTDataSendThread(port, baud_rate, ws_uri, self.main_window)
            self.iot_thread.transfer_complete.connect(onIOTTransferComplete)
            self.iot_thread.start()
            self.main_window.ui.ArduinoConnectButton.setText("Disconnect")
            print(f"Connected to Arduino on {port} at {baud_rate} baud")
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")

    def _disconnect_arduino(self):
        if self.iot_thread and self.iot_thread.isRunning():
            self.iot_thread.stop()
            self.iot_thread = None
            self.main_window.ui.ArduinoConnectButton.setText("Connect")
            print("Disconnected from Arduino")

    def handle_filter_code_sending(self):
        async def serve():
            filter_code = self.main_window.ui.lineEdit.text().upper().strip()
            async with websockets.connect('ws://192.168.0.104:9002') as websocket:
                await websocket.send(filter_code)
                self.main_window.ui.lineEdit.clear()
                self.main_window.ui.lineEdit.setPlaceholderText("Sent : " + filter_code)

        asyncio.run(serve())

    def populate_ports(self):
        ports = serial.tools.list_ports.comports()
        self.main_window.ui.ComComboBox.clear()
        for port in ports:
            self.main_window.ui.ComComboBox.addItem(port.device)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.video_widget = None
        self.data_processor_thread = None
        self.graph_timer = None
        self.science_payload_widget = None
        self.map_widget = None
        self.graph_data = None
        self.ui = Ui_MainWindow()
        self.ArduinoIsConnected = False
        self.ReleaseCommand = False
        self.ui.setupUi(self)
        self.setWindowIcon(QIcon("icons/logo.jpg"))  # Update with correct path
        self.setWindowTitle("Team Dyaus | Ground Control Station")
        self.ui.MissionProgressBar.setRange(0, 5)

        self.ui.MissionStateButton.setText("Mission Progress")
        self.button_handler = ButtonHandler(self)
        self.ui.lineEdit.setPlaceholderText("Write the filter code given by jury.")
        self.csv_data = BlackboxCSV()

        self.load_map_frame()
        self.load_science_payload_frame()
        self.load_video_frame()
        self.load_buttons()
        self.load_graph()
        self.load_styles()

        self.button_handler.populate_ports()

    def load_map_frame(self):
        map_layout = QVBoxLayout(self.ui.MapGroupBox)
        self.map_widget = MapWidget()
        self.map_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        map_layout.addWidget(self.map_widget)

    def load_science_payload_frame(self):
        science_layout = QVBoxLayout(self.ui.SciencePayloadGroupBoxWidget)
        self.science_payload_widget = OrientationDisplayWidget()
        self.science_payload_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        science_layout.addWidget(self.science_payload_widget)

    def load_graph(self):
        self.graph_data = {
            "TemperatureGraph": [],
            "Pressure2Graph": [],
            "Pressure1Graph": [],
            "Altitude2Graph": [],
            "Altitude1Graph": [],
            "IOTDataGraph": [],
        }
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.start(3000)  # Update every 3000 milliseconds (3 seconds)

    def update_graphs(self):
        try:
            self.data_processor_thread = DataProcessorThread(self.graph_data)
            self.data_processor_thread.data_processed.connect(self.on_data_processed)
            self.data_processor_thread.start()

        except Exception as e:
            print(f"An error occurred: {e}")

    def on_data_processed(self, processed_data):
        try:
            y_labels = {
                "TemperatureGraph": "Temperature (°C)",
                "Pressure2Graph": "Pressure (Pa)",
                "Pressure1Graph": "Pressure (Pa)",
                "Altitude2Graph": "Altitude (m)",
                "Altitude1Graph": "Altitude (m)",
                "IOTDataGraph": "Temperature (°C)"
            }

            for graph_name, data in processed_data.items():
                getattr(self.ui, graph_name).plot(data)
                getattr(self.ui, graph_name).setLabel('left', y_labels.get(graph_name, 'Value'))
                getattr(self.ui, graph_name).setLabel('bottom', 'Time (s)')
        except Exception as e:
            print(f"An error occurred while plotting the data: {e}")

    def load_video_frame(self):
        video_layout = QVBoxLayout(self.ui.VideoGroupBox)
        self.video_widget = VideoWidget(
            "http://192.168.0.104/335592_Flight/cam_pic.php")
        self.video_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        video_layout.addWidget(self.video_widget)

    def load_buttons(self):
        self.ui.SVSP_ToolButton.clicked.connect(self.button_handler.handle_select_file)
        self.ui.SVSP_SendButton.clicked.connect(self.button_handler.handle_send_file)
        self.ui.ConfigGroupBoxStartButton.clicked.connect(
            lambda: self.button_handler.handle_start_telemetry("StartButton")
        )
        self.ui.ConfigGroupBoxStopButton.clicked.connect(self.button_handler.handle_stop_telemetry)
        self.ui.ArduinoConnectButton.clicked.connect(self.button_handler.handle_connect_arduino)
        self.ui.SendFilterCodeButton.clicked.connect(self.button_handler.handle_filter_code_sending)
        self.ui.SendReleaseCommandButton.clicked.connect(self.button_handler.handle_release_command)
        self.ui.RefreshVIdeoButton.clicked.connect(
            lambda: self.button_handler.handle_start_telemetry("RefreshVideoButton")
        )

    def load_styles(self):
        self.setStyleSheet("""
        /* General Styles */
        QWidget {
            background-color: #08122C;
            color: #F0E9D2;
            font-family: "Roboto", sans-serif;
            font-size: 8pt;
        }

        /* Buttons */
        QPushButton {
            background-color: #0D1B45;
            color: #F0E9D2;
            border: 1px;
            padding: 6px 10px;
            border-radius: 4px;
        }

        QPushButton:hover {
            background-color: #142A73;
        }

        QPushButton:pressed {
            background-color: #1B3B9C;
        }

        /* Labels */
        QLabel {
            color: #F0E9D2;
        }

        /* Line Edits */
        QLineEdit {
            background-color: #0D1B45;
            color: #F0E9D2;
            border: none;
            padding: 6px 12px;
            border-radius: 4px;
        }

        QLineEdit:focus {
            border: 1px solid #1B3B9C;
        }

        /* ComboBoxes */
        QComboBox {
            background-color: #0D1B45;
            color: #F0E9D2;
            border: 1px;
            padding: 6px 12px;
            border-radius: 2px;
        }

        QComboBox::drop-down {
            border: none;
        }

        QComboBox QAbstractItemView {
            background-color: #0D1B45;
            color: #F0E9D2;
            selection-background-color: #1B3B9C;
        }

        /* Scroll Bars */
        QScrollBar:vertical {
            background-color: #0D1B45;
            width: 12px;
            margin: 0;
        }

        QScrollBar::handle:vertical {
            background-color: #142A73;
            min-height: 20px;
            border-radius: 4px;
        }

        QScrollBar::handle:vertical:hover {
            background-color: #1B3B9C;
        }

        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
            height: 0;
            width: 0;
        }

        /* Progress Bar */
        QProgressBar {
            background-color: #0D1B45;
            border: 1px solid #08122C;
            border-radius: 4px;
            text-align: center;
        }

        QProgressBar::chunk {
            background-color: #1B3B9C;
            border-radius: 4px;
        }

        /* Tooltips */
        QToolTip {
            background-color: #142A73;
            color: #F0E9D2;
            border: 1px solid #08122C;
            padding: 4px;
            border-radius: 4px;
        }

        /* Group Boxes */
        QGroupBox {
            background-color: #08122C;
            border: 0.5px solid #F0E9D2;
            border-radius: 4px;
            margin-top: 20px;
        }

        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top center;
            padding: 0px 3px;
            color: #F0E9D2;
        }

        /* Tabs */
        QTabWidget::pane {
            border: 1px solid #142A73;
        }

        QTabBar::tab {
            background-color: #0D1B45;
            color: #F0E9D2;
            border: 1px solid #08122C;
            padding: 6px 12px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }

        QTabBar::tab:selected {
            background-color: #1B3B9C;
        }

        QTabBar::tab:hover {
            background-color: #142A73;
        }
    """)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
