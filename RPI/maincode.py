import asyncio
import csv
import json
import logging
import os
import queue
import random
import shutil
import threading
import time
from pathlib import Path
from typing import Any

import RPi.GPIO as GPIO
import adafruit_bmp3xx
import adafruit_bno055
import board
import busio
import pigpio
import serial
import websockets
from adafruit_ds3231 import DS3231
from adafruit_tmp117 import TMP117
from flask import Flask, Response, send_from_directory, jsonify, render_template

LOG_FILE = "/home/dyaus/telemetry.log"

logger = logging.getLogger(__name__)

logger.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(message)s')

file_handler = logging.FileHandler(LOG_FILE)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)


class Sensor:
    def __init__(self, name: str, use_test: bool = True, delay_time: float = 1.0) -> None:
        self.name = name
        self.use_test = use_test
        self.delay_time = delay_time
        self.queue: queue.Queue[Any] = queue.Queue()
        self.active = False
        self.thread = None
        self.stop_event = threading.Event()  # Event to signal thread to stop

    def log(self, message: str, level: str = "info") -> None:
        log_function = getattr(logger, level.lower(), logger.info)
        log_entry = f"{self.name} | {message}"
        log_function(log_entry)

    @staticmethod
    def put_data_in_queue(data_queue: queue.Queue, data: Any) -> None:
        data_queue.queue.clear()
        data_queue.put(data)

    def worker(self) -> None:
        raise NotImplementedError("Sensor subclass must implement the worker method.")

    def start(self):
        if not self.active:
            self.active = True
            self.stop_event.clear()  # Clear the stop event before starting the thread
            self.thread = threading.Thread(target=self.worker)
            self.thread.start()
            self.log("Sensor started", "info")

    def stop(self):
        self.log("Stopping Sensor", "info")
        if self.active:
            self.active = False
            self.queue.queue.clear()
            self.stop_event.set()  # Set the event to signal the thread to stop
            self.thread.join()  # Wait for the thread to finish
            self.log("Sensor stopped", "info")


class BMP390Sensor(Sensor):
    def __init__(self, name: str, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)
        if not use_test:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bmp = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)
        self.initial_pressure = None
        self.initial_altitude = 0
        self.load_initial_pressure()

    def load_initial_pressure(self):
        try:
            if os.path.isfile("/home/dyaus/packet_count.txt"):
                with open("/home/dyaus/packet_count.txt", "r") as f:
                    content = f.read().strip().split(",")
                    if not int(content[1]) == 0:
                        self.initial_pressure = float(content[1])
                        self.log(f"Loaded initial pressure: {self.initial_pressure}", "info")

                    else:
                        self.log("Initial pressure in file is 0, using default behavior.", "info")
            else:
                self.log("Initial pressure file not found, using default behavior.", "info")

        except Exception as e:
            self.log(f"Error loading initial pressure: {e}", "error")
            self.initial_pressure = None

    def get_pressure(self) -> float:
        try:
            pressure = self.bmp.pressure * 100
            return pressure
        except Exception as e:
            self.log(f"Error getting pressure: {e}", "error")

    def get_pressure_test(self) -> float:
        try:
            return random.uniform(900, 1100)
        except Exception as e:
            self.log(f"Error generating random pressure: {e}", "error")

    def get_altitude(self, pressure: float) -> float:
        try:
            if self.initial_pressure is None:
                self.initial_pressure = pressure
            return 44307.7 * (1 - (pressure / self.initial_pressure) ** 0.190284)
        except Exception as e:
            self.log(f"Error getting altitude: {e}", "error")

    def get_altitude_test(self) -> float:
        if not hasattr(self, 'pressure_passed'):
            self.pressure_passed = self.initial_pressure >= 600

        if self.pressure_passed:
            self.initial_altitude = max(0, self.initial_altitude - random.uniform(30, 40))
        else:
            self.initial_altitude += random.uniform(30, 40)

        return round(self.initial_altitude, 2)

    def worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.use_test:
                    pressure = self.get_pressure_test()
                    altitude = self.get_altitude_test()
                else:
                    pressure = self.get_pressure()
                    altitude = self.get_altitude(pressure)

                if pressure is not None:
                    self.put_data_in_queue(PRESSURE_QUEUE, round(pressure, 2))
                if altitude is not None:
                    self.put_data_in_queue(ALTITUDE_QUEUE, round(altitude, 2))

                time.sleep(self.delay_time)
            except Exception as e:
                self.log(f"Error in {self.name} worker: {e}", "error")


class TMP117Sensor(Sensor):
    def __init__(self, name: str, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)
        self.previous_temperature = None

        if not use_test:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.tmp117 = TMP117(self.i2c, address=0x48)
            except Exception as e:
                self.log(f"Error initializing TMP117 sensor: {e}", "error")
                self.tmp117 = None

    def get_temperature(self) -> float:
        if self.tmp117 is None:
            self.log("TMP117 sensor not initialized.", "error")

        try:
            temperature = self.tmp117.temperature

            return temperature
        except Exception as e:
            self.log(f"Error getting temperature: {e}", "error")

    def get_temperature_test(self) -> float:
        try:
            return random.uniform(-20, 40)
        except Exception as e:
            self.log(f"Error generating random temperature: {e}", "error")

    def worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                temperature = self.get_temperature_test() if self.use_test else self.get_temperature()

                if temperature is not None:
                    self.put_data_in_queue(TEMPERATURE_QUEUE, round(temperature, 2))

                time.sleep(self.delay_time)
            except Exception as e:
                self.log(f"Error in {self.name} worker: {e}", "error")


class RP2040(Sensor):
    def __init__(self, name: str, rp2040_serial_manager, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)
        self.initial_pressure = None
        self.rp2040_serial_manager = rp2040_serial_manager
        self.initial_altitude = 0
        # end_time = time.time()+ 5  # Set end time 5 seconds from start

        # while time.time() < end_time:
        #     self.rp2040_serial_manager.read_data()

        # self.log("RP2040 Garbage Dump Completed")

        self.load_initial_pressure()

    def load_initial_pressure(self) -> None:
        try:
            if os.path.isfile("/home/dyaus/packet_count.txt"):
                with open("/home/dyaus/packet_count.txt", "r") as f:
                    content = f.read().strip().split(",")
                    if not int(content[2]) == 0:
                        self.initial_pressure = float(content[2])
                        self.log(f"Loaded initial pressure: {self.initial_pressure}", "info")
                    else:
                        self.log("Initial pressure in file is 0, using default behavior.", "info")
            else:
                self.log("Initial pressure file not found, using default behavior.", "info")
        except Exception as e:
            self.log(f"Error loading initial pressure: {e}", "error")
            self.initial_pressure = None

    def get_rp2040_data(self) -> dict:
        try:
            data = self.rp2040_serial_manager.read_data()
            self.log(data)
            print(data)

            if data:
                data = data.split(",")

                pressure = data[0]

                gps_data = {
                    "Latitude": data[1],
                    "Longitude": data[2],
                    "Altitude": data[3],
                }

                battery_voltage = data[4]
                try:
                    pressure = float(pressure)

                    if self.initial_pressure is None and int(pressure) > 0:
                        self.initial_pressure = pressure
                        print(f"Initial Pressure set to {pressure}.")
                except ValueError as e:
                    self.log(f"Invalid pressure value: {pressure} - {e}", "error")
                    pressure = 0

                try:
                    altitude = 44307.7 * (1 - (pressure / self.initial_pressure) ** 0.190284)
                except (ZeroDivisionError, TypeError) as e:
                    self.log(f"Error in altitude calculation: initial pressure is zero or invalid - {e}", "error")
                    altitude = 0

                return {
                    "Pressure2": round(pressure, 2),
                    "GPS": gps_data,
                    "Battery_Voltage": battery_voltage,
                    "Altitude2": round(altitude, 2)
                }

        except Exception as e:
            self.log(f"Error getting data from RP2040: {e}", "error")

    def get_rp2040_data_test(self) -> dict:
        try:
            self.pressure_passed = self.initial_pressure >= 600
            if self.pressure_passed and self.initial_altitude > 0:
                self.initial_altitude = max(0, self.initial_altitude - random.uniform(30, 40))
            else:
                self.initial_altitude += random.uniform(30, 40)

            pressure = random.uniform(900, 1100)
            gps_data = {
                "Latitude": round(random.uniform(-90, 90), 6),
                "Longitude": round(random.uniform(-180, 180), 6),
                "Altitude": random.uniform(-100, 1000),
                "Satellite_Status": 0
            }
            battery_voltage = round(random.uniform(3.0, 4.2), 2)

            return {
                "Pressure2": round(pressure, 2),
                "GPS": gps_data,
                "Battery_Voltage": battery_voltage,
                "Altitude2": round(self.initial_altitude, 2)
            }
        except Exception as e:
            self.log(f"Error generating test data for RP2040: {e}", "error")

    def worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.use_test:
                    received_rp2040_data = self.get_rp2040_data_test()
                else:
                    received_rp2040_data = self.get_rp2040_data()

                if received_rp2040_data is not None:
                    self.put_data_in_queue(RP2040_QUEUE, received_rp2040_data)

                time.sleep(self.delay_time)
            except Exception as e:
                self.log(f"Error in {self.name} worker: {e}", "error")


class BNOSensor(Sensor):
    def __init__(self, name: str, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)
        if not use_test:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.bno_sensor = adafruit_bno055.BNO055_I2C(self.i2c, address=0x28)
                self.bno_sensor.mode = 0x0C
                self.log(f"BNO055 initialized in NDOF mode.", "info")
            except Exception as e:
                self.log(f"Error initializing BNO055 sensor: {e}", "error")
                self.bno_sensor = None

    def get_bno(self) -> dict:
        if self.bno_sensor is None:
            self.log("BNO055 sensor not initialized.", "error")

        try:
            euler_angles = self.bno_sensor.euler

            # print(euler_angles)
            if euler_angles is None:
                raise ValueError("No data from BNO055 sensor.")

            bno_data = {
                "Roll": round(euler_angles[0], 2),
                "Pitch": round(euler_angles[1], 2),
                "Yaw": round(euler_angles[2], 2)
            }
            return bno_data
        except Exception as e:
            self.log(f"Error getting BNO data: {e}", "error")

    def get_bno_test(self) -> dict:
        try:
            return {
                "Roll": round(random.uniform(0, 360), 2),
                "Pitch": round(random.uniform(-90, 90), 2),
                "Yaw": round(random.uniform(-180, 180), 2)
            }
        except Exception as e:
            self.log(f"Error generating random BNO data: {e}", "error")

    def worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                if self.use_test:
                    bno_data = self.get_bno_test()
                else:
                    bno_data = self.get_bno()

                if bno_data is not None:
                    self.put_data_in_queue(BNO_QUEUE, bno_data)
                time.sleep(self.delay_time)
            except Exception as e:
                self.log(f"Error in {self.name} worker: {e}", "error")


class IOTDataReceiver(Sensor):
    def __init__(self, name: str, iot_serial_manager, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)
        self.websocket_task = None
        self.previous_pressure_data = 0
        self.iot_serial_manager = iot_serial_manager

    async def process_command(self, websocket):
        try:
            async for message in websocket:
                iot_data, trigger_burnwire = message.strip().split(",")
                self.log(f"{iot_data}, {trigger_burnwire}")
                print(iot_data, trigger_burnwire)
                if trigger_burnwire.lower() == "true":
                    print("Burning!")
                    self.log("Burning!")
                    self.iot_serial_manager.write_data("a")
                    self.log("Burnt Manually!")
                    print("Burnt Manually!")
                await self.handle_command(iot_data)
        except websockets.ConnectionClosed:
            print("Connection closed unexpectedly")
        except websockets.InvalidHandshake:
            print("Invalid handshake with server")
        except ConnectionRefusedError:
            print("Connection refused")

    async def handle_command(self, message):
        with IOT_DATA_QUEUE.mutex:
            IOT_DATA_QUEUE.queue.clear()
        IOT_DATA_QUEUE.put(message)
        self.previous_pressure_data = message

    async def periodic_check(self):
        while True:
            await asyncio.sleep(1)
            if IOT_DATA_QUEUE.empty():
                IOT_DATA_QUEUE.put(self.previous_pressure_data)

    async def serve(self):
        host = "0.0.0.0"
        port = 9001
        try:
            start_server = await websockets.serve(self.process_command, host, port)
            self.log(f"IOTDataReceiver WebSocket server started at {host}:{port}")
            await start_server.wait_closed()
        except Exception as e:
            self.log(f"Error in WebSocket server: {e}", "error")

    def worker(self) -> None:
        async def run():
            await asyncio.gather(self.serve(), self.periodic_check())

        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.run(run())


class MechFilterServer(Sensor):
    CONTROL_PIN = 13  # Pin 11
    FEEDBACK_PIN = 6  # Pin 12
    UNITS_FULL_CIRCLE = 360
    DUTY_SCALE = 1000
    DC_MIN = 29
    DC_MAX = 971
    Q2_MIN = UNITS_FULL_CIRCLE // 4
    Q3_MAX = Q2_MIN * 3
    TURN_THRESHOLD = 4
    DEADBAND = 5  # Adjust this value as necessary

    def __init__(self, name: str, use_test: bool = True, delay_time: float = 1.0) -> None:
        super().__init__(name, use_test, delay_time)

        self.dyaus = pigpio.pi()

        if not self.dyaus.connected:
            print("Failed to connect to pigpio daemon. Please ensure it is running.")
            exit()

        self.angle = 0
        self.target_angle = 0
        self.returning_to_neutral = False
        self.neutral_reached = False
        self.last_error = 0
        self.integral = 0
        self.t_high = 0
        self.t_low = 0
        self.last_tick = 0
        self.direction = 1
        self.stop_rot = False

        self.setup_servo()
        self.feedback_thread = threading.Thread(target=self.feedback360)
        self.control_thread = threading.Thread(target=self.control360)
        self.feedback_thread.start()
        self.control_thread.start()

    def setup_servo(self):
        self.dyaus.set_mode(self.CONTROL_PIN, pigpio.OUTPUT)
        self.dyaus.set_mode(self.FEEDBACK_PIN, pigpio.INPUT)
        self.dyaus.set_PWM_range(self.CONTROL_PIN, 3000)
        self.dyaus.callback(self.FEEDBACK_PIN, pigpio.EITHER_EDGE, self.cbf)

    def cbf(self, gpio, level, tick):
        if level == 1:  # Rising edge
            self.t_low = pigpio.tickDiff(self.last_tick, tick)
            self.last_tick = tick
        elif level == 0:  # Falling edge
            self.t_high = pigpio.tickDiff(self.last_tick, tick)
            self.last_tick = tick

    def feedback360(self):
        turns = 0
        theta_p = 0

        while True:
            t_cycle = self.t_high + self.t_low
            if 1000 < t_cycle < 1200:
                dc = (self.DUTY_SCALE * self.t_high) / t_cycle
                theta = (self.UNITS_FULL_CIRCLE - 1) - ((dc - self.DC_MIN) * self.UNITS_FULL_CIRCLE) / (
                        self.DC_MAX - self.DC_MIN + 1)
                theta = max(0, min(self.UNITS_FULL_CIRCLE - 1, theta))

                if theta < self.Q2_MIN and theta_p > self.Q3_MAX:
                    turns += 1
                elif theta_p < self.Q2_MIN and theta > self.Q3_MAX:
                    turns -= 1

                if turns >= 0:
                    self.angle = (turns * self.UNITS_FULL_CIRCLE) + theta
                else:
                    self.angle = ((turns + 1) * self.UNITS_FULL_CIRCLE) - (self.UNITS_FULL_CIRCLE - theta)

                theta_p = theta

            time.sleep(0.01)  # Adjust this delay as needed

    def control360(self):
        while True:
            if self.neutral_reached:
                continue

            current_angle = self.angle
            error = self.target_angle - current_angle

            # Check if within deadband range
            if abs(error) < self.DEADBAND:
                if self.returning_to_neutral:
                    self.returning_to_neutral = False
                    self.neutral_reached = True
                    self.dyaus.set_servo_pulsewidth(self.CONTROL_PIN, 1500)  # Stop the servo
                    continue

            # Proportional term
            proportional = 0.75 * error  # Kp = 1.0

            # Integral term with anti-windup
            self.integral += 0 * error  # Ki = 0
            if self.integral > 200:
                self.integral = 200
            elif self.integral < -200:
                self.integral = -200
            integral_term = self.integral

            # Derivative term
            derivative = 0 * (error - self.last_error)  # Kd = 0
            self.last_error = error

            # Calculate output
            output = proportional + integral_term + derivative

            # Limit output to servo PWM range (1000 to 2000)
            servo_pulse = 1500 + int(output)
            if servo_pulse < 1000:
                servo_pulse = 1000
            elif servo_pulse > 2000:
                servo_pulse = 2000

            # Set servo pulse width
            self.dyaus.set_servo_pulsewidth(self.CONTROL_PIN, servo_pulse)
            time.sleep(0.02)  # Repeat after 20 ms

    def move_servo_90_degrees(self):
        self.neutral_reached = False
        if self.direction == 1:
            self.target_angle += 90
        else:
            self.target_angle -= 90

    def return_to_neutral(self):
        self.neutral_reached = False
        current_angle = self.angle % self.UNITS_FULL_CIRCLE

        if self.direction == 1:
            self.target_angle = self.angle + (360 - current_angle)
        else:
            self.target_angle = self.angle - current_angle

        self.returning_to_neutral = True

    async def process_command(self, websocket):
        try:
            async for message in websocket:
                await self.handle_command(message.strip())
        except websockets.ConnectionClosed:
            print("Connection closed unexpectedly")
        except websockets.InvalidHandshake:
            print("Invalid handshake with server")
        except ConnectionRefusedError:
            print("Connection refused")

    async def handle_command(self, command):
        if len(command) != 4:
            return

        duration1 = int(command[0])
        filter_color_1 = command[1]
        duration2 = int(command[2])
        filter_color_2 = command[3]

        with MECH_FILTER_QUEUE.mutex:
            MECH_FILTER_QUEUE.queue.clear()
        MECH_FILTER_QUEUE.put(command)

        if self.use_test:
            self.log(f"Test mode: Data received - {duration1}, {filter_color_1}, {duration2}, {filter_color_2}")
            pass
        else:
            await self.rotate_servo(duration1, filter_color_1)
            self.return_to_neutral()
            await self.rotate_servo(duration2, filter_color_2)
            self.return_to_neutral()

    async def rotate_servo(self, duration: float, filter_color: str) -> None:
        rotations = {'R': 1, 'G': 2, 'B': 3}
        for _ in range(rotations.get(filter_color, 0)):
            self.move_servo_90_degrees()

        await asyncio.sleep(duration)

    async def serve(self):
        host = "0.0.0.0"
        port = 9002
        try:
            start_server = await websockets.serve(self.process_command, host, port)
            self.log(f"MechFilterServer WebSocket server started at {host}:{port}")
            await start_server.wait_closed()
        except Exception as e:
            self.log(f"Error in WebSocket server: {e}", "error")

    @staticmethod
    async def periodic_check():
        while True:
            await asyncio.sleep(1)
            if MECH_FILTER_QUEUE.empty():
                MECH_FILTER_QUEUE.put("0N0N")
            # Implement your periodic check logic here

    def worker(self) -> None:
        async def run():
            await asyncio.gather(self.serve(), self.periodic_check())

        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.run(run())


FSW_STATE = {
    'READY_TO_FLIGHT': 0,
    'ASCENT': 1,
    'MODEL_SATELLITE_DESCENT': 2,
    'RELEASE': 3,
    'SCIENCE_PAYLOAD_DESCENT': 4,
    'RECOVERY': 5
}


class BlackboxCSV:
    def __init__(self, rtc_time=None, folder_path="/home/dyaus/335592_BlackBox_RPI"):
        self.folder_path = Path(folder_path)
        self.folder_path.mkdir(parents=True, exist_ok=True)
        (self.folder_path / "Packet Count Files").mkdir(parents=True, exist_ok=True)
        timestamp = self._get_timestamp(rtc_time)

        if os.path.isfile("/home/dyaus/packet_count.txt"):
            with open("/home/dyaus/packet_count.txt", "r") as f:
                self.check_contents = f.read().strip().split(",")
            if len(self.check_contents) == 4:
                print("Skipping creating backup of CSV file since restart was detected!")
                return
            else:
                self.backup_file("335592_csv_data.csv", "Backup CSV Files", timestamp)
        else:
            self.backup_file("335592_csv_data.csv", "Backup CSV Files", timestamp)

    def write_csv(self, raw_data, filename="335592_csv_data.csv"):
        file_path = self.folder_path / filename
        header = [
            'Packet_Count', 'Satellite_Status', 'Error_Code', 'Mission_Time', 'Pressure1', 'Pressure2',
            'Altitude1', 'Altitude2', 'Altitude_Difference', 'Descent_Rate', 'Temperature', 'Battery_Voltage',
            'Gps_Latitude', 'Gps_Longitude', 'Gps_Altitude', 'Pitch', 'Roll', 'Yaw', 'LNLN', 'IOT_Data',
            'Team_Number'
        ]

        write_header = not file_path.exists()

        try:
            with file_path.open("a", newline="") as f:
                writer = csv.writer(f, delimiter=",")
                if write_header:
                    writer.writerow(header)
                writer.writerow(raw_data.split(','))
        except OSError as e:
            print(f"OS error writing data into CSV: {e}")
        except Exception as e:
            print(f"Error writing data into CSV: {e}")

    def backup_file(self, filename, backup_folder_name, timestamp):
        file_path = self.folder_path / filename
        if not file_path.exists():
            print(f"File {filename} does not exist. Skipping backup.")
            return

        backup_folder = self.folder_path / backup_folder_name
        backup_folder.mkdir(parents=True, exist_ok=True)

        new_file_name = f"{timestamp}_{filename}"
        try:
            shutil.copy(file_path, backup_folder / new_file_name)
            file_path.unlink()  # Delete original file after backup
            print(f"Backup created: {backup_folder / new_file_name}")
        except FileNotFoundError as e:
            print(f"File not found for backup: {e}")
        except PermissionError as e:
            print(f"Permission error during backup: {e}")
        except Exception as e:
            print(f"Error creating backup for {filename}: {e}")

    @staticmethod
    def _get_timestamp(rtc_time):
        if rtc_time is None:
            t = time.localtime()
        else:
            try:
                t = rtc_time.datetime
            except AttributeError:
                t = rtc_time
        return f"{t.tm_mday}_{t.tm_mon}_{t.tm_year}_{t.tm_hour}_{t.tm_min}_{t.tm_sec}"


class DataTransmitter(Sensor):
    def __init__(self, name: str, use_test: bool, delay_time: float = 1.0, *queues) -> None:
        super().__init__(name, use_test, delay_time)
        if not use_test:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.rtc = DS3231(i2c)
            t = self.rtc.datetime
            self.log(f"RTC : {t.tm_mday}/{t.tm_mon}/{t.tm_year}; {t.tm_hour}/{t.tm_min}/{t.tm_sec}")
        else:
            self.rtc = time.localtime()
        self.previous_roll = None
        self.alarm_codes = None
        self.queues = queues
        self.websocket_clients = set()
        self.Recovery_Time = None

        self.packet_count_file = "/home/dyaus/packet_count.txt"
        if os.path.isfile("/home/dyaus/packet_count.txt"):
            with open("/home/dyaus/packet_count.txt", "r") as f:
                self.current_state = f.read().strip().split(",")[3]
        else:
            self.current_state = FSW_STATE['READY_TO_FLIGHT']
        self.csv_data = BlackboxCSV(self.rtc)
        self.packet_count = None
        self.load_packet_count()

    def load_packet_count(self):
        try:
            if os.path.isfile("/home/dyaus/packet_count.txt"):
                with open("/home/dyaus/packet_count.txt", "r") as f:
                    self.packet_count = int(f.read().strip().split(",")[0])
                    print("Inside Load Packet Count", self.packet_count)
                    self.log(f"Loaded Packet Count: {self.packet_count}", "info")
            else:
                self.log("Initial Packet Count not found, using default behavior.", "info")
                self.packet_count = 0
        except Exception as e:
            self.log(f"Error loading packet count: {e}", "error")

    async def handle_connection(self, websocket):
        self.websocket_clients.add(websocket)
        try:
            await websocket.wait_closed()
        except Exception as e:
            self.log(f"Error in handle_connection: {e}")
        finally:
            self.websocket_clients.remove(websocket)

    # @staticmethod
    # def get_temperature_data():
    #     try:
    #         temps = psutil.sensors_temperatures()
    #         if not temps:
    #             return "No valid temperature data found"
    #         return {name: {entry.label: entry.current for entry in entries if entry.current is not None}
    #                 for name, entries in temps.items()}
    #     except AttributeError:
    #         return "Temperature data not supported on this system"

    async def transmit_data(self):
        while not self.stop_event.is_set():
            try:
                data = {"Packet": self.packet_count}

                # Gather sensor data
                for sensor_name, sensor_queue in zip(SENSOR_NAMES, self.queues):
                    try:
                        data[sensor_name] = sensor_queue.get()
                    except Exception as e:
                        self.log(f"Error fetching data for {sensor_name}: {e}")

                # memory = psutil.virtual_memory()
                # swap = psutil.swap_memory()
                # cpu_percent = psutil.cpu_percent(interval=1)
                # disk_usage = psutil.disk_usage('/')
                # net_io = psutil.net_io_counters()
                # temperature_info = self.get_temperature_data()

                # data["SystemInfo"] = {
                #     "CPU_Usage": f"{cpu_percent}%",
                #     "Memory": {
                #         "Total": f"{memory.total / (1024 ** 3):.2f} GB",
                #         "Used": f"{memory.used / (1024 ** 3):.2f} GB",
                #         "Free": f"{memory.available / (1024 ** 3):.2f} GB",
                #         "Percent_Used": f"{memory.percent}%"
                #     },
                #     "Swap_Memory": {
                #         "Total": f"{swap.total / (1024 ** 3):.2f} GB",
                #         "Used": f"{swap.used / (1024 ** 3):.2f} GB",
                #         "Free": f"{swap.free / (1024 ** 3):.2f} GB",
                #         "Percent_Used": f"{swap.percent}%"
                #     },
                #     "Disk": {
                #         "Total": f"{disk_usage.total / (1024 ** 3):.2f} GB",
                #         "Used": f"{disk_usage.used / (1024 ** 3):.2f} GB",
                #         "Free": f"{disk_usage.free / (1024 ** 3):.2f} GB",
                #         "Percent_Used": f"{disk_usage.percent}%"
                #     },
                #     "Network": {
                #         "Bytes_Sent": f"{net_io.bytes_sent / (1024 ** 2):.2f} MB",
                #         "Bytes_Received": f"{net_io.bytes_recv / (1024 ** 2):.2f} MB",
                #         "Packets_Sent": net_io.packets_sent,
                #         "Packets_Received": net_io.packets_recv
                #     },
                #     "Temperature": temperature_info
                # }

                # Save initial data to file on first packet
                if self.packet_count == 0:
                    self.save_to_file([data.get("Packet"), data.get("Pressure1"),
                                       data.get("RP2040_Data").get("Pressure2"), int(self.current_state)])

                # Update alarm codes and calculate descent rate
                self.alarm_codes, descent_rate = update_alarm_code(data, int(self.current_state))
                descent_rate = round(descent_rate, 2)
                data["Descent_Rate"] = descent_rate

                # Fetch mission time from RTC
                try:
                    t = self.rtc.datetime
                    data["Mission_Time"] = f"{t.tm_mday}/{t.tm_mon}/{t.tm_year}; {t.tm_hour}:{t.tm_min}:{t.tm_sec}"
                    self.log(f"RTC {data['Mission_Time']}")
                except:
                    t = self.rtc
                    data["Mission_Time"] = f"{t.tm_mday}/{t.tm_mon}/{t.tm_year}; {t.tm_hour}:{t.tm_min}:{t.tm_sec}"
                    self.log(f"Date {data['Mission_Time']}")

                # Calculate altitude difference
                altitude1 = data.get("Altitude1")
                altitude2 = data.get("RP2040_Data").get("Altitude2")
                data["Altitude_Difference"] = round(abs(altitude1 - altitude2), 2) if altitude2 is not None else None

                # Add error codes to data
                data["Error_Codes"] = ''.join(map(str, self.alarm_codes))

                # Calculate roll difference
                current_roll = data.get("BNO").get("Roll")
                if current_roll is not None:
                    roll_diff = abs(self.previous_roll - current_roll) if self.previous_roll is not None else None
                    self.previous_roll = current_roll
                else:
                    roll_diff = None

                # Update satellite status
                data["Satellite_Status"] = update_state(self.current_state, altitude1, descent_rate,
                                                        int(data["Altitude_Difference"]), roll_diff)

                # Update state if changed
                if data["Satellite_Status"] != self.current_state:
                    self.current_state = data["Satellite_Status"]

                # Prepare CSV data
                raw_data_packet_for_csv = [
                    data.get('Packet'),
                    data.get('Satellite_Status'),
                    data.get('Error_Codes'),
                    data.get('Mission_Time'),
                    data.get('Pressure1'),
                    data.get('RP2040_Data').get('Pressure2'),
                    data.get('Altitude1'),
                    data.get('RP2040_Data').get('Altitude2'),
                    data.get('Altitude_Difference'),
                    data.get('Descent_Rate'),
                    data.get('Temperature'),
                    data.get('RP2040_Data').get('Battery_Voltage'),
                    data.get('RP2040_Data').get('GPS').get('Latitude'),
                    data.get('RP2040_Data').get('GPS').get('Longitude'),
                    data.get('RP2040_Data').get('GPS').get('Altitude'),
                    data.get('BNO').get('Pitch'),
                    data.get('BNO').get('Roll'),
                    data.get('BNO').get('Yaw'),
                    data.get("Mech_Filter"),
                    data.get('IOT_Data'),
                    335592
                ]
                self.csv_data.write_csv(','.join(map(str, raw_data_packet_for_csv)))

                # Handle recovery state
                if self.current_state == FSW_STATE["RECOVERY"]:
                    if self.Recovery_Time is None:
                        self.Recovery_Time = time.time() + 30  # Set recovery time to 30 seconds from now

                    if time.time() >= self.Recovery_Time:  # Compare current time with Recovery_Time
                        await self.send_stop_command()

                self.log(f"Data sent: {data}")
                await asyncio.gather(
                    *[self.send_data_to_client(websocket, data) for websocket in self.websocket_clients]
                )

                self.packet_count += 1
                self.save_to_file([self.packet_count])

                await asyncio.sleep(self.delay_time)
            except Exception as e:
                self.log(f"Unexpected error in transmit_data loop: {e}")

    async def send_data_to_client(self, websocket, data):
        try:
            await websocket.send(json.dumps(data))
        except websockets.ConnectionClosed:
            self.websocket_clients.remove(websocket)
            self.log("Connection closed unexpectedly")
        except websockets.InvalidHandshake:
            self.log("Invalid handshake with server")
        except ConnectionRefusedError:
            self.log("Connection refused")
        except Exception as e:
            self.log(f"Error sending data to websocket: {e}")

    async def schedule_stop_command(self):
        await self.send_stop_command()

    async def send_stop_command(self):
        uri = f"ws://0.0.0.0:9004"
        async with websockets.connect(uri) as websocket:
            stop_command = "stop"
            await websocket.send(stop_command)
            self.log(f"Sent stop command to {uri}")

    def start_server(self):
        async def serve():
            host = "0.0.0.0"
            port = 9003
            self.server = await websockets.serve(self.handle_connection, host, port)
            self.log(f"DataTransmitter WebSocket server started at {host}:{port}")
            await self.transmit_data()

        asyncio.run(serve())

    def save_to_file(self, data: list) -> None:
        if len(data) == 4:
            with open(self.packet_count_file, "w") as f:
                _write_data = ','.join(map(str, data))
                f.write(_write_data)
        else:
            with open(self.packet_count_file, "r") as f:
                _data = f.read().strip().split(",")

            if len(_data) == 4:
                _data[0] = str(data[0])
                with open(self.packet_count_file, "w") as f:
                    _write_data = ','.join(_data)
                    f.write(_write_data)
            else:
                print("File content is not in the expected format.")

    def worker(self) -> None:
        self.start_server()


def update_state(current_state, altitude, velocity, alt_diff, roll_diff):
    # Ready to Flight -> Ascent
    if current_state == FSW_STATE['READY_TO_FLIGHT']:
        if 10 < int(altitude) < 700:
            return FSW_STATE['ASCENT']

    # Ascent -> Model Satellite Descent
    elif current_state == FSW_STATE['ASCENT']:
        if int(velocity) < -5 and int(altitude) > 450:
            return FSW_STATE['MODEL_SATELLITE_DESCENT']

    # Model Satellite Descent -> Release
    elif current_state == FSW_STATE['MODEL_SATELLITE_DESCENT']:
        if int(altitude) < 450 and int(alt_diff) > 25:
            return FSW_STATE['RELEASE']

    # Release -> Science Payload Descent
    elif current_state == FSW_STATE['RELEASE']:
        if int(altitude) < 400 and int(velocity) < -3:
            return FSW_STATE['SCIENCE_PAYLOAD_DESCENT']

    # Sc00ience Payload Descent -> Recovery
    elif current_state == FSW_STATE['SCIENCE_PAYLOAD_DESCENT']:
        if float(abs(roll_diff)) < 0.2 and int(altitude) < 20:
            return FSW_STATE['RECOVERY']

    return current_state


def update_alarm_code(data, state):
    # Alarm code indices
    CONTAINER_LANDING_ALARM = 0
    SCIENCE_PAYLOAD_LANDING_ALARM = 1
    PRESSURE_SENSOR_ALARM = 2
    GPS_DATA_ALARM = 3
    ALTITUDE_DIFFERENCE_ALARM = 4

    # Initialize alarm codes
    alarm_codes = [0] * 5

    # Extract necessary data
    rp2040_data = data.get("RP2040_Data")
    container_altitude = rp2040_data.get("Altitude2")
    container_pressure = rp2040_data.get("Pressure2")
    gps_data = rp2040_data.get("GPS")
    science_payload_altitude = data.get("Altitude1")

    altitude_difference = (abs(science_payload_altitude - container_altitude)
                           if science_payload_altitude is not None and container_altitude is not None
                           else None)

    current_time = time.time()

    if not hasattr(update_alarm_code, "previous_data"):
        update_alarm_code.previous_data = {
            "container_altitude": None,
            "container_time": None,
            "science_payload_altitude": None,
            "science_payload_time": None,
            "science_payload_landing_rate": None,
            "container_landing_rate": None,
            "altitude_difference_checked": False  # Flag to track if altitude difference check has been done
        }

    previous_data = update_alarm_code.previous_data

    def update_landing_rate(payload_type, current_altitude, min_rate, max_rate, alarm_index):
        if current_altitude is None:
            return

        prev_altitude = previous_data.get(f"{payload_type}_altitude")
        prev_time = previous_data.get(f"{payload_type}_time")

        if prev_altitude is not None and prev_time is not None:
            time_diff = current_time - prev_time
            if time_diff > 0:
                landing_rate = (current_altitude - prev_altitude) / time_diff
                previous_data[f"{payload_type}_landing_rate"] = landing_rate
                if not (min_rate <= landing_rate <= max_rate):
                    alarm_codes[alarm_index] = 1

        # Update previous altitude and time for the next iteration
        previous_data[f"{payload_type}_altitude"] = current_altitude
        previous_data[f"{payload_type}_time"] = current_time

    update_landing_rate('container', container_altitude, -14, -12, CONTAINER_LANDING_ALARM)
    update_landing_rate('science_payload', science_payload_altitude, -8, -6, SCIENCE_PAYLOAD_LANDING_ALARM)

    if not isinstance(container_pressure, (float, int)) or int(container_pressure) == 0:
        alarm_codes[PRESSURE_SENSOR_ALARM] = 1

    if any(gps_data.get(k) == "0.0000000" or gps_data.get(k) is None for k in
           ["Latitude", "Longitude"]) or gps_data.get("Altitude") in ["0.00", None]:
        alarm_codes[GPS_DATA_ALARM] = 1

    # Check altitude difference only once
    if not previous_data["altitude_difference_checked"]:
        if state < 3 or (altitude_difference is None or int(altitude_difference) < 25):
            alarm_codes[ALTITUDE_DIFFERENCE_ALARM] = 1
        else:
            previous_data["altitude_difference_checked"] = True

    return alarm_codes, previous_data.get('science_payload_landing_rate')


class SerialManager:
    def __init__(self, port='/dev/serial0', baudrate=115200, timeout=1, max_reconnect_attempts=5):
        self.serial_port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection = None

    def connect(self):
        if self.connection is None or not self.connection.is_open:
            try:
                self.connection = serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=self.timeout)
                print(f"Connected to {self.serial_port} at {self.baudrate} baud.")
            except serial.SerialException as e:
                print(f"Failed to connect to {self.serial_port}: {e}")
                self.connection = None
        return self.connection

    def disconnect(self):
        if self.connection and self.connection.is_open:
            self.connection.close()
            print(f"Disconnected from {self.serial_port}")
            self.connection = None

    def read_data(self):
        if self.connection and self.connection.is_open:
            try:
                data = self.connection.readline().decode('utf-8', errors='ignore').strip()
                if not data:
                    print("Warning: No data received from serial.")
                return data
            except serial.SerialException as e:
                print(f"Error reading from {self.serial_port}: {e}")
                return None
        else:
            raise ConnectionError("Serial connection is not established.")

    def write_data(self, data):
        if self.connection and self.connection.is_open:
            try:
                print(f"Writing data: {data}")
                self.connection.write(data.encode('utf-8'))
            except serial.SerialException as e:
                print(f"Error writing to {self.serial_port}: {e}")
        else:
            raise ConnectionError("Serial connection is not established.")


class CommandReceiver(Sensor):
    def __init__(self, sensors: list, buzzer_pin: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sensors = sensors
        self.buzzer_pin = buzzer_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        self.buzzer_off()

        self.folder_path = Path("/home/dyaus/335592_BlackBox_RPI")
        self.folder_path.mkdir(parents=True, exist_ok=True)
        (self.folder_path / "Packet Count Files").mkdir(parents=True, exist_ok=True)

        self.buzzer_blink(1)

        # Automatically start sensors if the file exists
        if os.path.isfile("/home/dyaus/packet_count.txt"):
            self.start_sensors()
        else:
            self.log("packet_count.txt not found. Sensors will wait for a start command.")

    def backup_file(self, filename, backup_folder_name, time_stamp):
        file_path = self.folder_path / filename
        if not file_path.exists():
            self.log(f"File {filename} does not exist. Skipping backup.")
            return

        backup_folder = self.folder_path / backup_folder_name
        backup_folder.mkdir(parents=True, exist_ok=True)

        new_file_name = f"{time_stamp}_{filename}"
        try:
            shutil.copy(file_path, backup_folder / new_file_name)
            file_path.unlink()
            self.log(f"Backup created: {backup_folder / new_file_name}")
        except Exception as e:
            self.log(f"Error creating backup for {filename}: {e}")

    async def process_command(self, websocket, path):
        try:
            async for message in websocket:
                await self.handle_command(message.strip())
        except websockets.ConnectionClosed:
            self.log("Connection closed unexpectedly", "error")
        except websockets.InvalidHandshake:
            self.log("Invalid handshake with server", "error")
        except ConnectionRefusedError:
            self.log("Connection refused", "error")

    async def handle_command(self, command):
        if command == "start":
            self.buzzer_blink(4)
            self.start_sensors()
        elif command == "stop":
            self.stop_sensors()
            self.buzzer_on_indefinitely()
        else:
            self.log(f"Unknown command: {command}", "warning")
            self.buzzer_off()

    def start_sensors(self):
        print("Starting Sensors")
        for sensor in self.sensors:
            sensor.start()

    def stop_sensors(self):
        print("Stopping Sensors")
        for sensor in self.sensors:
            sensor.stop()
        # if os.path.isfile("/home/dyaus/packet_count.txt"):
        #     os.remove("/home/dyaus/packet_count.txt")

    def worker(self) -> None:
        async def run():
            host = "0.0.0.0"
            port = 9004
            while not self.stop_event.is_set():
                try:
                    start_server = await websockets.serve(self.process_command, host, port)
                    self.log(f"CommandReceiver WebSocket server started at {host}:{port}")
                    await start_server.wait_closed()
                    self.log("CommandReceiver WebSocket server stopped.")
                except Exception as e:
                    self.log(f"Error in WebSocket server: {e}", "error")
                    await asyncio.sleep(5)

        asyncio.set_event_loop(asyncio.new_event_loop())
        asyncio.run(run())

    def buzzer_blink(self, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            GPIO.output(self.buzzer_pin, GPIO.HIGH)
            time.sleep(0.25)
            GPIO.output(self.buzzer_pin, GPIO.LOW)
            time.sleep(0.25)
        self.log(f"Buzzer blinked for {duration} seconds")

    def buzzer_on_indefinitely(self):
        self.log("Buzzer is ON indefinitely")
        GPIO.output(self.buzzer_pin, GPIO.HIGH)

    def buzzer_off(self):
        GPIO.output(self.buzzer_pin, GPIO.LOW)
        self.log("Buzzer is OFF")


TEMPLATES_DIR = '/home/dyaus/templates'
INDEX_HTML_PATH = os.path.join(TEMPLATES_DIR, 'index.html')
MEDIA_DIR = "/var/www/335592_Flight/media"


def create_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)
        print(f"Directory created: {path}")
    else:
        print(f"Directory already exists: {path}")


def create_file(path, content=''):
    if not os.path.exists(path):
        with open(path, 'w') as file:
            file.write(content)
        print(f"File created: {path}")
    else:
        print(f"File already exists {path}")


create_directory(TEMPLATES_DIR)

index_html_content = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Media Gallery</title>
    <style>
        #media-list a {
            display: block;
            margin: 5px 0;
        }
    </style>
</head>
<body>
    <h1>Media Gallery</h1>
    <div id="media-list"></div>

    <script>
        async function fetchMediaList() {
            const response = await fetch('/media_list');
            const files = await response.json();
            const mediaListDiv = document.getElementById('media-list');

            files.forEach(file => {
                const link = document.createElement('a');
                link.href = file.url;  // Use the URL from JSON
                link.textContent = file.name;
                link.target = '_blank';  // Open in a new tab
                mediaListDiv.appendChild(link);
            });
        }

        window.onload = fetchMediaList;
    </script>
</body>
</html>

"""

create_file(INDEX_HTML_PATH, index_html_content)

app = Flask(__name__)


def generate_log(file_to_gen):
    with open(file_to_gen, 'r') as f:
        while True:
            yield f.read()


@app.route('/telemetry.log')
def stream_telemetry_log():
    return Response(generate_log(LOG_FILE), content_type='text/event-stream')


@app.route('/telemetry.csv')
def stream_telemetry_csv():
    return Response(generate_log("/home/dyaus/335592_BlackBox_RPI/335592_csv_data.csv"),
                    content_type='text/event-stream')


@app.route('/packet_count.txt')
def stream_packet_count():
    return Response(generate_log("/home/dyaus/packet_count.txt"), content_type='text/event-stream')


@app.route('/media_list')
def list_media():
    files = os.listdir(MEDIA_DIR)
    file_list = [{"name": file, "url": f"/media/{file}"} for file in files]
    return jsonify(file_list)


@app.route('/media/<filename>')
def serve_media(filename):
    return send_from_directory(MEDIA_DIR, filename)


@app.route('/')
def index():
    return render_template('index.html')


def run_flask_server():
    app.run(debug=True)


serial_manager = SerialManager()
serial_manager.connect()

SENSOR_NAMES = ["Pressure1", "Altitude1", "Temperature", "RP2040_Data", "BNO", "IOT_Data", "Mech_Filter"]
PRESSURE_QUEUE = queue.Queue()
ALTITUDE_QUEUE = queue.Queue()
TEMPERATURE_QUEUE = queue.Queue()
RP2040_QUEUE = queue.Queue()
BNO_QUEUE = queue.Queue()
IOT_DATA_QUEUE = queue.Queue()
MECH_FILTER_QUEUE = queue.Queue()
useTest = False
delay = 0
pressure1_sensor = BMP390Sensor("Pressure1 and Altitude2", useTest, delay)
temperature_sensor = TMP117Sensor("Temperature", useTest, delay)
rp2040_data = RP2040("RP2040", serial_manager, useTest, delay)
bno_sensor = BNOSensor("BNO", useTest, delay)
iot_data_receiver = IOTDataReceiver("IOT_Data", serial_manager, useTest, delay)
mech_filter_server = MechFilterServer("Mech_Filter", useTest, delay)
data_transmitter = DataTransmitter("Data_Transmitter", useTest, delay, PRESSURE_QUEUE, ALTITUDE_QUEUE,
                                   TEMPERATURE_QUEUE,
                                   RP2040_QUEUE, BNO_QUEUE, IOT_DATA_QUEUE, MECH_FILTER_QUEUE)

command_receiver_sensors = [pressure1_sensor, temperature_sensor, rp2040_data, bno_sensor]
normal_thread_sensor = [iot_data_receiver, mech_filter_server, data_transmitter]

command_receiver = CommandReceiver(command_receiver_sensors, 5, "CommandReceiver", useTest, delay)

threads = [
    threading.Thread(target=sensor.start) for sensor in normal_thread_sensor
]

threads.append(threading.Thread(target=command_receiver.worker))
flask_thread = threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})

threads.append(flask_thread)

for thread in threads:
    thread.start()

for thread in threads:
    thread.join()
