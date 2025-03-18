import time
import sys
import argparse
import signal
import threading
import numpy as np
from datetime import datetime
from typing import Dict, Any, Optional, List, Tuple

from vectornav import Sensor, Registers, VnError

class VN200Interface:
    
    def __init__(self, port: str = "/dev/ttyUSB0", baud_rate: int = 115200):

        self.port = port
        self.baud_rate = baud_rate
        self.sensor = Sensor()
        self.connected = False
        self.data_lock = threading.Lock()
        self.latest_data = {}
        self.running = False
        self.data_thread = None

    def connect(self) -> bool:

        try:
            self.sensor.connect(self.port, self.baud_rate)
            self.connected = True
            print(f"Connected to VN-200 on {self.port} at {self.sensor.connectedBaudRate()} baud")
            
            model_register = Registers.Model()
            self.sensor.readRegister(model_register)
            print(f"Device: {model_register.model}")
            
            return True
        except VnError as e:
            print(f"Error connecting to VN-200: {e}")
            return False

    def configure_outputs(self, update_rate_hz: int = 20) -> bool:

        try:
            # Calculate the appropriate rate divisor
            # The VN-200 internal rate is 800Hz, so I divide to get our desired rate
            rate_divisor = int(800 / update_rate_hz)
            binary_output1 = Registers.BinaryOutput1()
            binary_output1.rateDivisor = rate_divisor
            # Enable async output on UART port 1
            binary_output1.asyncMode.serial1 = 1
            
            # Group 1: Common Data
            binary_output1.common.timeStartup = 1        # Timestamp
            binary_output1.common.timeGps = 1            # GPS time
            binary_output1.common.ypr = 1                # Yaw, Pitch, Roll
            binary_output1.common.quaternion = 1         # Orientation as quaternion
            binary_output1.common.angularRate = 1        # Angular velocity
            binary_output1.common.position = 1           # GPS position
            binary_output1.common.velocity = 1           # Velocity
            binary_output1.common.accel = 1              # Linear acceleration
            binary_output1.common.imu = 1                # Raw IMU measurements
            binary_output1.common.magPres = 1            # Magnetometer and pressure
            
            # Group 2: INS-specific Data
            binary_output1.ins.insStatus = 1             # INS status flags
            binary_output1.ins.posLla = 1                # Latitude, longitude, altitude
            binary_output1.ins.velNed = 1                # North, East, Down velocity
            self.sensor.writeRegister(binary_output1)
            print(f"VN-200 configured to output at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring VN-200 outputs: {e}")
            return False

    def start_data_collection(self) -> None:
        if not self.connected:
            print("Cannot start data collection. Sensor not connected.")
            return
        
        self.running = True
        self.data_thread = threading.Thread(target=self._data_collection_loop, daemon=True)
        self.data_thread.start()
        print("Data collection started")

    def stop_data_collection(self) -> None:

        self.running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        print("Data collection stopped")

    def _data_collection_loop(self) -> None:

        while self.running and self.connected:
            try:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  # Small sleep to avoid CPU hogging
                    continue
                
                data = {}
                
                if cd.time.timeStartup:
                    data['timestamp'] = cd.time.timeStartup.nanoseconds() / 1e9  # Convert to seconds
                
                if cd.attitude.ypr:
                    data['yaw'] = cd.attitude.ypr.x
                    data['pitch'] = cd.attitude.ypr.y
                    data['roll'] = cd.attitude.ypr.z
                
                if cd.attitude.quaternion:
                    data['quaternion'] = [
                        cd.attitude.quaternion.x,
                        cd.attitude.quaternion.y,
                        cd.attitude.quaternion.z,
                        cd.attitude.quaternion.w
                    ]
                
                if cd.imu.accel:
                    data['acceleration'] = [
                        cd.imu.accel.x,
                        cd.imu.accel.y,
                        cd.imu.accel.z
                    ]
                
                if cd.imu.gyro:
                    data['angular_rate'] = [
                        cd.imu.gyro.x,
                        cd.imu.gyro.y,
                        cd.imu.gyro.z
                    ]
                
                if cd.ins.posLla:
                    data['position'] = {
                        'latitude': cd.ins.posLla.x,
                        'longitude': cd.ins.posLla.y,
                        'altitude': cd.ins.posLla.z
                    }
                
                if cd.ins.velNed:
                    data['velocity'] = {
                        'north': cd.ins.velNed.x,
                        'east': cd.ins.velNed.y,
                        'down': cd.ins.velNed.z
                    }
                    
                    data['speed'] = np.sqrt(
                        cd.ins.velNed.x**2 + 
                        cd.ins.velNed.y**2 + 
                        cd.ins.velNed.z**2
                    )
                
                with self.data_lock:
                    self.latest_data = data
                
            except VnError as e:
                print(f"Error reading from sensor: {e}")
                time.sleep(0.1)  # Sleep as I was getting spamming errors.
            except Exception as e:
                print(f"Unexpected error: {e}")
                time.sleep(0.1)

    def get_latest_data(self) -> Dict[str, Any]:

        with self.data_lock:
            return self.latest_data.copy()

    def disconnect(self) -> None:
        if self.connected:
            self.stop_data_collection()

            try:
                self.sensor.disconnect()
                print("Disconnected from VN-200")
            except VnError as e:
                print(f"Error disconnecting: {e}")
            
            self.connected = False


def main():
    parser = argparse.ArgumentParser(description='VN-200 Interface for Jetson Orin')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    parser.add_argument('--duration', type=int, default=30, help='Run duration in seconds')
    args = parser.parse_args()
    
    vn200 = VN200Interface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        vn200.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not vn200.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not vn200.configure_outputs(update_rate_hz=args.rate):
        print("Failed to configure VN-200. Exiting.")
        vn200.disconnect()
        sys.exit(1)
    
    vn200.start_data_collection()
    
    start_time = time.time()
    try:
        while time.time() - start_time < args.duration:
            data = vn200.get_latest_data()
            if data:
                print("\033c", end="") 
                print(f"VN-200 Data - {datetime.now().strftime('%H:%M:%S')}")
                print("-" * 50)
                
                if 'yaw' in data:
                    print(f"Orientation (Yaw,Pitch,Roll): {data['yaw']:.2f}°, {data['pitch']:.2f}°, {data['roll']:.2f}°")
                
                if 'position' in data:
                    pos = data['position']
                    print(f"Position (Lat,Lon,Alt): {pos['latitude']:.6f}°, {pos['longitude']:.6f}°, {pos['altitude']:.2f}m")
                
                if 'velocity' in data:
                    vel = data['velocity']
                    print(f"Velocity (N,E,D): {vel['north']:.2f}, {vel['east']:.2f}, {vel['down']:.2f} m/s")
                    print(f"Speed: {data.get('speed', 0):.2f} m/s")
                
                if 'acceleration' in data:
                    accel = data['acceleration']
                    print(f"Acceleration (X,Y,Z): {accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f} m/s²")
                
                if 'angular_rate' in data:
                    gyro = data['angular_rate']
                    print(f"Angular Rate (X,Y,Z): {gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f} °/s")
                
                print("\nPress Ctrl+C to exit")
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        pass
    finally:
        vn200.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
