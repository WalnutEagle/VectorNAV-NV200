import time
import sys
import argparse
import signal
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200AccelerationInterface:
    
    def __init__(self, port="/dev/ttyUSB0", baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.sensor = Sensor()
        self.connected = False
    
    def connect(self):
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
    
    def configure_acceleration_output(self, update_rate_hz=50):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeStartup = 1
            binary_output.common.accel = 1
            binary_output.common.imu = 1
            binary_output.common.accelNed = 1
            
            self.sensor.writeRegister(binary_output)
            print(f"Acceleration output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring acceleration output: {e}")
            return False
    
    def get_acceleration_register(self):
        try:
            accel_register = Registers.Acceleration()
            self.sensor.readRegister(accel_register)
            
            x = accel_register.accel.x
            y = accel_register.accel.y
            z = accel_register.accel.z
            
            magnitude = np.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,
                "y": y,
                "z": z,
                "magnitude": magnitude,
                "method": "acceleration_register"
            }
        except VnError as e:
            print(f"Error reading acceleration register: {e}")
            return None
    
    def get_raw_accel_data(self):
        try:
            imu_register = Registers.ImuMeasurements()
            self.sensor.readRegister(imu_register)
            
            accel_x = imu_register.accel.x
            accel_y = imu_register.accel.y
            accel_z = imu_register.accel.z
            
            return {
                "accel_x": accel_x,
                "accel_y": accel_y,
                "accel_z": accel_z,
                "method": "raw_imu_measurements"
            }
        except VnError as e:
            print(f"Error reading raw accelerometer data: {e}")
            return None
    
    def get_acceleration_ned(self):
        try:
            accel_ned_register = Registers.AccelNed()
            self.sensor.readRegister(accel_ned_register)
            
            north = accel_ned_register.accelNed.x
            east = accel_ned_register.accelNed.y
            down = accel_ned_register.accelNed.z
            
            return {
                "north": north,
                "east": east,
                "down": down,
                "method": "accel_ned_register"
            }
        except VnError as e:
            print(f"Error reading NED acceleration: {e}")
            return None
    
    def get_acceleration_linear(self):
        try:
            accel = self.get_acceleration_register()
            if not accel:
                return None
            
            quat_register = Registers.QuaternionMag()
            self.sensor.readRegister(quat_register)
            
            qx = quat_register.quat.x
            qy = quat_register.quat.y
            qz = quat_register.quat.z
            qw = quat_register.quat.w
            
            g = 9.81
            
            gx = 2 * (qx * qz - qw * qy) * g
            gy = 2 * (qy * qz + qw * qx) * g
            gz = (1 - 2 * (qx**2 + qy**2)) * g
            
            linear_x = accel["x"] - gx
            linear_y = accel["y"] - gy
            linear_z = accel["z"] - gz
            
            magnitude = np.sqrt(linear_x**2 + linear_y**2 + linear_z**2)
            
            return {
                "x": linear_x,
                "y": linear_y,
                "z": linear_z,
                "magnitude": magnitude,
                "method": "linear_acceleration"
            }
        except VnError as e:
            print(f"Error computing linear acceleration: {e}")
            return None
        except Exception as e:
            print(f"Error in linear acceleration calculation: {e}")
            return None
    
    def get_acceleration_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)
                    continue
                
                if cd.imu.accel:
                    x = cd.imu.accel.x
                    y = cd.imu.accel.y
                    z = cd.imu.accel.z
                    
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                    
                    result = {
                        "x": x,
                        "y": y,
                        "z": z,
                        "magnitude": magnitude,
                        "timestamp": cd.time.timeStartup.nanoseconds() / 1e9 if cd.time.timeStartup else 0,
                        "method": "async_measurement"
                    }
                    
                    if cd.acceleration.accelNed:
                        result["north"] = cd.acceleration.accelNed.x
                        result["east"] = cd.acceleration.accelNed.y
                        result["down"] = cd.acceleration.accelNed.z
                    
                    return result
            
            print("Timeout waiting for acceleration data")
            return None
        except VnError as e:
            print(f"Error getting async acceleration: {e}")
            return None
    
    def disconnect(self):
        if self.connected:
            try:
                self.sensor.disconnect()
                print("Disconnected from VN-200")
            except VnError as e:
                print(f"Error disconnecting: {e}")
            
            self.connected = False


def main():
    parser = argparse.ArgumentParser(description='VN-200 Acceleration Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=50, help='Update rate in Hz')
    args = parser.parse_args()
    
    accel_interface = VN200AccelerationInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        accel_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not accel_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not accel_interface.configure_acceleration_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        accel_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 Acceleration Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            accel_data = accel_interface.get_acceleration_register()
            if accel_data:
                print(f"\nMethod 1 - Compensated Acceleration:")
                print(f"  X-axis: {accel_data['x']:.3f} m/s²")
                print(f"  Y-axis: {accel_data['y']:.3f} m/s²")
                print(f"  Z-axis: {accel_data['z']:.3f} m/s²")
                print(f"  Magnitude: {accel_data['magnitude']:.3f} m/s²")
            
            raw_accel = accel_interface.get_raw_accel_data()
            if raw_accel:
                print(f"\nMethod 2 - Raw Accelerometer Data:")
                print(f"  X-axis: {raw_accel['accel_x']:.3f} m/s²")
                print(f"  Y-axis: {raw_accel['accel_y']:.3f} m/s²")
                print(f"  Z-axis: {raw_accel['accel_z']:.3f} m/s²")
            
            ned_accel = accel_interface.get_acceleration_ned()
            if ned_accel:
                print(f"\nMethod 3 - NED Frame Acceleration:")
                print(f"  North: {ned_accel['north']:.3f} m/s²")
                print(f"  East:  {ned_accel['east']:.3f} m/s²")
                print(f"  Down:  {ned_accel['down']:.3f} m/s²")
            
            linear_accel = accel_interface.get_acceleration_linear()
            if linear_accel:
                print(f"\nMethod 4 - Linear Acceleration (gravity removed):")
                print(f"  X-axis: {linear_accel['x']:.3f} m/s²")
                print(f"  Y-axis: {linear_accel['y']:.3f} m/s²")
                print(f"  Z-axis: {linear_accel['z']:.3f} m/s²")
                print(f"  Magnitude: {linear_accel['magnitude']:.3f} m/s²")
            
            async_data = accel_interface.get_acceleration_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 5 - Asynchronous Acceleration Data:")
                print(f"  X-axis: {async_data['x']:.3f} m/s²")
                print(f"  Y-axis: {async_data['y']:.3f} m/s²")
                print(f"  Z-axis: {async_data['z']:.3f} m/s²")
                
                if "north" in async_data:
                    print(f"  North: {async_data['north']:.3f} m/s²")
                    print(f"  East:  {async_data['east']:.3f} m/s²")
                    print(f"  Down:  {async_data['down']:.3f} m/s²")
                
                print(f"  Timestamp: {async_data['timestamp']:.3f} s")
            
            print("\n" + "-" * 40)
            time.sleep(2)
            
    except KeyboardInterrupt:
        pass
    finally:
        accel_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
