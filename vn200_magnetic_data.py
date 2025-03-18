import time
import sys
import argparse
import signal
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200MagneticInterface:
    
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
    
    def configure_magnetic_output(self, update_rate_hz=20):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeStartup = 1      # Timestamp
            binary_output.common.mag = 1              # Compensated magnetic
            binary_output.common.imu = 1              # Raw IMU measurements
            binary_output.common.quaternion = 1       # For magnetic reference conversions
            
            self.sensor.writeRegister(binary_output)
            print(f"Magnetic output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring magnetic output: {e}")
            return False
    
    def get_magnetic_register(self):
        try:
            mag_register = Registers.MagneticMeasurements()
            self.sensor.readRegister(mag_register)
            
            x = mag_register.mag.x
            y = mag_register.mag.y
            z = mag_register.mag.z
            
            magnitude = np.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,  # X-axis magnetic field (Gauss)
                "y": y,  # Y-axis magnetic field (Gauss)
                "z": z,  # Z-axis magnetic field (Gauss)
                "magnitude": magnitude,
                "method": "magnetic_register"
            }
        except VnError as e:
            print(f"Error reading magnetic register: {e}")
            return None
    
    def get_raw_magnetic_data(self):
        try:
            imu_register = Registers.ImuMeasurements()
            self.sensor.readRegister(imu_register)
            
            mag_x = imu_register.mag.x
            mag_y = imu_register.mag.y
            mag_z = imu_register.mag.z
            
            return {
                "mag_x": mag_x,  # Raw X-axis magnetic field (Gauss)
                "mag_y": mag_y,  # Raw Y-axis magnetic field (Gauss)
                "mag_z": mag_z,  # Raw Z-axis magnetic field (Gauss)
                "magnitude": np.sqrt(mag_x**2 + mag_y**2 + mag_z**2),
                "method": "raw_imu_measurements"
            }
        except VnError as e:
            print(f"Error reading raw magnetometer data: {e}")
            return None
    
    def get_magnetic_ned(self):
        try:
            mag_data = self.get_magnetic_register()
            if not mag_data:
                return None
            
            quat_register = Registers.QuaternionMag()
            self.sensor.readRegister(quat_register)
            
            qx = quat_register.quat.x
            qy = quat_register.quat.y
            qz = quat_register.quat.z
            qw = quat_register.quat.w
            
            mx = mag_data["x"]
            my = mag_data["y"]
            mz = mag_data["z"]
            
            # Quaternion rotation to transform body frame to NED frame
            t0 = 2.0 * (qy * mz - qz * my)
            t1 = 1.0 - 2.0 * (qx * qx + qy * qy)
            mx_ned = mx * t1 + t0 * qx
            
            t0 = 2.0 * (qx * mz - qz * mx)
            t1 = 1.0 - 2.0 * (qy * qy + qz * qz)
            my_ned = my * t1 + t0 * qy
            
            t0 = 2.0 * (qx * my - qy * mx)
            t1 = 1.0 - 2.0 * (qz * qz + qx * qx)
            mz_ned = mz * t1 + t0 * qz
            
            return {
                "north": mx_ned,  # Magnetic field in North direction (Gauss)
                "east": my_ned,   # Magnetic field in East direction (Gauss)
                "down": mz_ned,   # Magnetic field in Down direction (Gauss)
                "magnitude": np.sqrt(mx_ned**2 + my_ned**2 + mz_ned**2),
                "method": "magnetic_ned_frame"
            }
        except VnError as e:
            print(f"Error computing magnetic NED: {e}")
            return None
        except Exception as e:
            print(f"Error in magnetic NED calculation: {e}")
            return None
    
    def get_magnetic_heading(self):
        try:
            mag_ned = self.get_magnetic_ned()
            if mag_ned:
                heading = np.degrees(np.arctan2(mag_ned["east"], mag_ned["north"])) % 360
                
                return {
                    "magnetic_heading": heading,
                    "north": mag_ned["north"],
                    "east": mag_ned["east"],
                    "method": "magnetic_heading_ned"
                }
            
            mag_data = self.get_magnetic_register()
            if not mag_data:
                return None
            
            heading = np.degrees(np.arctan2(mag_data["y"], mag_data["x"])) % 360
            
            return {
                "magnetic_heading_approx": heading,
                "x": mag_data["x"],
                "y": mag_data["y"],
                "note": "Approximate heading, only valid when sensor is level",
                "method": "magnetic_heading_body"
            }
        except Exception as e:
            print(f"Error calculating magnetic heading: {e}")
            return None
    
    def get_quaternion_with_mag(self):
        try:
            quat_mag_register = Registers.QuaternionMag()
            self.sensor.readRegister(quat_mag_register)
            
            qx = quat_mag_register.quat.x
            qy = quat_mag_register.quat.y
            qz = quat_mag_register.quat.z
            qw = quat_mag_register.quat.w
            
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
            
            sinp = 2.0 * (qw * qy - qz * qx)
            if abs(sinp) >= 1:
                pitch = np.sign(sinp) * 90.0  # Use 90 degrees if out of range
            else:
                pitch = np.degrees(np.arcsin(sinp))
            
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp)) % 360
            
            return {
                "qw": qw,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "roll": roll,       # X-axis rotation in degrees
                "pitch": pitch,     # Y-axis rotation in degrees
                "yaw": yaw,         # Z-axis rotation in degrees (magnetic heading)
                "method": "quaternion_mag_register"
            }
        except VnError as e:
            print(f"Error reading quaternion with magnetic reference: {e}")
            return None
    
    def get_magnetic_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001) 
                    continue
                
                if cd.imu.mag:
                    x = cd.imu.mag.x
                    y = cd.imu.mag.y
                    z = cd.imu.mag.z
                    
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                    
                    result = {
                        "x": x,
                        "y": y,
                        "z": z,
                        "magnitude": magnitude,
                        "timestamp": cd.time.timeStartup.nanoseconds() / 1e9 if cd.time.timeStartup else 0,
                        "method": "async_measurement"
                    }
                    
                    if cd.attitude.quaternion:
                        qx = cd.attitude.quaternion.x
                        qy = cd.attitude.quaternion.y
                        qz = cd.attitude.quaternion.z
                        qw = cd.attitude.quaternion.w
                        
                        siny_cosp = 2.0 * (qw * qz + qx * qy)
                        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                        yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp)) % 360
                        
                        result["magnetic_heading"] = yaw
                    
                    return result
            
            print("Timeout waiting for magnetic data")
            return None
        except VnError as e:
            print(f"Error getting async magnetic data: {e}")
            return None
    
    def calibrate_hard_iron(self, samples=100, delay=0.1):

        try:
            print(f"Starting hard iron calibration with {samples} samples...")
            print("Rotate the sensor through all orientations during sampling.")
            
            x_samples = []
            y_samples = []
            z_samples = []
            
            for i in range(samples):
                print(f"Sample {i+1}/{samples}", end="\r")
                mag_data = self.get_magnetic_register()
                if mag_data:
                    x_samples.append(mag_data["x"])
                    y_samples.append(mag_data["y"])
                    z_samples.append(mag_data["z"])
                time.sleep(delay)
            
            print("\nCalculating hard iron offsets...")
            
            x_offset = sum(x_samples) / len(x_samples)
            y_offset = sum(y_samples) / len(y_samples)
            z_offset = sum(z_samples) / len(z_samples)
            
            return {
                "x_offset": x_offset,
                "y_offset": y_offset,
                "z_offset": z_offset,
                "method": "hard_iron_calibration"
            }
        except Exception as e:
            print(f"Error during hard iron calibration: {e}")
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

    parser = argparse.ArgumentParser(description='VN-200 Magnetic Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    parser.add_argument('--calibrate', action='store_true', help='Perform a simple hard iron calibration')
    args = parser.parse_args()
    
    mag_interface = VN200MagneticInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        mag_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not mag_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not mag_interface.configure_magnetic_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        mag_interface.disconnect()
        sys.exit(1)
    
    if args.calibrate:
        calibration = mag_interface.calibrate_hard_iron(samples=100)
        if calibration:
            print("\n=== Hard Iron Calibration Results ===")
            print(f"X Offset: {calibration['x_offset']:.6f} Gauss")
            print(f"Y Offset: {calibration['y_offset']:.6f} Gauss")
            print(f"Z Offset: {calibration['z_offset']:.6f} Gauss")
            print("Note: These offsets should be applied to raw magnetometer readings.")
            
            response = input("\nContinue with magnetic data demo? (y/n): ").strip().lower()
            if response != 'y':
                mag_interface.disconnect()
                print("Program terminated")
                sys.exit(0)
    
    try:
        print("\n=== VN-200 Magnetic Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            mag_data = mag_interface.get_magnetic_register()
            if mag_data:
                print(f"\nMethod 1 - Compensated Magnetic Field:")
                print(f"  X-axis: {mag_data['x']:.6f} Gauss")
                print(f"  Y-axis: {mag_data['y']:.6f} Gauss")
                print(f"  Z-axis: {mag_data['z']:.6f} Gauss")
                print(f"  Magnitude: {mag_data['magnitude']:.6f} Gauss")
            
            raw_mag = mag_interface.get_raw_magnetic_data()
            if raw_mag:
                print(f"\nMethod 2 - Raw Magnetometer Data:")
                print(f"  X-axis: {raw_mag['mag_x']:.6f} Gauss")
                print(f"  Y-axis: {raw_mag['mag_y']:.6f} Gauss")
                print(f"  Z-axis: {raw_mag['mag_z']:.6f} Gauss")
                print(f"  Magnitude: {raw_mag['magnitude']:.6f} Gauss")
            
            ned_mag = mag_interface.get_magnetic_ned()
            if ned_mag:
                print(f"\nMethod 3 - NED Frame Magnetic Field:")
                print(f"  North: {ned_mag['north']:.6f} Gauss")
                print(f"  East:  {ned_mag['east']:.6f} Gauss")
                print(f"  Down:  {ned_mag['down']:.6f} Gauss")
                print(f"  Magnitude: {ned_mag['magnitude']:.6f} Gauss")
            
            heading = mag_interface.get_magnetic_heading()
            if heading:
                print(f"\nMethod 4 - Magnetic Heading:")
                if "magnetic_heading" in heading:
                    print(f"  Magnetic Heading: {heading['magnetic_heading']:.2f}°")
                    print(f"  North Component: {heading['north']:.6f} Gauss")
                    print(f"  East Component:  {heading['east']:.6f} Gauss")
                else:
                    print(f"  Approx. Heading: {heading['magnetic_heading_approx']:.2f}°")
                    print(f"  Note: {heading['note']}")
            
            quat_mag = mag_interface.get_quaternion_with_mag()
            if quat_mag:
                print(f"\nMethod 5 - Quaternion with Magnetic Reference:")
                print(f"  Quaternion: [{quat_mag['qw']:.4f}, {quat_mag['qx']:.4f}, {quat_mag['qy']:.4f}, {quat_mag['qz']:.4f}]")
                print(f"  Roll:  {quat_mag['roll']:.2f}°")
                print(f"  Pitch: {quat_mag['pitch']:.2f}°")
                print(f"  Yaw:   {quat_mag['yaw']:.2f}° (Magnetic North)")
            
            async_data = mag_interface.get_magnetic_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 6 - Asynchronous Magnetic Data:")
                print(f"  X-axis: {async_data['x']:.6f} Gauss")
                print(f"  Y-axis: {async_data['y']:.6f} Gauss")
                print(f"  Z-axis: {async_data['z']:.6f} Gauss")
                
                if "magnetic_heading" in async_data:
                    print(f"  Magnetic Heading: {async_data['magnetic_heading']:.2f}°")
                
                print(f"  Timestamp: {async_data['timestamp']:.3f} s")
            
            print("\n" + "-" * 40)
            time.sleep(2)  
            
    except KeyboardInterrupt:
        pass
    finally:
        mag_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
