import time
import sys
import argparse
import signal
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200AngularRateInterface:

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
    
    def configure_angular_rate_output(self, update_rate_hz=50):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeStartup = 1      
            binary_output.common.angularRate = 1      
            binary_output.common.imu = 1             
            
            self.sensor.writeRegister(binary_output)
            print(f"Angular rate output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring angular rate output: {e}")
            return False
    
    def get_angular_rate_register(self):

        try:
            angular_rate_register = Registers.AngularRate()
            self.sensor.readRegister(angular_rate_register)
            
            x = angular_rate_register.omega.x
            y = angular_rate_register.omega.y
            z = angular_rate_register.omega.z
            
            magnitude = np.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,  
                "y": y,  
                "z": z,  
                "magnitude": magnitude,
                "method": "angular_rate_register"
            }
        except VnError as e:
            print(f"Error reading angular rate register: {e}")
            return None
    
    def get_raw_gyro_data(self):
        
        try:
            imu_register = Registers.ImuMeasurements()
            self.sensor.readRegister(imu_register)
            
            gyro_x = imu_register.gyro.x
            gyro_y = imu_register.gyro.y
            gyro_z = imu_register.gyro.z
            
            return {
                "gyro_x": gyro_x,  
                "gyro_y": gyro_y,  
                "gyro_z": gyro_z,  
                "method": "raw_imu_measurements"
            }
        except VnError as e:
            print(f"Error reading raw gyro data: {e}")
            return None
    
    def get_angular_rate_async(self, timeout=1.0):
        
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  # Small sleep to avoid CPU hogging
                    continue
                
                if cd.imu.gyro:
                    x = cd.imu.gyro.x
                    y = cd.imu.gyro.y
                    z = cd.imu.gyro.z
                    
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                    
                    return {
                        "x": x,
                        "y": y,
                        "z": z,
                        "magnitude": magnitude,
                        "timestamp": cd.time.timeStartup.nanoseconds() / 1e9 if cd.time.timeStartup else 0,
                        "method": "async_measurement"
                    }
            
            print("Timeout waiting for angular rate data")
            return None
        except VnError as e:
            print(f"Error getting async angular rate: {e}")
            return None
    
    def get_angular_rate_body_and_inertial(self):
        
        try:
            body_rates = self.get_angular_rate_register()
            if not body_rates:
                return None
                
            quat_register = Registers.QuaternionMag()
            self.sensor.readRegister(quat_register)
            
            qx = quat_register.quat.x
            qy = quat_register.quat.y
            qz = quat_register.quat.z
            qw = quat_register.quat.w
            
            body_rate_vector = np.array([body_rates["x"], body_rates["y"], body_rates["z"]])
            
            q_matrix = np.array([
                [1-2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1-2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx**2 + qy**2)]
            ])
            
            inertial_rate_vector = np.dot(q_matrix, body_rate_vector)
            
            return {
                "body_rates": {
                    "x": body_rates["x"],
                    "y": body_rates["y"],
                    "z": body_rates["z"]
                },
                "inertial_rates": {
                    "x": float(inertial_rate_vector[0]),
                    "y": float(inertial_rate_vector[1]),
                    "z": float(inertial_rate_vector[2])
                },
                "method": "body_and_inertial_frames"
            }
        except VnError as e:
            print(f"Error computing angular rates in different frames: {e}")
            return None
        except Exception as e:
            print(f"Error in frame conversion: {e}")
            return None
    
    def get_gyro_bias(self):
        
        try:
            gyro_bias_register = Registers.GyroBias()
            self.sensor.readRegister(gyro_bias_register)
            
            x_bias = gyro_bias_register.b.x
            y_bias = gyro_bias_register.b.y
            z_bias = gyro_bias_register.b.z
            
            return {
                "x_bias": x_bias,
                "y_bias": y_bias,
                "z_bias": z_bias,
                "method": "gyro_bias"
            }
        except VnError as e:
            print(f"Error reading gyro bias: {e}")
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
    
    parser = argparse.ArgumentParser(description='VN-200 Angular Rate Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=50, help='Update rate in Hz')
    args = parser.parse_args()
    
    angular_rate_interface = VN200AngularRateInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        angular_rate_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not angular_rate_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not angular_rate_interface.configure_angular_rate_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        angular_rate_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 Angular Rate Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            angular_rates = angular_rate_interface.get_angular_rate_register()
            if angular_rates:
                print(f"\nMethod 1 - Compensated Angular Rates:")
                print(f"  X-axis: {angular_rates['x']:.2f} °/s")
                print(f"  Y-axis: {angular_rates['y']:.2f} °/s")
                print(f"  Z-axis: {angular_rates['z']:.2f} °/s")
                print(f"  Magnitude: {angular_rates['magnitude']:.2f} °/s")
            
            raw_gyro = angular_rate_interface.get_raw_gyro_data()
            if raw_gyro:
                print(f"\nMethod 2 - Raw Gyroscope Data:")
                print(f"  X-axis: {raw_gyro['gyro_x']:.2f} °/s")
                print(f"  Y-axis: {raw_gyro['gyro_y']:.2f} °/s")
                print(f"  Z-axis: {raw_gyro['gyro_z']:.2f} °/s")
            
            async_data = angular_rate_interface.get_angular_rate_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 3 - Asynchronous Angular Rate Data:")
                print(f"  X-axis: {async_data['x']:.2f} °/s")
                print(f"  Y-axis: {async_data['y']:.2f} °/s")
                print(f"  Z-axis: {async_data['z']:.2f} °/s")
                print(f"  Timestamp: {async_data['timestamp']:.3f} s")
            
            frame_data = angular_rate_interface.get_angular_rate_body_and_inertial()
            if frame_data:
                print(f"\nMethod 4 - Body and Inertial Frame Angular Rates:")
                body = frame_data["body_rates"]
                inertial = frame_data["inertial_rates"]
                print(f"  Body Frame:    X={body['x']:.2f}, Y={body['y']:.2f}, Z={body['z']:.2f} °/s")
                print(f"  Inertial Frame: X={inertial['x']:.2f}, Y={inertial['y']:.2f}, Z={inertial['z']:.2f} °/s")
            
            bias_data = angular_rate_interface.get_gyro_bias()
            if bias_data:
                print(f"\nMethod 5 - Gyroscope Bias Values:")
                print(f"  X-axis Bias: {bias_data['x_bias']:.6f} °/s")
                print(f"  Y-axis Bias: {bias_data['y_bias']:.6f} °/s")
                print(f"  Z-axis Bias: {bias_data['z_bias']:.6f} °/s")
            
            print("\n" + "-" * 40)
            time.sleep(2)  
            
    except KeyboardInterrupt:
        pass
    finally:
        angular_rate_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
