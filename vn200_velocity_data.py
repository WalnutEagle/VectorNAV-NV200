import time
import sys
import argparse
import signal
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200VelocityInterface:
    
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
    
    def configure_velocity_output(self, update_rate_hz=20):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeStartup = 1      # Timestamp
            binary_output.common.velocity = 1         # Velocity (body frame)
            
            binary_output.ins.velNed = 1              # Velocity in NED frame
            binary_output.ins.velEcef = 1             # Velocity in ECEF frame
            binary_output.ins.velU = 1                # Velocity uncertainty
            
            self.sensor.writeRegister(binary_output)
            print(f"Velocity output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring velocity output: {e}")
            return False
    
    def get_velocity_body(self):
        try:
            velocity_register = Registers.Velocity()
            self.sensor.readRegister(velocity_register)
            
            x = velocity_register.vel.x
            y = velocity_register.vel.y
            z = velocity_register.vel.z
            
            speed = np.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,  # X-axis velocity (m/s)
                "y": y,  # Y-axis velocity (m/s)
                "z": z,  # Z-axis velocity (m/s)
                "speed": speed,
                "method": "velocity_body_frame"
            }
        except VnError as e:
            print(f"Error reading body velocity: {e}")
            return None
    
    def get_velocity_ned(self):
        try:
            vel_ned_register = Registers.VelNed()
            self.sensor.readRegister(vel_ned_register)
            
            north = vel_ned_register.velNed.x
            east = vel_ned_register.velNed.y
            down = vel_ned_register.velNed.z
            
            horizontal_speed = np.sqrt(north**2 + east**2)
            
            speed_3d = np.sqrt(north**2 + east**2 + down**2)
            
            course = np.degrees(np.arctan2(east, north)) % 360
            
            return {
                "north": north,  # Velocity in North direction (m/s)
                "east": east,    # Velocity in East direction (m/s)
                "down": down,    # Velocity in Down direction (m/s)
                "horizontal_speed": horizontal_speed,
                "speed_3d": speed_3d,
                "course": course,
                "method": "velocity_ned_frame"
            }
        except VnError as e:
            print(f"Error reading NED velocity: {e}")
            return None
    
    def get_velocity_ecef(self):
        try:
            vel_ecef_register = Registers.VelEcef()
            self.sensor.readRegister(vel_ecef_register)
            x = vel_ecef_register.velEcef.x
            y = vel_ecef_register.velEcef.y
            z = vel_ecef_register.velEcef.z
            
            speed = np.sqrt(x**2 + y**2 + z**2)
            
            return {
                "x": x,  # X-axis ECEF velocity (m/s)
                "y": y,  # Y-axis ECEF velocity (m/s)
                "z": z,  # Z-axis ECEF velocity (m/s)
                "speed": speed,
                "method": "velocity_ecef_frame"
            }
        except VnError as e:
            print(f"Error reading ECEF velocity: {e}")
            return None
    
    def get_velocity_uncertainty(self):

        try:
            vel_u_register = Registers.VelUncertainty()
            self.sensor.readRegister(vel_u_register)
            
            x_uncertainty = vel_u_register.velU.x
            y_uncertainty = vel_u_register.velU.y
            z_uncertainty = vel_u_register.velU.z
            
            return {
                "x_uncertainty": x_uncertainty,
                "y_uncertainty": y_uncertainty,
                "z_uncertainty": z_uncertainty,
                "method": "velocity_uncertainty"
            }
        except VnError as e:
            print(f"Error reading velocity uncertainty: {e}")
            return None
    
    def get_velocity_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  
                    continue
                
                result = {"method": "async_measurement"}
                
                if cd.velocity.vel:
                    result["x"] = cd.velocity.vel.x
                    result["y"] = cd.velocity.vel.y
                    result["z"] = cd.velocity.vel.z
                    result["speed"] = np.sqrt(
                        cd.velocity.vel.x**2 + 
                        cd.velocity.vel.y**2 + 
                        cd.velocity.vel.z**2
                    )
                
                if cd.velocity.velNed:
                    result["north"] = cd.velocity.velNed.x
                    result["east"] = cd.velocity.velNed.y
                    result["down"] = cd.velocity.velNed.z
                    
                    result["horizontal_speed"] = np.sqrt(
                        cd.velocity.velNed.x**2 + 
                        cd.velocity.velNed.y**2
                    )
                    
                    result["speed_3d"] = np.sqrt(
                        cd.velocity.velNed.x**2 + 
                        cd.velocity.velNed.y**2 + 
                        cd.velocity.velNed.z**2
                    )
                    
                    result["course"] = np.degrees(
                        np.arctan2(cd.velocity.velNed.y, cd.velocity.velNed.x)
                    ) % 360
                
                if cd.time.timeStartup:
                    result["timestamp"] = cd.time.timeStartup.nanoseconds() / 1e9
                
                if "x" in result or "north" in result:
                    return result
            
            print("Timeout waiting for velocity data")
            return None
        except VnError as e:
            print(f"Error getting async velocity: {e}")
            return None
    
    def get_estimated_ground_speed(self):

        try:
            ned_vel = self.get_velocity_ned()
            if not ned_vel:
                return None
            
            uncertainty = self.get_velocity_uncertainty()
            
            result = {
                "ground_speed": ned_vel["horizontal_speed"],
                "true_heading": ned_vel["course"],
                "vertical_speed": -ned_vel["down"],
                "method": "estimated_ground_speed"
            }
            
            if uncertainty:
                result["horizontal_uncertainty"] = np.sqrt(
                    uncertainty["x_uncertainty"]**2 + 
                    uncertainty["y_uncertainty"]**2
                )
                result["vertical_uncertainty"] = uncertainty["z_uncertainty"]
            
            return result
        except Exception as e:
            print(f"Error calculating ground speed: {e}")
            return None
    
    def disconnect(self):
        """Disconnect from the VN-200 sensor."""
        if self.connected:
            try:
                self.sensor.disconnect()
                print("Disconnected from VN-200")
            except VnError as e:
                print(f"Error disconnecting: {e}")
            
            self.connected = False


def main():

    parser = argparse.ArgumentParser(description='VN-200 Velocity Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    args = parser.parse_args()
    
    velocity_interface = VN200VelocityInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        velocity_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not velocity_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not velocity_interface.configure_velocity_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        velocity_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 Velocity Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            body_vel = velocity_interface.get_velocity_body()
            if body_vel:
                print(f"\nMethod 1 - Body Frame Velocity:")
                print(f"  X-axis: {body_vel['x']:.2f} m/s")
                print(f"  Y-axis: {body_vel['y']:.2f} m/s")
                print(f"  Z-axis: {body_vel['z']:.2f} m/s")
                print(f"  Speed:  {body_vel['speed']:.2f} m/s")
            
            ned_vel = velocity_interface.get_velocity_ned()
            if ned_vel:
                print(f"\nMethod 2 - NED Frame Velocity:")
                print(f"  North:   {ned_vel['north']:.2f} m/s")
                print(f"  East:    {ned_vel['east']:.2f} m/s")
                print(f"  Down:    {ned_vel['down']:.2f} m/s")
                print(f"  Horizontal Speed: {ned_vel['horizontal_speed']:.2f} m/s")
                print(f"  3D Speed:         {ned_vel['speed_3d']:.2f} m/s")
                print(f"  Course:           {ned_vel['course']:.2f}°")
            
            ecef_vel = velocity_interface.get_velocity_ecef()
            if ecef_vel:
                print(f"\nMethod 3 - ECEF Frame Velocity:")
                print(f"  X-axis: {ecef_vel['x']:.2f} m/s")
                print(f"  Y-axis: {ecef_vel['y']:.2f} m/s")
                print(f"  Z-axis: {ecef_vel['z']:.2f} m/s")
                print(f"  Speed:  {ecef_vel['speed']:.2f} m/s")
            
            uncertainty = velocity_interface.get_velocity_uncertainty()
            if uncertainty:
                print(f"\nMethod 4 - Velocity Uncertainty:")
                print(f"  X Uncertainty: {uncertainty['x_uncertainty']:.3f} m/s")
                print(f"  Y Uncertainty: {uncertainty['y_uncertainty']:.3f} m/s")
                print(f"  Z Uncertainty: {uncertainty['z_uncertainty']:.3f} m/s")
            
            async_data = velocity_interface.get_velocity_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 5 - Asynchronous Velocity Data:")
                if "x" in async_data:
                    print(f"  Body Frame:")
                    print(f"    X-axis: {async_data['x']:.2f} m/s")
                    print(f"    Y-axis: {async_data['y']:.2f} m/s")
                    print(f"    Z-axis: {async_data['z']:.2f} m/s")
                    print(f"    Speed:  {async_data['speed']:.2f} m/s")
                
                if "north" in async_data:
                    print(f"  NED Frame:")
                    print(f"    North:   {async_data['north']:.2f} m/s")
                    print(f"    East:    {async_data['east']:.2f} m/s")
                    print(f"    Down:    {async_data['down']:.2f} m/s")
                    print(f"    Horizontal Speed: {async_data['horizontal_speed']:.2f} m/s")
                    print(f"    Course:           {async_data['course']:.2f}°")
            
            ground_speed = velocity_interface.get_estimated_ground_speed()
            if ground_speed:
                print(f"\nMethod 6 - Estimated Ground Speed:")
                print(f"  Ground Speed:    {ground_speed['ground_speed']:.2f} m/s")
                print(f"  True Heading:    {ground_speed['true_heading']:.2f}°")
                print(f"  Vertical Speed:  {ground_speed['vertical_speed']:.2f} m/s (+ is up)")
                
                if "horizontal_uncertainty" in ground_speed:
                    print(f"  Horizontal Uncertainty: {ground_speed['horizontal_uncertainty']:.3f} m/s")
                    print(f"  Vertical Uncertainty:   {ground_speed['vertical_uncertainty']:.3f} m/s")
            
            print("\n" + "-" * 40)
            time.sleep(2) 
            
    except KeyboardInterrupt:
        pass
    finally:
        velocity_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
