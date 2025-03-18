import time
import sys
import argparse
import signal
from vectornav import Sensor, Registers, VnError

class VN200PositionInterface:
    
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
    
    def configure_position_output(self, update_rate_hz=10):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.position = 1         # LLA position (lat, long, alt)
            binary_output.common.timeGps = 1          # GPS time
            
            binary_output.ins.posLla = 1              # Latitude, longitude, altitude
            binary_output.ins.posEcef = 1             # ECEF position
            binary_output.ins.posU = 1                # Position uncertainty
            binary_output.ins.gpsLla = 1              # Direct GPS LLA measurements
            
            self.sensor.writeRegister(binary_output)
            print(f"Position output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring position output: {e}")
            return False
    
    def get_position_register(self):
        try:
            gps_lla_register = Registers.GpsLla()
            self.sensor.readRegister(gps_lla_register)
            
            latitude = gps_lla_register.lla.x
            longitude = gps_lla_register.lla.y
            altitude = gps_lla_register.lla.z
            
            return {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "method": "register_read"
            }
        except VnError as e:
            print(f"Error reading position register: {e}")
            return None
    
    def get_ins_position(self):
        try:
            ins_lla_register = Registers.InsLla()
            self.sensor.readRegister(ins_lla_register)
            
            latitude = ins_lla_register.lla.x
            longitude = ins_lla_register.lla.y
            altitude = ins_lla_register.lla.z
            
            return {
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "method": "ins_solution"
            }
        except VnError as e:
            print(f"Error reading INS position: {e}")
            return None
    
    def get_position_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  
                    continue
                
                if cd.ins.posLla:
                    return {
                        "latitude": cd.ins.posLla.x,
                        "longitude": cd.ins.posLla.y,
                        "altitude": cd.ins.posLla.z,
                        "method": "async_measurement"
                    }
            
            print("Timeout waiting for position data")
            return None
        except VnError as e:
            print(f"Error getting async position: {e}")
            return None
    
    def get_position_uncertainty(self):
        try:
            ins_status_register = Registers.InsStatus()
            self.sensor.readRegister(ins_status_register)
            
            uncertainty_register = Registers.NedPosU()
            self.sensor.readRegister(uncertainty_register)
            
            return {
                "north_uncertainty": uncertainty_register.posU.x,
                "east_uncertainty": uncertainty_register.posU.y,
                "down_uncertainty": uncertainty_register.posU.z,
                "gps_fix": ins_status_register.mode,
                "method": "position_uncertainty"
            }
        except VnError as e:
            print(f"Error reading position uncertainty: {e}")
            return None
    
    def get_position_ecef(self):
        try:
            ecef_register = Registers.InsEcef()
            self.sensor.readRegister(ecef_register)
            
            return {
                "ecef_x": ecef_register.ecef.x,  # X coordinate (meters)
                "ecef_y": ecef_register.ecef.y,  # Y coordinate (meters)
                "ecef_z": ecef_register.ecef.z,  # Z coordinate (meters)
                "method": "ecef_coordinates"
            }
        except VnError as e:
            print(f"Error reading ECEF position: {e}")
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
    parser = argparse.ArgumentParser(description='VN-200 Position Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=10, help='Update rate in Hz')
    args = parser.parse_args()
    
    position_interface = VN200PositionInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        position_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not position_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not position_interface.configure_position_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        position_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 Position Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            pos_data = position_interface.get_position_register()
            if pos_data:
                print(f"\nMethod 1 - Direct Register Read:")
                print(f"  Latitude:  {pos_data['latitude']:.7f}°")
                print(f"  Longitude: {pos_data['longitude']:.7f}°")
                print(f"  Altitude:  {pos_data['altitude']:.2f} m")
            
            ins_pos = position_interface.get_ins_position()
            if ins_pos:
                print(f"\nMethod 2 - INS Solution:")
                print(f"  Latitude:  {ins_pos['latitude']:.7f}°")
                print(f"  Longitude: {ins_pos['longitude']:.7f}°")
                print(f"  Altitude:  {ins_pos['altitude']:.2f} m")
            
            async_pos = position_interface.get_position_async(timeout=0.5)
            if async_pos:
                print(f"\nMethod 3 - Async Measurement:")
                print(f"  Latitude:  {async_pos['latitude']:.7f}°")
                print(f"  Longitude: {async_pos['longitude']:.7f}°")
                print(f"  Altitude:  {async_pos['altitude']:.2f} m")
            
            uncertainty = position_interface.get_position_uncertainty()
            if uncertainty:
                print(f"\nMethod 4 - Position Uncertainty:")
                print(f"  North Uncertainty: {uncertainty['north_uncertainty']:.2f} m")
                print(f"  East Uncertainty:  {uncertainty['east_uncertainty']:.2f} m")
                print(f"  Down Uncertainty:  {uncertainty['down_uncertainty']:.2f} m")
                print(f"  GPS Fix Status:    {uncertainty['gps_fix']}")
            
            ecef_pos = position_interface.get_position_ecef()
            if ecef_pos:
                print(f"\nMethod 5 - ECEF Coordinates:")
                print(f"  ECEF X: {ecef_pos['ecef_x']:.2f} m")
                print(f"  ECEF Y: {ecef_pos['ecef_y']:.2f} m")
                print(f"  ECEF Z: {ecef_pos['ecef_z']:.2f} m")
            
            print("\n" + "-" * 40)
            time.sleep(2)  
            
    except KeyboardInterrupt:
        pass
    finally:
        position_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
