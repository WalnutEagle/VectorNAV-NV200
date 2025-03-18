import time
import sys
import argparse
import signal
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200INSInterface:
    
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
            
            # Read model info to verify connection
            model_register = Registers.Model()
            self.sensor.readRegister(model_register)
            print(f"Device: {model_register.model}")
            
            return True
        except VnError as e:
            print(f"Error connecting to VN-200: {e}")
            return False
    
    def configure_ins_output(self, update_rate_hz=10):
        try:
            rate_divisor = int(800 / update_rate_hz)
            binary_output = Registers.BinaryOutput1()
            binary_output.rateDivisor = rate_divisor
            binary_output.asyncMode.serial1 = 1
            binary_output.common.timeStartup = 1      # Timestamp
            binary_output.ins.insStatus = 1           # INS status
            binary_output.ins.posLla = 1              # Position (Lat, Lon, Alt)
            binary_output.ins.velNed = 1              # Velocity (NED)
            binary_output.ins.posU = 1                # Position uncertainty
            binary_output.ins.velU = 1                # Velocity uncertainty
            
            self.sensor.writeRegister(binary_output)
            print(f"INS status and solution output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring INS output: {e}")
            return False
    
    def get_ins_status(self):
        try:
            ins_status_register = Registers.InsStatus()
            self.sensor.readRegister(ins_status_register)
            
            mode_strs = [
                "Not Tracking",
                "Tracking, No GPS",
                "GPS Heading",
                "GPS + INS Heading",
                "INS Solution",
                "INS + GNSS Solution"
            ]
            
            try:
                mode_str = mode_strs[ins_status_register.mode]
            except IndexError:
                mode_str = f"Unknown ({ins_status_register.mode})"
            
            status_flags = {}
            status = ins_status_register.status
            status_flags["gps_fix"] = bool(status & 0x01)
            status_flags["time_error"] = bool(status & 0x02)
            status_flags["imu_error"] = bool(status & 0x04)
            status_flags["mag_pres_error"] = bool(status & 0x08)
            status_flags["gps_error"] = bool(status & 0x10)
            
            if status == 0 and ins_status_register.mode >= 3:
                solution_status = "Good"
            elif status_flags["gps_fix"] and not any([
                status_flags["time_error"], 
                status_flags["imu_error"],
                status_flags["mag_pres_error"],
                status_flags["gps_error"]
            ]):
                solution_status = "Degraded"
            else:
                solution_status = "Poor"
            
            return {
                "mode": ins_status_register.mode,
                "mode_str": mode_str,
                "status": status,
                "status_flags": status_flags,
                "solution_status": solution_status,
                "method": "ins_status_register"
            }
        except VnError as e:
            print(f"Error reading INS status: {e}")
            return None
    
    def get_ins_position(self):
        try:
            ins_lla_register = Registers.InsLla()
            self.sensor.readRegister(ins_lla_register)

            latitude = ins_lla_register.position.x
            longitude = ins_lla_register.position.y
            altitude = ins_lla_register.position.z
            
            return {
                "latitude": latitude,   # Latitude in degrees
                "longitude": longitude, # Longitude in degrees
                "altitude": altitude,   # Altitude in meters
                "method": "ins_position_register"
            }
        except VnError as e:
            print(f"Error reading INS position: {e}")
            return None
    
    def get_ins_velocity(self):
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
                "method": "ins_velocity_register"
            }
        except VnError as e:
            print(f"Error reading INS velocity: {e}")
            return None
    
    def get_position_uncertainty(self):
        try:
            pos_u_register = Registers.PosUncertainty()
            self.sensor.readRegister(pos_u_register)
            
            return {
                "north_uncertainty": pos_u_register.posU.x,  # North position uncertainty (m)
                "east_uncertainty": pos_u_register.posU.y,   # East position uncertainty (m)
                "down_uncertainty": pos_u_register.posU.z,   # Down position uncertainty (m)
                "horizontal_uncertainty": np.sqrt(
                    pos_u_register.posU.x**2 + pos_u_register.posU.y**2
                ),
                "method": "position_uncertainty_register"
            }
        except VnError as e:
            print(f"Error reading position uncertainty: {e}")
            return None
    
    def get_velocity_uncertainty(self):
        try:
            vel_u_register = Registers.VelUncertainty()
            self.sensor.readRegister(vel_u_register)
            
            return {
                "north_uncertainty": vel_u_register.velU.x,  # North velocity uncertainty (m/s)
                "east_uncertainty": vel_u_register.velU.y,   # East velocity uncertainty (m/s)
                "down_uncertainty": vel_u_register.velU.z,   # Down velocity uncertainty (m/s)
                "horizontal_uncertainty": np.sqrt(
                    vel_u_register.velU.x**2 + vel_u_register.velU.y**2
                ),
                "method": "velocity_uncertainty_register"
            }
        except VnError as e:
            print(f"Error reading velocity uncertainty: {e}")
            return None
    
    def get_ins_solution(self):
        try:
            status_data = self.get_ins_status()
            position_data = self.get_ins_position()
            velocity_data = self.get_ins_velocity()
            
            pos_uncertainty = self.get_position_uncertainty()
            vel_uncertainty = self.get_velocity_uncertainty()
            
            solution = {
                "method": "combined_ins_solution"
            }
            
            if status_data:
                solution["mode"] = status_data["mode"]
                solution["mode_str"] = status_data["mode_str"]
                solution["solution_status"] = status_data["solution_status"]
                solution["status_flags"] = status_data["status_flags"]
            
            if position_data:
                solution["latitude"] = position_data["latitude"]
                solution["longitude"] = position_data["longitude"]
                solution["altitude"] = position_data["altitude"]
            
            if velocity_data:
                solution["north_velocity"] = velocity_data["north"]
                solution["east_velocity"] = velocity_data["east"]
                solution["down_velocity"] = velocity_data["down"]
                solution["horizontal_speed"] = velocity_data["horizontal_speed"]
                solution["speed_3d"] = velocity_data["speed_3d"]
                solution["course"] = velocity_data["course"]
            
            if pos_uncertainty:
                solution["position_uncertainty"] = {
                    "north": pos_uncertainty["north_uncertainty"],
                    "east": pos_uncertainty["east_uncertainty"],
                    "down": pos_uncertainty["down_uncertainty"],
                    "horizontal": pos_uncertainty["horizontal_uncertainty"]
                }
            
            if vel_uncertainty:
                solution["velocity_uncertainty"] = {
                    "north": vel_uncertainty["north_uncertainty"],
                    "east": vel_uncertainty["east_uncertainty"],
                    "down": vel_uncertainty["down_uncertainty"],
                    "horizontal": vel_uncertainty["horizontal_uncertainty"]
                }
            
            return solution
        except Exception as e:
            print(f"Error getting combined INS solution: {e}")
            return None
    
    def get_ins_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  # Small sleep to avoid CPU hogging
                    continue
                
                result = {"method": "async_measurement"}
                has_data = False
                
                if cd.ins.insStatus:
                    result["mode"] = cd.ins.insStatus.mode
                    
                    status = cd.ins.insStatus.status
                    result["status_flags"] = {
                        "gps_fix": bool(status & 0x01),
                        "time_error": bool(status & 0x02),
                        "imu_error": bool(status & 0x04),
                        "mag_pres_error": bool(status & 0x08),
                        "gps_error": bool(status & 0x10)
                    }
                    has_data = True
                
                if cd.ins.posLla:
                    result["latitude"] = cd.ins.posLla.x
                    result["longitude"] = cd.ins.posLla.y
                    result["altitude"] = cd.ins.posLla.z
                    has_data = True
                
                if cd.ins.velNed:
                    result["north_velocity"] = cd.ins.velNed.x
                    result["east_velocity"] = cd.ins.velNed.y
                    result["down_velocity"] = cd.ins.velNed.z
                    
                    result["horizontal_speed"] = np.sqrt(
                        cd.ins.velNed.x**2 + cd.ins.velNed.y**2
                    )
                    result["course"] = np.degrees(
                        np.arctan2(cd.ins.velNed.y, cd.ins.velNed.x)
                    ) % 360
                    has_data = True
                
                if cd.ins.posU:
                    result["position_uncertainty"] = {
                        "north": cd.ins.posU.x,
                        "east": cd.ins.posU.y,
                        "down": cd.ins.posU.z,
                        "horizontal": np.sqrt(cd.ins.posU.x**2 + cd.ins.posU.y**2)
                    }
                    has_data = True
                
                if cd.ins.velU:
                    result["velocity_uncertainty"] = {
                        "north": cd.ins.velU.x,
                        "east": cd.ins.velU.y,
                        "down": cd.ins.velU.z,
                        "horizontal": np.sqrt(cd.ins.velU.x**2 + cd.ins.velU.y**2)
                    }
                    has_data = True
                
                if cd.time.timeStartup:
                    result["timestamp"] = cd.time.timeStartup.nanoseconds() / 1e9
                
                if has_data:
                    return result
            
            print("Timeout waiting for INS data")
            return None
        except VnError as e:
            print(f"Error getting async INS data: {e}")
            return None
    
    def get_ins_state_flags(self):
        try:
            status_data = self.get_ins_status()
            if not status_data:
                return None
            
            mode = status_data["mode"]
            status = status_data["status"]
            
            ins_state = {
                "mode": mode,
                "mode_str": status_data["mode_str"],
                "solution_status": status_data["solution_status"],
                "raw_status": status,
                "method": "ins_state_flags"
            }
            
            ins_state["has_gps_fix"] = bool(status & 0x01)
            ins_state["has_time_error"] = bool(status & 0x02)
            ins_state["has_imu_error"] = bool(status & 0x04)
            ins_state["has_mag_pres_error"] = bool(status & 0x08)
            ins_state["has_gps_error"] = bool(status & 0x10)
            
            if mode == 0:
                ins_state["tracking_status"] = "Not tracking"
                ins_state["heading_source"] = "None"
            elif mode == 1:
                ins_state["tracking_status"] = "Tracking without GPS"
                ins_state["heading_source"] = "Magnetometer or external"
            elif mode == 2:
                ins_state["tracking_status"] = "Tracking with GPS"
                ins_state["heading_source"] = "GPS velocity vector"
            elif mode == 3:
                ins_state["tracking_status"] = "Tracking with GPS"
                ins_state["heading_source"] = "GPS + IMU integration"
            elif mode == 4:
                ins_state["tracking_status"] = "Full INS solution"
                ins_state["heading_source"] = "IMU integration"
            elif mode == 5:
                ins_state["tracking_status"] = "Full INS + GNSS solution"
                ins_state["heading_source"] = "Integrated IMU + GNSS"
            else:
                ins_state["tracking_status"] = "Unknown"
                ins_state["heading_source"] = "Unknown"
            
            if ins_state["solution_status"] == "Good":
                ins_state["quality"] = "High - Full solution available"
            elif ins_state["solution_status"] == "Degraded":
                ins_state["quality"] = "Medium - Solution available but degraded"
            else:
                ins_state["quality"] = "Low - Solution may be unreliable"
            
            return ins_state
        except Exception as e:
            print(f"Error getting INS state flags: {e}")
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
    parser = argparse.ArgumentParser(description='VN-200 INS Status and Solution Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=10, help='Update rate in Hz')
    args = parser.parse_args()
    
    ins_interface = VN200INSInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        ins_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not ins_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not ins_interface.configure_ins_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        ins_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 INS Status and Solution Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            status_data = ins_interface.get_ins_status()
            if status_data:
                print(f"\nMethod 1 - INS Status Information:")
                print(f"  Mode:           {status_data['mode_str']} ({status_data['mode']})")
                print(f"  Solution Status: {status_data['solution_status']}")
                print(f"  Status Flags:")
                for flag, value in status_data['status_flags'].items():
                    print(f"    {flag}: {'Yes' if value else 'No'}")
            
            position_data = ins_interface.get_ins_position()
            if position_data:
                print(f"\nMethod 2 - INS Position Solution:")
                print(f"  Latitude:  {position_data['latitude']:.6f}°")
                print(f"  Longitude: {position_data['longitude']:.6f}°")
                print(f"  Altitude:  {position_data['altitude']:.2f} m")
            
            velocity_data = ins_interface.get_ins_velocity()
            if velocity_data:
                print(f"\nMethod 3 - INS Velocity Solution:")
                print(f"  North:   {velocity_data['north']:.2f} m/s")
                print(f"  East:    {velocity_data['east']:.2f} m/s")
                print(f"  Down:    {velocity_data['down']:.2f} m/s")
                print(f"  Horizontal Speed: {velocity_data['horizontal_speed']:.2f} m/s")
                print(f"  3D Speed:         {velocity_data['speed_3d']:.2f} m/s")
                print(f"  Course:           {velocity_data['course']:.2f}°")
            
            pos_uncertainty = ins_interface.get_position_uncertainty()
            if pos_uncertainty:
                print(f"\nMethod 4 - Position Uncertainty:")
                print(f"  North Uncertainty:      {pos_uncertainty['north_uncertainty']:.2f} m")
                print(f"  East Uncertainty:       {pos_uncertainty['east_uncertainty']:.2f} m")
                print(f"  Down Uncertainty:       {pos_uncertainty['down_uncertainty']:.2f} m")
                print(f"  Horizontal Uncertainty: {pos_uncertainty['horizontal_uncertainty']:.2f} m")
            
            vel_uncertainty = ins_interface.get_velocity_uncertainty()
            if vel_uncertainty:
                print(f"\nMethod 5 - Velocity Uncertainty:")
                print(f"  North Uncertainty:      {vel_uncertainty['north_uncertainty']:.2f} m/s")
                print(f"  East Uncertainty:       {vel_uncertainty['east_uncertainty']:.2f} m/s")
                print(f"  Down Uncertainty:       {vel_uncertainty['down_uncertainty']:.2f} m/s")
                print(f"  Horizontal Uncertainty: {vel_uncertainty['horizontal_uncertainty']:.2f} m/s")
            
            state_flags = ins_interface.get_ins_state_flags()
            if state_flags:
                print(f"\nMethod 8 - INS State Flags (Detailed):")
                print(f"  Tracking Status: {state_flags['tracking_status']}")
                print(f"  Heading Source:  {state_flags['heading_source']}")
                print(f"  Solution Quality: {state_flags['quality']}")
            
            async_data = ins_interface.get_ins_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 7 - Asynchronous INS Data:")
                
                if "mode" in async_data:
                    print(f"  Mode: {async_data['mode']}")
                    
                    if "status_flags" in async_data:
                        flags = async_data["status_flags"]
                        for flag, value in flags.items():
                            if value:
                                print(f"    {flag}: Yes")
                
                if "latitude" in async_data:
                    print(f"  Position: {async_data['latitude']:.6f}°, {async_data['longitude']:.6f}°, {async_data['altitude']:.2f} m")
                
                if "north_velocity" in async_data:
                    print(f"  Velocity: {async_data['horizontal_speed']:.2f} m/s at {async_data['course']:.2f}°")
                
                if "position_uncertainty" in async_data:
                    pu = async_data["position_uncertainty"]
                    print(f"  Position Uncertainty: {pu['horizontal']:.2f} m horizontal")
                
                if "velocity_uncertainty" in async_data:
                    vu = async_data["velocity_uncertainty"]
                    print(f"  Velocity Uncertainty: {vu['horizontal']:.2f} m/s horizontal")
            
            combined_solution = ins_interface.get_ins_solution()
            if combined_solution:
                print(f"\nMethod 6 - Combined INS Solution:")
                print(f"  Comprehensive INS solution data is available.")
                print(f"  Mode: {combined_solution.get('mode_str', 'N/A')}")
                print(f"  Status: {combined_solution.get('solution_status', 'N/A')}")
                
                if "latitude" in combined_solution:
                    lat = combined_solution["latitude"]
                    lon = combined_solution["longitude"]
                    alt = combined_solution["altitude"]
                    print(f"  Position: {lat:.6f}°, {lon:.6f}°, {alt:.2f} m")
                
                if "horizontal_speed" in combined_solution:
                    speed = combined_solution["horizontal_speed"]
                    course = combined_solution["course"]
                    print(f"  Movement: {speed:.2f} m/s at {course:.2f}°")
            
            print("\n" + "-" * 40)
            time.sleep(2)  # Pause between updates
            
    except KeyboardInterrupt:
        pass
    finally:
        ins_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
