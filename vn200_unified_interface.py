import time
import sys
import argparse
import signal
from vn200_position_data import VN200PositionInterface
from vn200_orientation_data import VN200OrientationInterface
from vn200_velocity_data import VN200VelocityInterface
from vn200_gps_data import VN200GpsInterface


class VN200UnifiedInterface:
    
    def __init__(self, port="/dev/ttyUSB0", baud_rate=115200):

        self.port = port
        self.baud_rate = baud_rate
        
        self.position = VN200PositionInterface(port=port, baud_rate=baud_rate)
        self.orientation = VN200OrientationInterface(port=port, baud_rate=baud_rate)
        self.velocity = VN200VelocityInterface(port=port, baud_rate=baud_rate)
        self.gps = VN200GpsInterface(port=port, baud_rate=baud_rate)
        
        self.connected = False
    
    def connect(self):
        if self.position.connect():
            self.connected = True
            sensor = self.position.sensor
            self.orientation.sensor = sensor
            self.orientation.connected = True           
            self.velocity.sensor = sensor
            self.velocity.connected = True            
            self.gps.sensor = sensor
            self.gps.connected = True
            
            return True
        
        return False
    
    def configure_outputs(self, position_rate=20, orientation_rate=50, 
                         velocity_rate=20, gps_rate=5):
        success = True
        if not self.position.configure_position_output(update_rate_hz=position_rate):
            print("Failed to configure position output")
            success = False
        
        if not self.orientation.configure_orientation_output(update_rate_hz=orientation_rate):
            print("Failed to configure orientation output")
            success = False
        
        if not self.velocity.configure_velocity_output(update_rate_hz=velocity_rate):
            print("Failed to configure velocity output")
            success = False
        
        if not self.gps.configure_gps_output(update_rate_hz=gps_rate):
            print("Failed to configure GPS output")
            success = False
        
        return success
    
    def get_navigation_solution(self):
        position_data = self.position.get_position_lla()
        orientation_data = self.orientation.get_yaw_pitch_roll()
        quaternion_data = self.orientation.get_quaternion()
        velocity_data = self.velocity.get_velocity_ned()
        gps_data = self.gps.get_gps_fix()
        nav_solution = {
            "timestamp": time.time(),
            "source": "vn200_unified_interface"
        }
        
        if position_data:
            nav_solution["position"] = {
                "latitude": position_data.get("latitude"),
                "longitude": position_data.get("longitude"),
                "altitude": position_data.get("altitude")
            }
        
        if orientation_data:
            nav_solution["orientation"] = {
                "yaw": orientation_data.get("yaw"),
                "pitch": orientation_data.get("pitch"),
                "roll": orientation_data.get("roll")
            }
        
        if quaternion_data:
            nav_solution["quaternion"] = {
                "w": quaternion_data.get("w"),
                "x": quaternion_data.get("x"),
                "y": quaternion_data.get("y"),
                "z": quaternion_data.get("z")
            }
        
        if velocity_data:
            nav_solution["velocity"] = {
                "north": velocity_data.get("north"),
                "east": velocity_data.get("east"),
                "down": velocity_data.get("down"),
                "speed": velocity_data.get("horizontal_speed"),
                "course": velocity_data.get("course")
            }
        
        if gps_data:
            nav_solution["gps_status"] = {
                "fix_type": gps_data.get("fix_type"),
                "fix_type_str": gps_data.get("fix_type_str"),
                "satellites_used": gps_data.get("satellites_used")
            }
        
        return nav_solution
    
    def get_position_orientation(self):
        position_data = self.position.get_position_lla()
        orientation_data = self.orientation.get_yaw_pitch_roll()
        
        if not position_data or not orientation_data:
            return None
        
        return {
            "timestamp": time.time(),
            "latitude": position_data.get("latitude"),
            "longitude": position_data.get("longitude"),
            "altitude": position_data.get("altitude"),
            "yaw": orientation_data.get("yaw"),
            "pitch": orientation_data.get("pitch"),
            "roll": orientation_data.get("roll")
        }
    
    def get_velocity_heading(self):
        velocity_data = self.velocity.get_velocity_ned()
        orientation_data = self.orientation.get_yaw_pitch_roll()
        
        if not velocity_data:
            return None
        
        result = {
            "timestamp": time.time(),
            "speed": velocity_data.get("horizontal_speed"),
            "vertical_speed": -velocity_data.get("down"),
            "course_over_ground": velocity_data.get("course")
        }
        
        if orientation_data:
            result["true_heading"] = orientation_data.get("yaw")
            
            result["crab_angle"] = (result["course_over_ground"] - 
                                   result["true_heading"]) % 360
            if result["crab_angle"] > 180:
                result["crab_angle"] -= 360
        
        return result
    
    def get_gps_position_update(self):
        position_data = self.position.get_position_lla()
        gps_fix = self.gps.get_gps_fix()
        gps_dop = self.gps.get_gps_dop()
        
        if not position_data or not gps_fix:
            return None
        
        result = {
            "timestamp": time.time(),
            "latitude": position_data.get("latitude"),
            "longitude": position_data.get("longitude"),
            "altitude": position_data.get("altitude"),
            "fix_type": gps_fix.get("fix_type"),
            "fix_type_str": gps_fix.get("fix_type_str"),
            "satellites_used": gps_fix.get("satellites_used")
        }
        
        if gps_dop:
            result["hdop"] = gps_dop.get("horizontal_dop")
            result["vdop"] = gps_dop.get("vertical_dop")
            result["pdop"] = gps_dop.get("position_dop")
        
        return result
    
    def get_all_async_data(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.position.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)
                    continue
                
                result = {
                    "timestamp": time.time(),
                    "source": "async_measurement"
                }
                
                if cd.position.positionLla:
                    result["position"] = {
                        "latitude": cd.position.positionLla.x,
                        "longitude": cd.position.positionLla.y,
                        "altitude": cd.position.positionLla.z
                    }
                
                if cd.attitude.yawPitchRoll:
                    result["orientation"] = {
                        "yaw": cd.attitude.yawPitchRoll.x,
                        "pitch": cd.attitude.yawPitchRoll.y,
                        "roll": cd.attitude.yawPitchRoll.z
                    }
                
                if cd.attitude.quaternion:
                    result["quaternion"] = {
                        "w": cd.attitude.quaternion.w,
                        "x": cd.attitude.quaternion.x,
                        "y": cd.attitude.quaternion.y,
                        "z": cd.attitude.quaternion.z
                    }
                
                if cd.velocity.velNed:
                    result["velocity"] = {
                        "north": cd.velocity.velNed.x,
                        "east": cd.velocity.velNed.y,
                        "down": cd.velocity.velNed.z
                    }
                
                if cd.gps.gpsFix:
                    result["gps"] = {
                        "fix_type": cd.gps.gpsFix.fix,
                        "satellites_used": cd.gps.gpsFix.numSats
                    }
                
                if len(result) > 2: 
                    return result
            
            print("Timeout waiting for async data")
            return None
        except Exception as e:
            print(f"Error getting async data: {e}")
            return None
    
    def disconnect(self):
        if self.connected:
            self.position.disconnect()
            self.orientation.connected = False
            self.velocity.connected = False
            self.gps.connected = False
            self.connected = False


def main():
    parser = argparse.ArgumentParser(description='VN-200 Unified Interface Demo')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    args = parser.parse_args()
    
    vn200 = VN200UnifiedInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        vn200.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Connecting to VN-200 sensor...")
    if not vn200.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    print("Configuring sensor outputs...")
    if not vn200.configure_outputs(
        position_rate=20,
        orientation_rate=50,
        velocity_rate=20,
        gps_rate=5
    ):
        print("Warning: Some outputs could not be configured properly.")
        print("Continuing with available data...")
    
    try:
        print("\n=== VN-200 Unified Interface Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            nav_solution = vn200.get_navigation_solution()
            if nav_solution:
                print("\n--- Comprehensive Navigation Solution ---")
                
                if "position" in nav_solution:
                    pos = nav_solution["position"]
                    print(f"Position: {pos['latitude']:.6f}°, {pos['longitude']:.6f}°, {pos['altitude']:.2f}m")
                
                if "orientation" in nav_solution:
                    ori = nav_solution["orientation"]
                    print(f"Orientation: Yaw {ori['yaw']:.2f}°, Pitch {ori['pitch']:.2f}°, Roll {ori['roll']:.2f}°")
                
                if "velocity" in nav_solution:
                    vel = nav_solution["velocity"]
                    print(f"Velocity: {vel['speed']:.2f} m/s at {vel['course']:.2f}° course")
                    print(f"  North: {vel['north']:.2f} m/s, East: {vel['east']:.2f} m/s, Down: {vel['down']:.2f} m/s")
                
                if "gps_status" in nav_solution:
                    gps = nav_solution["gps_status"]
                    print(f"GPS: {gps['fix_type_str']} ({gps['satellites_used']} satellites)")
            
            pos_ori = vn200.get_position_orientation()
            if pos_ori:
                print("\n--- Position & Orientation ---")
                print(f"Location: {pos_ori['latitude']:.6f}°, {pos_ori['longitude']:.6f}°, {pos_ori['altitude']:.2f}m")
                print(f"Attitude: Yaw {pos_ori['yaw']:.2f}°, Pitch {pos_ori['pitch']:.2f}°, Roll {pos_ori['roll']:.2f}°")
            
            vel_heading = vn200.get_velocity_heading()
            if vel_heading:
                print("\n--- Velocity & Heading ---")
                print(f"Speed: {vel_heading['speed']:.2f} m/s")
                print(f"Vertical Speed: {vel_heading['vertical_speed']:.2f} m/s (+ is up)")
                print(f"Course over Ground: {vel_heading['course_over_ground']:.2f}°")
                
                if "true_heading" in vel_heading:
                    print(f"True Heading: {vel_heading['true_heading']:.2f}°")
                    print(f"Crab Angle: {vel_heading['crab_angle']:.2f}°")
            
            gps_pos = vn200.get_gps_position_update()
            if gps_pos:
                print("\n--- GPS Position Update ---")
                print(f"Location: {gps_pos['latitude']:.6f}°, {gps_pos['longitude']:.6f}°, {gps_pos['altitude']:.2f}m")
                print(f"Fix: {gps_pos['fix_type_str']} using {gps_pos['satellites_used']} satellites")
                
                if "hdop" in gps_pos:
                    print(f"Quality: HDOP {gps_pos['hdop']:.2f}, VDOP {gps_pos['vdop']:.2f}")
            
            async_data = vn200.get_all_async_data(timeout=0.5)
            if async_data:
                print("\n--- Asynchronous Data ---")
                
                if "position" in async_data:
                    pos = async_data["position"]
                    print(f"Position: {pos['latitude']:.6f}°, {pos['longitude']:.6f}°, {pos['altitude']:.2f}m")
                
                if "orientation" in async_data:
                    ori = async_data["orientation"]
                    print(f"Orientation: Yaw {ori['yaw']:.2f}°, Pitch {ori['pitch']:.2f}°, Roll {ori['roll']:.2f}°")
                
                if "velocity" in async_data:
                    vel = async_data["velocity"]
                    north = vel["north"]
                    east = vel["east"]
                    speed = (north**2 + east**2)**0.5
                    print(f"Velocity: {speed:.2f} m/s")
                
                if "gps" in async_data:
                    gps = async_data["gps"]
                    print(f"GPS: Fix type {gps['fix_type']} ({gps['satellites_used']} satellites)")
            
            print("\n" + "-" * 60)
            time.sleep(2)
            
    except KeyboardInterrupt:
        pass
    finally:
        vn200.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
