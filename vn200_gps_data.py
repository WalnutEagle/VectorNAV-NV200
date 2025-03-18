import time
import sys
import argparse
import signal
import datetime
from vectornav import Sensor, Registers, VnError

class VN200GpsInterface:
    
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
    
    def configure_gps_output(self, update_rate_hz=5):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            

            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeGps = 1          # GPS time
            binary_output.common.timeUtc = 1          # UTC time
            binary_output.common.gps = 1              # GPS data
            
            binary_output.gps.gps1 = 1                # GPS fix info
            binary_output.gps.gps2 = 1                # GPS satellite info
            binary_output.gps.gpsDop = 1              # GPS DOP data
            
            self.sensor.writeRegister(binary_output)
            print(f"GPS output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring GPS output: {e}")
            return False
    
    def get_gps_fix(self):
        try:
            gps_fix_register = Registers.GpsFix()
            self.sensor.readRegister(gps_fix_register)
            
            fix_types = [
                "No Fix",
                "Time Only",
                "2D Fix",
                "3D Fix",
                "3D + SBAS Fix",
                "3D + RTK Fix"
            ]
            
            try:
                fix_type_str = fix_types[gps_fix_register.fix]
            except IndexError:
                fix_type_str = f"Unknown ({gps_fix_register.fix})"
            
            return {
                "fix_type": gps_fix_register.fix,
                "fix_type_str": fix_type_str,
                "satellites_used": gps_fix_register.numSats,
                "fix_mode": gps_fix_register.fixMode,
                "method": "gps_fix_register"
            }
        except VnError as e:
            print(f"Error reading GPS fix: {e}")
            return None
    
    def get_gps_satellites(self):
        try:
            gps_sat_register = Registers.GpsSat()
            self.sensor.readRegister(gps_sat_register)
            
            return {
                "satellites_visible": gps_sat_register.numSats,
                "satellites_used": gps_sat_register.satsUsed,
                "method": "gps_satellites_register"
            }
        except VnError as e:
            print(f"Error reading GPS satellites: {e}")
            return None
    
    def get_gps_dop(self):
        try:
            gps_dop_register = Registers.GpsDop()
            self.sensor.readRegister(gps_dop_register)
            
            return {
                "geometric_dop": gps_dop_register.gDop,     # Geometric DOP
                "position_dop": gps_dop_register.pDop,      # Position DOP
                "time_dop": gps_dop_register.tDop,          # Time DOP
                "horizontal_dop": gps_dop_register.hDop,    # Horizontal DOP
                "vertical_dop": gps_dop_register.vDop,      # Vertical DOP
                "method": "gps_dop_register"
            }
        except VnError as e:
            print(f"Error reading GPS DOP: {e}")
            return None
    
    def get_gps_time(self):
        try:
            gps_time_register = Registers.GpsTime()
            self.sensor.readRegister(gps_time_register)
            
            time_of_week = gps_time_register.tow          # Time of week in seconds
            week_number = gps_time_register.week          # GPS week number
            
            utc_time_register = Registers.TimeUtc()
            self.sensor.readRegister(utc_time_register)
            
            utc_time_str = f"{utc_time_register.year:04d}-{utc_time_register.month:02d}-{utc_time_register.day:02d} "
            utc_time_str += f"{utc_time_register.hour:02d}:{utc_time_register.min:02d}:{utc_time_register.sec:02d}.{utc_time_register.ms:03d}"
            
            try:
                utc_datetime = datetime.datetime(
                    utc_time_register.year,
                    utc_time_register.month,
                    utc_time_register.day,
                    utc_time_register.hour,
                    utc_time_register.min,
                    utc_time_register.sec,
                    utc_time_register.ms * 1000
                )
            except ValueError:
                utc_datetime = None
            
            return {
                "gps_tow": time_of_week,            # GPS time of week (seconds)
                "gps_week": week_number,            # GPS week number
                "utc_year": utc_time_register.year,
                "utc_month": utc_time_register.month,
                "utc_day": utc_time_register.day,
                "utc_hour": utc_time_register.hour,
                "utc_minute": utc_time_register.min,
                "utc_second": utc_time_register.sec,
                "utc_millisecond": utc_time_register.ms,
                "utc_time_str": utc_time_str,       # Formatted UTC time string
                "utc_datetime": utc_datetime,       # Python datetime object (if valid)
                "method": "gps_time_register"
            }
        except VnError as e:
            print(f"Error reading GPS time: {e}")
            return None
        except Exception as e:
            print(f"Error processing GPS time: {e}")
            return None
    
    def get_ins_gps_status(self):
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
            
            return {
                "mode": ins_status_register.mode,
                "mode_str": mode_str,
                "status": status,
                "status_flags": status_flags,
                "method": "ins_status_register"
            }
        except VnError as e:
            print(f"Error reading INS GPS status: {e}")
            return None
    
    def get_gps_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  
                    continue
                
                if cd.gps.gpsFix:
                    result = {
                        "method": "async_measurement",
                        "fix_type": cd.gps.gpsFix.fix
                    }
                    
                    if cd.gps.gpsSat:
                        result["satellites_visible"] = cd.gps.gpsSat.numSats
                        result["satellites_used"] = cd.gps.gpsSat.satsUsed
                    
                    if cd.time.timeUtc:
                        result["utc_year"] = cd.time.timeUtc.year
                        result["utc_month"] = cd.time.timeUtc.month
                        result["utc_day"] = cd.time.timeUtc.day
                        result["utc_hour"] = cd.time.timeUtc.hour
                        result["utc_minute"] = cd.time.timeUtc.min
                        result["utc_second"] = cd.time.timeUtc.sec
                        result["utc_millisecond"] = cd.time.timeUtc.ms
                    
                    return result
            
            print("Timeout waiting for GPS data")
            return None
        except VnError as e:
            print(f"Error getting async GPS data: {e}")
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
    parser = argparse.ArgumentParser(description='VN-200 GPS Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=5, help='Update rate in Hz')
    args = parser.parse_args()
    
    gps_interface = VN200GpsInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        gps_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not gps_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not gps_interface.configure_gps_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        gps_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 GPS Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            fix_data = gps_interface.get_gps_fix()
            if fix_data:
                print(f"\nMethod 1 - GPS Fix Information:")
                print(f"  Fix Type:        {fix_data['fix_type_str']} ({fix_data['fix_type']})")
                print(f"  Satellites Used: {fix_data['satellites_used']}")
                print(f"  Fix Mode:        {fix_data['fix_mode']}")
            
            sat_data = gps_interface.get_gps_satellites()
            if sat_data:
                print(f"\nMethod 2 - GPS Satellite Information:")
                print(f"  Satellites Visible: {sat_data['satellites_visible']}")
                print(f"  Satellites Used:    {sat_data['satellites_used']}")
            
            dop_data = gps_interface.get_gps_dop()
            if dop_data:
                print(f"\nMethod 3 - GPS Dilution of Precision (DOP):")
                print(f"  Geometric DOP:  {dop_data['geometric_dop']:.2f}")
                print(f"  Position DOP:   {dop_data['position_dop']:.2f}")
                print(f"  Horizontal DOP: {dop_data['horizontal_dop']:.2f}")
                print(f"  Vertical DOP:   {dop_data['vertical_dop']:.2f}")
                print(f"  Time DOP:       {dop_data['time_dop']:.2f}")
            
            time_data = gps_interface.get_gps_time()
            if time_data:
                print(f"\nMethod 4 - GPS Time Information:")
                print(f"  UTC Time:    {time_data['utc_time_str']}")
                print(f"  GPS Week:    {time_data['gps_week']}")
                print(f"  GPS Time of Week: {time_data['gps_tow']:.3f} seconds")
            
            ins_status = gps_interface.get_ins_gps_status()
            if ins_status:
                print(f"\nMethod 5 - INS/GPS Status:")
                print(f"  Mode:        {ins_status['mode_str']} ({ins_status['mode']})")
                print(f"  Status Flags:")
                flags = ins_status['status_flags']
                for flag, value in flags.items():
                    print(f"    {flag}: {'Yes' if value else 'No'}")
            
            async_data = gps_interface.get_gps_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 6 - Asynchronous GPS Data:")
                print(f"  Fix Type: {async_data.get('fix_type', 'N/A')}")
                
                if "satellites_visible" in async_data:
                    print(f"  Satellites Visible: {async_data['satellites_visible']}")
                    print(f"  Satellites Used:    {async_data['satellites_used']}")
                
                if "utc_year" in async_data:
                    time_str = f"{async_data['utc_year']:04d}-{async_data['utc_month']:02d}-{async_data['utc_day']:02d} "
                    time_str += f"{async_data['utc_hour']:02d}:{async_data['utc_minute']:02d}:{async_data['utc_second']:02d}"
                    print(f"  UTC Time: {time_str}")
            
            print("\n" + "-" * 40)
            time.sleep(2)  # Pause between updates
            
    except KeyboardInterrupt:
        pass
    finally:
        gps_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()