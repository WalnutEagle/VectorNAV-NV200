import time
import sys
import argparse
import signal
import math
from vectornav import Sensor, Registers, VnError

class VN200PressureTempInterface:
    
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
    
    def configure_pressure_temp_output(self, update_rate_hz=20):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.timeStartup = 1      # Timestamp
            binary_output.common.temp = 1             # Temperature
            binary_output.common.pres = 1             # Pressure
            
            self.sensor.writeRegister(binary_output)
            print(f"Pressure and temperature output configured at {update_rate_hz}Hz")
            
            return True
        except VnError as e:
            print(f"Error configuring pressure and temperature output: {e}")
            return False
    
    def get_pressure(self):
        try:
            pres_register = Registers.Pressure()
            self.sensor.readRegister(pres_register)
            
            # Pressure in millibars (hPa)
            pressure = pres_register.pressure
            
            return {
                "pressure": pressure,  # Barometric pressure in millibars (hPa)
                "method": "pressure_register"
            }
        except VnError as e:
            print(f"Error reading pressure register: {e}")
            return None
    
    def get_temperature(self):
        try:
            temp_register = Registers.Temperature()
            self.sensor.readRegister(temp_register)

            temperature = temp_register.temp
            
            return {
                "temperature": temperature,  # Temperature in degrees Celsius
                "temperature_f": (temperature * 9/5) + 32,  # Temperature in degrees Fahrenheit
                "method": "temperature_register"
            }
        except VnError as e:
            print(f"Error reading temperature register: {e}")
            return None
    
    def calculate_altitude(self, reference_pressure=1013.25):
        try:
            pressure_data = self.get_pressure()
            if not pressure_data:
                return None
            
            pressure = pressure_data["pressure"]
            
            altitude = 44330.0 * (1.0 - math.pow(pressure / reference_pressure, 1.0/5.255))
            
            return {
                "altitude": altitude,  # Estimated altitude in meters
                "altitude_ft": altitude * 3.28084,  # Altitude in feet
                "pressure": pressure,  # Current pressure in millibars
                "reference_pressure": reference_pressure,  # Reference pressure used
                "method": "barometric_altitude"
            }
        except Exception as e:
            print(f"Error calculating altitude: {e}")
            return None
    
    def get_temp_pressure_combined(self):
        try:
            temp_data = self.get_temperature()
            if not temp_data:
                return None
            
            pressure_data = self.get_pressure()
            if not pressure_data:
                return None
            
            combined_data = {
                "temperature": temp_data["temperature"],  # Temperature in degrees Celsius
                "temperature_f": temp_data["temperature_f"],  # Temperature in degrees Fahrenheit
                "pressure": pressure_data["pressure"],  # Pressure in millibars
                "method": "combined_temp_pressure"
            }
            
            altitude_data = self.calculate_altitude()
            if altitude_data:
                combined_data["altitude"] = altitude_data["altitude"]
                combined_data["altitude_ft"] = altitude_data["altitude_ft"]
            
            return combined_data
        except Exception as e:
            print(f"Error getting combined temperature and pressure: {e}")
            return None
    
    def get_pressure_temp_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  
                    continue
                
                result = {"method": "async_measurement"}
                has_data = False
                
                if cd.temperature.temp:
                    result["temperature"] = cd.temperature.temp
                    result["temperature_f"] = (cd.temperature.temp * 9/5) + 32
                    has_data = True
                
                if cd.pressure.pressure:
                    result["pressure"] = cd.pressure.pressure
                    has_data = True
                    
                    altitude = 44330.0 * (1.0 - math.pow(
                        cd.pressure.pressure / 1013.25, 1.0/5.255))
                    result["altitude"] = altitude
                    result["altitude_ft"] = altitude * 3.28084
                
                if cd.time.timeStartup:
                    result["timestamp"] = cd.time.timeStartup.nanoseconds() / 1e9
                
                if has_data:
                    return result
            
            print("Timeout waiting for pressure/temperature data")
            return None
        except VnError as e:
            print(f"Error getting async pressure/temperature data: {e}")
            return None
    
    def monitor_altitude_change(self, duration=10, interval=0.5, reference_pressure=None):
        try:
            print(f"Monitoring altitude for {duration} seconds...")
            
            initial_data = self.get_pressure()
            if not initial_data:
                return None
            
            if reference_pressure is None:
                reference_pressure = initial_data["pressure"]
            
            samples = []
            start_time = time.time()
            end_time = start_time + duration
            
            while time.time() < end_time:
                pressure_data = self.get_pressure()
                if pressure_data:
                    pressure = pressure_data["pressure"]
                    
                    altitude = 44330.0 * (1.0 - math.pow(
                        pressure / reference_pressure, 1.0/5.255))
                    
                    elapsed = time.time() - start_time
                    samples.append({
                        "time": elapsed,
                        "pressure": pressure,
                        "altitude": altitude
                    })
                
                time.sleep(interval)
            
            if not samples:
                return None
            
            altitudes = [s["altitude"] for s in samples]
            pressures = [s["pressure"] for s in samples]
            
            min_altitude = min(altitudes)
            max_altitude = max(altitudes)
            avg_altitude = sum(altitudes) / len(altitudes)
            
            min_pressure = min(pressures)
            max_pressure = max(pressures)
            avg_pressure = sum(pressures) / len(pressures)
            
            total_altitude_change = max_altitude - min_altitude
            initial_altitude = altitudes[0]
            final_altitude = altitudes[-1]
            net_altitude_change = final_altitude - initial_altitude
            
            return {
                "samples": samples,
                "sample_count": len(samples),
                "duration": duration,
                "reference_pressure": reference_pressure,
                "min_altitude": min_altitude,
                "max_altitude": max_altitude,
                "avg_altitude": avg_altitude,
                "total_altitude_change": total_altitude_change,
                "net_altitude_change": net_altitude_change,
                "min_pressure": min_pressure,
                "max_pressure": max_pressure,
                "avg_pressure": avg_pressure,
                "method": "altitude_monitoring"
            }
        except Exception as e:
            print(f"Error monitoring altitude: {e}")
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
    parser = argparse.ArgumentParser(description='VN-200 Pressure and Temperature Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    parser.add_argument('--ref-pressure', type=float, default=1013.25, 
                        help='Reference pressure at sea level in millibars (default: 1013.25)')
    parser.add_argument('--monitor', action='store_true', 
                        help='Monitor altitude changes for 30 seconds')
    args = parser.parse_args()
    
    pt_interface = VN200PressureTempInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        pt_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not pt_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not pt_interface.configure_pressure_temp_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        pt_interface.disconnect()
        sys.exit(1)
    
    if args.monitor:
        print("\n=== VN-200 Altitude Monitoring ===")
        print("Monitoring altitude changes for 30 seconds...")
        print("Move the sensor up and down to observe changes.")
        
        monitor_data = pt_interface.monitor_altitude_change(duration=30, interval=0.5, 
                                                           reference_pressure=args.ref_pressure)
        
        if monitor_data:
            print("\n=== Altitude Monitoring Results ===")
            print(f"Reference Pressure: {monitor_data['reference_pressure']:.2f} mbar")
            print(f"Samples Collected: {monitor_data['sample_count']}")
            print(f"Duration: {monitor_data['duration']:.1f} seconds")
            print(f"\nAltitude Statistics:")
            print(f"  Minimum Altitude: {monitor_data['min_altitude']:.2f} m ({monitor_data['min_altitude']*3.28084:.2f} ft)")
            print(f"  Maximum Altitude: {monitor_data['max_altitude']:.2f} m ({monitor_data['max_altitude']*3.28084:.2f} ft)")
            print(f"  Average Altitude: {monitor_data['avg_altitude']:.2f} m ({monitor_data['avg_altitude']*3.28084:.2f} ft)")
            print(f"  Total Range: {monitor_data['total_altitude_change']:.2f} m ({monitor_data['total_altitude_change']*3.28084:.2f} ft)")
            print(f"  Net Change: {monitor_data['net_altitude_change']:.2f} m ({monitor_data['net_altitude_change']*3.28084:.2f} ft)")
            
            print(f"\nPressure Statistics:")
            print(f"  Minimum Pressure: {monitor_data['min_pressure']:.2f} mbar")
            print(f"  Maximum Pressure: {monitor_data['max_pressure']:.2f} mbar")
            print(f"  Average Pressure: {monitor_data['avg_pressure']:.2f} mbar")
        
        response = input("\nContinue with pressure and temperature data demo? (y/n): ").strip().lower()
        if response != 'y':
            pt_interface.disconnect()
            print("Program terminated")
            sys.exit(0)
    
    try:
        print("\n=== VN-200 Pressure and Temperature Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            pressure_data = pt_interface.get_pressure()
            if pressure_data:
                print(f"\nMethod 1 - Pressure Data:")
                print(f"  Barometric Pressure: {pressure_data['pressure']:.2f} mbar (hPa)")
            
            temp_data = pt_interface.get_temperature()
            if temp_data:
                print(f"\nMethod 2 - Temperature Data:")
                print(f"  Temperature: {temp_data['temperature']:.2f} °C ({temp_data['temperature_f']:.2f} °F)")
            
            altitude_data = pt_interface.calculate_altitude(reference_pressure=args.ref_pressure)
            if altitude_data:
                print(f"\nMethod 3 - Barometric Altitude:")
                print(f"  Current Pressure: {altitude_data['pressure']:.2f} mbar")
                print(f"  Reference Pressure: {altitude_data['reference_pressure']:.2f} mbar")
                print(f"  Estimated Altitude: {altitude_data['altitude']:.2f} m ({altitude_data['altitude_ft']:.2f} ft)")
            
            combined_data = pt_interface.get_temp_pressure_combined()
            if combined_data:
                print(f"\nMethod 4 - Combined Temperature and Pressure:")
                print(f"  Temperature: {combined_data['temperature']:.2f} °C ({combined_data['temperature_f']:.2f} °F)")
                print(f"  Pressure: {combined_data['pressure']:.2f} mbar")
                
                if "altitude" in combined_data:
                    print(f"  Estimated Altitude: {combined_data['altitude']:.2f} m ({combined_data['altitude_ft']:.2f} ft)")
            
            async_data = pt_interface.get_pressure_temp_async(timeout=0.5)
            if async_data:
                print(f"\nMethod 5 - Asynchronous Pressure/Temperature Data:")
                
                if "temperature" in async_data:
                    print(f"  Temperature: {async_data['temperature']:.2f} °C ({async_data['temperature_f']:.2f} °F)")
                
                if "pressure" in async_data:
                    print(f"  Pressure: {async_data['pressure']:.2f} mbar")
                    
                    if "altitude" in async_data:
                        print(f"  Estimated Altitude: {async_data['altitude']:.2f} m ({async_data['altitude_ft']:.2f} ft)")
                
                if "timestamp" in async_data:
                    print(f"  Timestamp: {async_data['timestamp']:.3f} s")
            
            print("\n" + "-" * 40)
            time.sleep(2)  
            
    except KeyboardInterrupt:
        pass
    finally:
        pt_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
