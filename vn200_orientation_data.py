import time
import sys
import argparse
import signal
import math
import numpy as np
from vectornav import Sensor, Registers, VnError

class VN200OrientationInterface:
    
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
    
    def configure_orientation_output(self, update_rate_hz=20):
        try:
            rate_divisor = int(800 / update_rate_hz)
            
            binary_output = Registers.BinaryOutput1()
            
            binary_output.rateDivisor = rate_divisor
            
            binary_output.asyncMode.serial1 = 1
            
            binary_output.common.ypr = 1             # Yaw, Pitch, Roll
            binary_output.common.quaternion = 1      # Quaternion
            binary_output.common.dcm = 1             # Direction Cosine Matrix
            binary_output.common.magNed = 1          # Magnetic NED
            binary_output.common.accelNed = 1        # Acceleration NED
            
            self.sensor.writeRegister(binary_output)
            print(f"Orientation output configured at {update_rate_hz}Hz")
            
            asyncDataOutputType = Registers.AsyncOutputType()
            asyncDataOutputType.ador = Registers.AsyncOutputType.Ador.YPR
            asyncDataOutputType.serialPort = Registers.AsyncOutputType.SerialPort.Serial1
            self.sensor.writeRegister(asyncDataOutputType)
            
            asyncDataOutputFreq = Registers.AsyncOutputFreq()
            asyncDataOutputFreq.adof = Registers.AsyncOutputFreq.Adof.Rate20Hz
            asyncDataOutputFreq.serialPort = Registers.AsyncOutputFreq.SerialPort.Serial1
            self.sensor.writeRegister(asyncDataOutputFreq)
            
            return True
        except VnError as e:
            print(f"Error configuring orientation output: {e}")
            return False
    
    def get_yaw_pitch_roll(self):
        try:
            ypr_register = Registers.YawPitchRoll()
            self.sensor.readRegister(ypr_register)
            
            yaw = ypr_register.yaw
            pitch = ypr_register.pitch
            roll = ypr_register.roll
            
            return {
                "yaw": yaw,    # Yaw (heading) in degrees
                "pitch": pitch,  # Pitch in degrees
                "roll": roll,   # Roll in degrees
                "method": "ypr_register"
            }
        except VnError as e:
            print(f"Error reading YPR register: {e}")
            return None
    
    def get_quaternion(self):
        try:
            quat_register = Registers.QuaternionMag()
            self.sensor.readRegister(quat_register)
            
            qx = quat_register.quat.x
            qy = quat_register.quat.y
            qz = quat_register.quat.z
            qw = quat_register.quat.w
            
            return {
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
                "method": "quaternion_register"
            }
        except VnError as e:
            print(f"Error reading quaternion register: {e}")
            return None
    
    def get_direction_cosine_matrix(self):
        try:
            dcm_register = Registers.VpeControl()
            self.sensor.readRegister(dcm_register)
            
            quat = self.get_quaternion()
            if not quat:
                return None
                
            qx, qy, qz, qw = quat["qx"], quat["qy"], quat["qz"], quat["qw"]
            
            dcm = np.zeros((3, 3))
            
            dcm[0, 0] = 1 - 2 * (qy**2 + qz**2)
            dcm[0, 1] = 2 * (qx * qy - qw * qz)
            dcm[0, 2] = 2 * (qx * qz + qw * qy)
            
            dcm[1, 0] = 2 * (qx * qy + qw * qz)
            dcm[1, 1] = 1 - 2 * (qx**2 + qz**2)
            dcm[1, 2] = 2 * (qy * qz - qw * qx)
            
            dcm[2, 0] = 2 * (qx * qz - qw * qy)
            dcm[2, 1] = 2 * (qy * qz + qw * qx)
            dcm[2, 2] = 1 - 2 * (qx**2 + qy**2)
            
            return {
                "dcm": dcm.tolist(),
                "method": "dcm_conversion"
            }
        except VnError as e:
            print(f"Error getting DCM: {e}")
            return None
        except Exception as e:
            print(f"Error in DCM calculation: {e}")
            return None
    
    def get_orientation_async(self, timeout=1.0):
        try:
            start_time = time.time()
            while time.time() - start_time < timeout:
                cd = self.sensor.getNextMeasurement()
                if not cd:
                    time.sleep(0.001)  
                    continue
                
                if cd.attitude.ypr:
                    result = {
                        "method": "async_measurement"
                    }
                    
                    if cd.attitude.ypr:
                        result["yaw"] = cd.attitude.ypr.x
                        result["pitch"] = cd.attitude.ypr.y
                        result["roll"] = cd.attitude.ypr.z
                    
                    if cd.attitude.quaternion:
                        result["qx"] = cd.attitude.quaternion.x
                        result["qy"] = cd.attitude.quaternion.y
                        result["qz"] = cd.attitude.quaternion.z
                        result["qw"] = cd.attitude.quaternion.w
                    
                    return result
            
            print("Timeout waiting for orientation data")
            return None
        except VnError as e:
            print(f"Error getting async orientation: {e}")
            return None
    
    def get_attitude_uncertainty(self):
        try:
            att_register = Registers.AttitudeStdev()
            self.sensor.readRegister(att_register)
            
            return {
                "yaw_uncertainty": att_register.yawPitchRollStdev.x,
                "pitch_uncertainty": att_register.yawPitchRollStdev.y,
                "roll_uncertainty": att_register.yawPitchRollStdev.z,
                "method": "attitude_uncertainty"
            }
        except VnError as e:
            print(f"Error reading attitude uncertainty: {e}")
            return None
    
    @staticmethod
    def quaternion_to_euler(qx, qy, qz, qw):
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi
        
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(90.0, sinp)
        else:
            pitch = math.asin(sinp) * 180.0 / math.pi
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi
        
        return yaw, pitch, roll
    
    @staticmethod
    def euler_to_quaternion(yaw, pitch, roll):
        yaw = math.radians(yaw)
        pitch = math.radians(pitch)
        roll = math.radians(roll)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        
        return qx, qy, qz, qw
    
    def disconnect(self):
        if self.connected:
            try:
                self.sensor.disconnect()
                print("Disconnected from VN-200")
            except VnError as e:
                print(f"Error disconnecting: {e}")
            
            self.connected = False


def main():

    parser = argparse.ArgumentParser(description='VN-200 Orientation Data Interface')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    args = parser.parse_args()
    
    orientation_interface = VN200OrientationInterface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        orientation_interface.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not orientation_interface.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not orientation_interface.configure_orientation_output(update_rate_hz=args.rate):
        print("Failed to configure VN-200 outputs. Exiting.")
        orientation_interface.disconnect()
        sys.exit(1)
    
    try:
        print("\n=== VN-200 Orientation Data Demo ===")
        print("Press Ctrl+C to exit\n")
        
        while True:
            ypr_data = orientation_interface.get_yaw_pitch_roll()
            if ypr_data:
                print(f"\nMethod 1 - Yaw, Pitch, Roll (Euler angles):")
                print(f"  Yaw (Heading): {ypr_data['yaw']:.2f}°")
                print(f"  Pitch:         {ypr_data['pitch']:.2f}°")
                print(f"  Roll:          {ypr_data['roll']:.2f}°")
            
            quat_data = orientation_interface.get_quaternion()
            if quat_data:
                print(f"\nMethod 2 - Quaternion:")
                print(f"  qx: {quat_data['qx']:.6f}")
                print(f"  qy: {quat_data['qy']:.6f}")
                print(f"  qz: {quat_data['qz']:.6f}")
                print(f"  qw: {quat_data['qw']:.6f}")
                
                yaw, pitch, roll = orientation_interface.quaternion_to_euler(
                    quat_data['qx'], quat_data['qy'], quat_data['qz'], quat_data['qw'])
                print(f"  Converted to YPR: {yaw:.2f}°, {pitch:.2f}°, {roll:.2f}°")
            
            dcm_data = orientation_interface.get_direction_cosine_matrix()
            if dcm_data:
                print(f"\nMethod 3 - Direction Cosine Matrix (DCM):")
                for i in range(3):
                    print(f"  [ {dcm_data['dcm'][i][0]:.6f}, {dcm_data['dcm'][i][1]:.6f}, {dcm_data['dcm'][i][2]:.6f} ]")
            
            async_data = orientation_interface.get_orientation_async(timeout=0.5)
            if async_data and 'yaw' in async_data:
                print(f"\nMethod 4 - Asynchronous Orientation:")
                print(f"  Yaw (Heading): {async_data['yaw']:.2f}°")
                print(f"  Pitch:         {async_data['pitch']:.2f}°")
                print(f"  Roll:          {async_data['roll']:.2f}°")
            
            uncertainty = orientation_interface.get_attitude_uncertainty()
            if uncertainty:
                print(f"\nMethod 5 - Attitude Uncertainty:")
                print(f"  Yaw Uncertainty:   {uncertainty['yaw_uncertainty']:.4f}°")
                print(f"  Pitch Uncertainty: {uncertainty['pitch_uncertainty']:.4f}°")
                print(f"  Roll Uncertainty:  {uncertainty['roll_uncertainty']:.4f}°")
            
            print("\n" + "-" * 40)
            time.sleep(2)  
            
    except KeyboardInterrupt:
        pass
    finally:
        orientation_interface.disconnect()
        print("Program terminated")


if __name__ == "__main__":
    main()
