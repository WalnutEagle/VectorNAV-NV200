import time
import sys
import signal
import argparse
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


from vn200_jetson_interface import VN200Interface

class DataVisualizer:
    
    def __init__(self, vn200, history_length=100):

        self.vn200 = vn200
        self.history_length = history_length
        self.timestamps = deque(maxlen=history_length)
        self.yaw = deque(maxlen=history_length)
        self.pitch = deque(maxlen=history_length)
        self.roll = deque(maxlen=history_length)
        self.latitude = deque(maxlen=history_length)
        self.longitude = deque(maxlen=history_length)
        self.altitude = deque(maxlen=history_length)            
        self.vel_north = deque(maxlen=history_length)
        self.vel_east = deque(maxlen=history_length)
        self.vel_down = deque(maxlen=history_length)
        self.speed = deque(maxlen=history_length)        
        self.accel_x = deque(maxlen=history_length)
        self.accel_y = deque(maxlen=history_length)
        self.accel_z = deque(maxlen=history_length)        
        self.gyro_x = deque(maxlen=history_length)
        self.gyro_y = deque(maxlen=history_length)
        self.gyro_z = deque(maxlen=history_length)        
        self.fig = plt.figure(figsize=(15, 10))
        self.fig.canvas.manager.set_window_title('VN-200 Data Visualizer')        
        self.setup_subplots()        
        self.ani = FuncAnimation(self.fig, self.update_plots, interval=50)
    
    def setup_subplots(self):

        self.orientation_plot = plt.subplot(3, 2, 1)
        self.position_plot = plt.subplot(3, 2, 2)
        self.velocity_plot = plt.subplot(3, 2, 3)
        self.speed_plot = plt.subplot(3, 2, 4)
        self.acceleration_plot = plt.subplot(3, 2, 5)
        self.angular_rate_plot = plt.subplot(3, 2, 6)        
        self.orientation_plot.set_title('Orientation')
        self.orientation_plot.set_ylabel('Degrees')
        self.yaw_line, = self.orientation_plot.plot([], [], label='Yaw')
        self.pitch_line, = self.orientation_plot.plot([], [], label='Pitch')
        self.roll_line, = self.orientation_plot.plot([], [], label='Roll')
        self.orientation_plot.legend()
        self.orientation_plot.grid(True)        
        self.position_plot.set_title('Position (GPS)')
        self.position_plot.set_ylabel('Altitude (m)')
        self.position_plot.set_xlabel('Longitude')
        self.position_plot.grid(True)        
        self.velocity_plot.set_title('Velocity')
        self.velocity_plot.set_ylabel('m/s')
        self.vel_north_line, = self.velocity_plot.plot([], [], label='North')
        self.vel_east_line, = self.velocity_plot.plot([], [], label='East')
        self.vel_down_line, = self.velocity_plot.plot([], [], label='Down')
        self.velocity_plot.legend()
        self.velocity_plot.grid(True)        
        self.speed_plot.set_title('Speed')
        self.speed_plot.set_ylabel('m/s')
        self.speed_line, = self.speed_plot.plot([], [], label='Speed')
        self.speed_plot.legend()
        self.speed_plot.grid(True)        
        self.acceleration_plot.set_title('Acceleration')
        self.acceleration_plot.set_ylabel('m/s²')
        self.accel_x_line, = self.acceleration_plot.plot([], [], label='X')
        self.accel_y_line, = self.acceleration_plot.plot([], [], label='Y')
        self.accel_z_line, = self.acceleration_plot.plot([], [], label='Z')
        self.acceleration_plot.legend()
        self.acceleration_plot.grid(True)        
        self.angular_rate_plot.set_title('Angular Rate')
        self.angular_rate_plot.set_ylabel('°/s')
        self.gyro_x_line, = self.angular_rate_plot.plot([], [], label='X')
        self.gyro_y_line, = self.angular_rate_plot.plot([], [], label='Y')
        self.gyro_z_line, = self.angular_rate_plot.plot([], [], label='Z')
        self.angular_rate_plot.legend()
        self.angular_rate_plot.grid(True)        
        plt.tight_layout()
    
    def update_data(self):

        data = self.vn200.get_latest_data()
        if not data:
            return False
        
        if not self.timestamps:
            self.first_timestamp = data.get('timestamp', 0)
            self.timestamps.append(0)
        else:
            self.timestamps.append(data.get('timestamp', 0) - self.first_timestamp)
        
        self.yaw.append(data.get('yaw', 0))
        self.pitch.append(data.get('pitch', 0))
        self.roll.append(data.get('roll', 0))
        
        if 'position' in data:
            pos = data['position']
            self.latitude.append(pos.get('latitude', 0))
            self.longitude.append(pos.get('longitude', 0))
            self.altitude.append(pos.get('altitude', 0))
        else:
            self.latitude.append(self.latitude[-1] if self.latitude else 0)
            self.longitude.append(self.longitude[-1] if self.longitude else 0)
            self.altitude.append(self.altitude[-1] if self.altitude else 0)
        
        if 'velocity' in data:
            vel = data['velocity']
            self.vel_north.append(vel.get('north', 0))
            self.vel_east.append(vel.get('east', 0))
            self.vel_down.append(vel.get('down', 0))
        else:
            self.vel_north.append(self.vel_north[-1] if self.vel_north else 0)
            self.vel_east.append(self.vel_east[-1] if self.vel_east else 0)
            self.vel_down.append(self.vel_down[-1] if self.vel_down else 0)
        
        self.speed.append(data.get('speed', 0))
        
        if 'acceleration' in data:
            self.accel_x.append(data['acceleration'][0])
            self.accel_y.append(data['acceleration'][1])
            self.accel_z.append(data['acceleration'][2])
        else:
            self.accel_x.append(self.accel_x[-1] if self.accel_x else 0)
            self.accel_y.append(self.accel_y[-1] if self.accel_y else 0)
            self.accel_z.append(self.accel_z[-1] if self.accel_z else 0)
        
        if 'angular_rate' in data:
            self.gyro_x.append(data['angular_rate'][0])
            self.gyro_y.append(data['angular_rate'][1])
            self.gyro_z.append(data['angular_rate'][2])
        else:
            self.gyro_x.append(self.gyro_x[-1] if self.gyro_x else 0)
            self.gyro_y.append(self.gyro_y[-1] if self.gyro_y else 0)
            self.gyro_z.append(self.gyro_z[-1] if self.gyro_z else 0)
        
        return True
    
    def update_plots(self, frame):

        if not self.update_data():
            return
        
        times = list(self.timestamps)
        
        self.yaw_line.set_data(times, self.yaw)
        self.pitch_line.set_data(times, self.pitch)
        self.roll_line.set_data(times, self.roll)
        self.orientation_plot.relim()
        self.orientation_plot.autoscale_view()
        
        self.position_plot.clear()
        self.position_plot.set_title('Position (GPS)')
        self.position_plot.set_ylabel('Latitude')
        self.position_plot.set_xlabel('Longitude')
        if self.longitude and self.latitude:
            self.position_plot.scatter(
                list(self.longitude), 
                list(self.latitude),
                c=list(self.altitude), 
                cmap='viridis'
            )
            self.position_plot.grid(True)
        
        self.vel_north_line.set_data(times, self.vel_north)
        self.vel_east_line.set_data(times, self.vel_east)
        self.vel_down_line.set_data(times, self.vel_down)
        self.velocity_plot.relim()
        self.velocity_plot.autoscale_view()
        
        self.speed_line.set_data(times, self.speed)
        self.speed_plot.relim()
        self.speed_plot.autoscale_view()
        
        self.accel_x_line.set_data(times, self.accel_x)
        self.accel_y_line.set_data(times, self.accel_y)
        self.accel_z_line.set_data(times, self.accel_z)
        self.acceleration_plot.relim()
        self.acceleration_plot.autoscale_view()
        
        self.gyro_x_line.set_data(times, self.gyro_x)
        self.gyro_y_line.set_data(times, self.gyro_y)
        self.gyro_z_line.set_data(times, self.gyro_z)
        self.angular_rate_plot.relim()
        self.angular_rate_plot.autoscale_view()
        
        self.fig.suptitle(f'VN-200 Data - Latest Yaw: {self.yaw[-1]:.2f}° | ' +
                         f'Speed: {self.speed[-1]:.2f} m/s | Alt: {self.altitude[-1]:.2f}m',
                         fontsize=14)
        
        return (self.yaw_line, self.pitch_line, self.roll_line,
                self.vel_north_line, self.vel_east_line, self.vel_down_line,
                self.speed_line, self.accel_x_line, self.accel_y_line, self.accel_z_line,
                self.gyro_x_line, self.gyro_y_line, self.gyro_z_line)
    
    def run(self):
        plt.show()


def main():

    parser = argparse.ArgumentParser(description='VN-200 Data Visualizer')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port for VN-200 connection')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate for serial connection')
    parser.add_argument('--rate', type=int, default=20, help='Update rate in Hz')
    args = parser.parse_args()
    
    vn200 = VN200Interface(port=args.port, baud_rate=args.baud)
    
    def signal_handler(sig, frame):
        print("\nExiting...")
        vn200.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    if not vn200.connect():
        print("Failed to connect to VN-200. Exiting.")
        sys.exit(1)
    
    if not vn200.configure_outputs(update_rate_hz=args.rate):
        print("Failed to configure VN-200. Exiting.")
        vn200.disconnect()
        sys.exit(1)
    
    vn200.start_data_collection()
    
    try:
        visualizer = DataVisualizer(vn200)
        visualizer.run()
    except KeyboardInterrupt:
        pass
    finally:
        vn200.disconnect()
        print("Program terminated")

if __name__ == "__main__":
    main()
