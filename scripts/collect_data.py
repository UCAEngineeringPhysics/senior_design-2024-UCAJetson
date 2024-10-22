import os
import sys
import json
import csv
from time import time
from datetime import datetime
from hardware import setup_camera, setup_serial, setup_joystick, setup_led, encode_steering, encode_throttle
import pygame
import cv2 as cv


# SETUP
# Load configs
params_file_path = os.path.join(sys.path[0], 'configs.json')
with open(params_file_path) as params_file:
    params = json.load(params_file)

# Constants
STEERING_AXIS = params['steering_joy_axis']
THROTTLE_AXIS = params['throttle_joy_axis']
RECORD_BUTTON = params['record_btn']
STOP_BUTTON = params['stop_btn']

# Initialize hardware
headlight = setup_led(params['led_pin'])
ser_pico = setup_serial(port='/dev/ttyACM0', baudrate=115200)
cam = setup_camera((120, 160), frame_rate=20)
js = setup_joystick()

# Create data directories
data_dir = os.path.join('data', datetime.now().strftime("%Y-%m-%d-%H-%M"))
image_dir = os.path.join(data_dir, 'images/')
label_path = os.path.join(data_dir, 'labels.csv')
os.makedirs(image_dir, exist_ok=True)

# Initialize variables
is_recording = False
frame_counts = 0
start_time = time()

# MAIN LOOP
try:
    while True:
        ret, frame = cam.read()
        if frame is None:
            print("No frame received. TERMINATE!")
            break

        # Controller input
        for e in pygame.event.get():
            if e.type == pygame.JOYAXISMOTION:
                ax_val_st = round(js.get_axis(STEERING_AXIS), 2)
                ax_val_th = round(js.get_axis(THROTTLE_AXIS), 2)
            elif e.type == pygame.JOYBUTTONDOWN:
                if js.get_button(RECORD_BUTTON):
                    is_recording = not is_recording
                    headlight.toggle()
                elif js.get_button(STOP_BUTTON):
                    print("E-STOP PRESSED. TERMINATE!")
                    break

        # Encode and transmit control signals
        duty_st = int(encode_steering(ax_val_st, params))
        duty_th = int(encode_throttle(ax_val_th, params))
        ser_pico.write(f"{duty_st},{duty_th}\n".encode('utf-8'))

        # Log data
        if is_recording:
            cv.imwrite(os.path.join(image_dir, f"{frame_counts}.jpg"), frame)
            with open(label_path, 'a+', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([f"{frame_counts}.jpg", ax_val_st, ax_val_th])

        frame_counts += 1

except KeyboardInterrupt:
    print("Terminated by user.")
finally:
    cam.stop()
    pygame.quit()
    ser_pico.close()
    headlight.off()
    cv.destroyAllWindows()
    
"""
# ROS2 RealSense and RP LiDAR Integration 
import os
import sys
import json
import csv
from time import time
from datetime import datetime
from hardware import setup_serial, setup_joystick, setup_led, encode_steering, encode_throttle
import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

# Load configs
params_file_path = os.path.join(sys.path[0], 'configs.json')
with open(params_file_path) as params_file:
    params = json.load(params_file)

# Constants
STEERING_AXIS = params['steering_joy_axis']
THROTTLE_AXIS = params['throttle_joy_axis']
RECORD_BUTTON = params['record_btn']
STOP_BUTTON = params['stop_btn']

# Initialize hardware
headlight = setup_led(params['led_pin'])
ser_pico = setup_serial(port='/dev/ttyACM0', baudrate=115200)
js = setup_joystick()

# Create data directories
data_dir = os.path.join('data', datetime.now().strftime("%Y-%m-%d-%H-%M"))
image_dir = os.path.join(data_dir, 'images/')
label_path = os.path.join(data_dir, 'labels.csv')
os.makedirs(image_dir, exist_ok=True)

# Initialize variables
is_recording = False
frame_counts = 0

# ROS2 Node for RealSense and LiDAR
class RealSenseLidarSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_lidar_subscriber')
        self.bridge = CvBridge()

        # Subscribe to color and depth topics
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        # Subscribe to LiDAR topic
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',  # Topic for RP LiDAR (can vary depending on setup)
            self.lidar_callback,
            10)

        # Initialize frames and LiDAR data
        self.color_frame = None
        self.depth_frame = None
        self.lidar_data = None

    def color_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def lidar_callback(self, msg):
        # Store LiDAR data for later use (ranges is an array of distances)
        self.lidar_data = msg.ranges

def main(args=None):
    rclpy.init(args=args)

    # Initialize ROS2 RealSense + LiDAR node
    realsense_lidar_subscriber = RealSenseLidarSubscriber()

    # Initialize Pygame for joystick handling
    pygame.init()

    global is_recording
    global frame_counts

    try:
        while rclpy.ok():
            # Keep processing ROS2 topics (color, depth, LiDAR)
            rclpy.spin_once(realsense_lidar_subscriber, timeout_sec=0.01)

            # Get color, depth, and LiDAR data from RealSense and LiDAR
            color_image = realsense_lidar_subscriber.color_frame
            depth_image = realsense_lidar_subscriber.depth_frame
            lidar_data = realsense_lidar_subscriber.lidar_data

            # Controller input handling
            for e in pygame.event.get():
                if e.type == pygame.JOYAXISMOTION:
                    ax_val_st = round(js.get_axis(STEERING_AXIS), 2)
                    ax_val_th = round(js.get_axis(THROTTLE_AXIS), 2)
                elif e.type == pygame.JOYBUTTONDOWN:
                    if js.get_button(RECORD_BUTTON):
                        is_recording = not is_recording
                        headlight.toggle()
                    elif js.get_button(STOP_BUTTON):
                        print("E-STOP PRESSED. TERMINATE!")
                        break

            # Encode and transmit control signals
            duty_st = int(encode_steering(ax_val_st, params))
            duty_th = int(encode_throttle(ax_val_th, params))
            ser_pico.write(f"{duty_st},{duty_th}\n".encode('utf-8'))

            # Save color, depth, LiDAR, and joystick data if recording
            if is_recording and color_image is not None and depth_image is not None and lidar_data is not None:
                # Save images
                cv.imwrite(os.path.join(image_dir, f"{frame_counts}_color.jpg"), color_image)
                cv.imwrite(os.path.join(image_dir, f"{frame_counts}_depth.png"), depth_image)

                # Log joystick and LiDAR data
                with open(label_path, 'a+', newline='') as f:
                    writer = csv.writer(f)
                    lidar_summary = f"{min(lidar_data):.2f}-{max(lidar_data):.2f}"  # Min and max range as example
                    writer.writerow([f"{frame_counts}_color.jpg", f"{frame_counts}_depth.png", lidar_summary, ax_val_st, ax_val_th])

                frame_counts += 1

    except KeyboardInterrupt:
        print("Terminated by user.")
    finally:
        # Cleanup
        rclpy.shutdown()
        pygame.quit()
        ser_pico.close()
        headlight.off()
        cv.destroyAllWindows()

if __name__ == '__main__':
    main()

    """
