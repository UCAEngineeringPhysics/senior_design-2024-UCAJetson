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
