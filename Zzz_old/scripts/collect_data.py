import sys
import os
import json
from time import time
from datetime import datetime
import csv
import serial
import pygame
import cv2 as cv
from picamera2 import Picamera2
from gpiozero import LED


# SETUP
# Load configs
params_file_path = os.path.join(sys.path[0], 'configs.json')
params_file = open(params_file_path)
params = json.load(params_file)
# Constants
STEERING_AXIS = params['steering_joy_axis']
STEERING_CENTER = params['steering_center']
STEERING_RANGE = params['steering_range']
THROTTLE_AXIS = params['throttle_joy_axis']
THROTTLE_STALL = params['throttle_stall']
THROTTLE_FWD_RANGE = params['throttle_fwd_range']
THROTTLE_REV_RANGE = params['throttle_rev_range']
THROTTLE_LIMIT = params['throttle_limit']
RECORD_BUTTON = params['record_btn']
STOP_BUTTON = params['stop_btn']
# Init LED
headlight = LED(params['led_pin'])
headlight.off()
# Init serial port
ser_pico = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
print(f"Pico is connected to port: {ser_pico.name}")
# Init controller
pygame.display.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
# Create data directory
image_dir = os.path.join(
    os.path.dirname(sys.path[0]),
    'data', datetime.now().strftime("%Y-%m-%d-%H-%M"),
    'images/'
)
if not os.path.exists(image_dir):
    try:
        os.makedirs(image_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
label_path = os.path.join(os.path.dirname(os.path.dirname(image_dir)), 'labels.csv')
# Init camera
cv.startWindowThread()
cam = Picamera2()
cam.configure(
    cam.create_preview_configuration(
        main={"format": 'RGB888', "size": (120, 160)},
        controls={"FrameDurationLimits": (50000, 50000)},  # 20 FPS
    )
)
cam.start()
for i in reversed(range(60)):
    frame = cam.capture_array()
    # cv.imshow("Camera", frame)
    # cv.waitKey(1)
    if frame is None:
        print("No frame received. TERMINATE!")
        sys.exit()
    if not i % 20:
        print(i/20)  # count down 3, 2, 1 sec
# Init timer for FPS computing
start_stamp = time()
frame_counts = 0
ave_frame_rate = 0.
# Init variables
ax_val_st = 0. # center steering
ax_val_th = 0. # shut throttle
is_recording = False

# LOOP
try:
    while True:
        frame = cam.capture_array() # read image
        if frame is None:
            print("No frame received. TERMINATE!")
            headlight.close()
            cv.destroyAllWindows()
            pygame.quit()
            ser_pico.close()
            sys.exit()
        for e in pygame.event.get(): # read controller input
            if e.type == pygame.JOYAXISMOTION:
                ax_val_st = round((js.get_axis(STEERING_AXIS)), 2)  # keep 2 decimals
                ax_val_th = round((js.get_axis(THROTTLE_AXIS)), 2)  # keep 2 decimals
            elif e.type == pygame.JOYBUTTONDOWN:
                if js.get_button(RECORD_BUTTON):
                    is_recording = not is_recording
                    print(f"Recording: {is_recording}")
                    headlight.toggle()
                elif js.get_button(STOP_BUTTON): # emergency stop
                    print("E-STOP PRESSED. TERMINATE!")
                    headlight.off()
                    headlight.close()
                    cv.destroyAllWindows()
                    pygame.quit()
                    ser_pico.close()
                    sys.exit()
        # Calaculate steering and throttle value
        act_st = ax_val_st  # steer action: -1: left, 1: right
        act_th = -ax_val_th  # throttle action: -1: max forward, 1: max backward
        # Encode steering value to dutycycle in nanosecond
        duty_st = STEERING_CENTER - STEERING_RANGE + int(STEERING_RANGE * (act_st + 1))
        # Encode throttle value to dutycycle in nanosecond
        if act_th > 0:
            duty_th = THROTTLE_STALL + int(THROTTLE_FWD_RANGE * min(act_th, THROTTLE_LIMIT))
        elif act_th < 0:
            duty_th = THROTTLE_STALL + int(THROTTLE_REV_RANGE * max(act_th, -THROTTLE_LIMIT))
        else:
            duty_th = THROTTLE_STALL 
        msg = (str(duty_st) + "," + str(duty_th) + "\n").encode('utf-8')
        # Transmit control signals
        ser_pico.write(msg)
        # Log data
        action = [act_st, act_th]
        # print(f"action: {action}")
        if is_recording:
            # img = cv.resize(frame, (120, 160))
            cv.imwrite(image_dir + str(frame_counts) + '.jpg', frame)
            label = [str(frame_counts) + '.jpg'] + action
            with open(label_path, 'a+', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(label)
        frame_counts += 1
        # Log frame rate
        since_start = time() - start_stamp
        frame_rate = frame_counts / since_start
        print(f"frame rate: {frame_rate}")
        # Press "q" to quit
        if cv.waitKey(1)==ord('q'):
            headlight.off()
            headlight.close()
            cv.destroyAllWindows()
            pygame.quit()
            ser_pico.close()
            sys.exit()

# Take care terminate signal (Ctrl-c)
except KeyboardInterrupt:
    headlight.off()
    headlight.close()
    cv.destroyAllWindows()
    pygame.quit()
    ser_pico.close()
    sys.exit()
