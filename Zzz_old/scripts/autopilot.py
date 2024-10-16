import sys
import os
import json
from time import time
import torch
from torchvision import transforms
import convnets
import serial
import pygame
import cv2 as cv
from picamera2 import Picamera2
from gpiozero import LED


# SETUP
# Load configs and init servo controller
model_path = os.path.join(
    os.path.dirname(sys.path[0]),
    'models', 
    'DonkeyNet-15epochs-0.001lr.pth'
)
to_tensor = transforms.ToTensor()
model = convnets.DonkeyNet()
model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
model.eval()
# Load configs
params_file_path = os.path.join(sys.path[0], 'configs.json')
params_file = open(params_file_path)
params = json.load(params_file)
# Constants
STEERING_CENTER = params['steering_center']
STEERING_RANGE = params['steering_range']
THROTTLE_STALL = params['throttle_stall']
THROTTLE_FWD_RANGE = params['throttle_fwd_range']
THROTTLE_REV_RANGE = params['throttle_rev_range']
THROTTLE_LIMIT = params['throttle_limit']
PAUSE_BUTTON = params['record_btn']
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
# init camera
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
is_paused = True


# LOOP
try:
    while True:
        frame = cam.capture_array()  # read image
        if frame is None:
            print("No frame received. TERMINATE!")
            headlight.close()
            cv.destroyAllWindows()
            pygame.quit()
            ser_pico.close()
            sys.exit()
        for e in pygame.event.get():  # read controller input
            if e.type == pygame.JOYBUTTONDOWN:
                if js.get_button(PAUSE_BUTTON):
                    is_paused = not is_paused
                    print(f"Paused: {is_paused}")
                    headlight.toggle()
                elif js.get_button(STOP_BUTTON):  # emergency stop 
                    print("E-STOP PRESSED. TERMINATE!")
                    headlight.off()
                    headlight.close()
                    cv.destroyAllWindows()
                    pygame.quit()
                    sys.exit()
        # predict steer and throttle
        img_tensor = to_tensor(frame)
        pred_st, pred_th = model(img_tensor[None, :]).squeeze()
        st_trim = float(pred_st)
        if st_trim >= 1:  # trim steering signal
            st_trim = .999
        elif st_trim <= -1:
            st_trim = -.999
        th_trim = (float(pred_th))
        if th_trim >= 1:  # trim throttle signal
            th_trim = .999
        elif th_trim <= -1:
            th_trim = -.999
        # Encode steering value to dutycycle in nanosecond
        if is_paused:
            duty_st = STEERING_CENTER
        else:
            duty_st = STEERING_CENTER - STEERING_RANGE + int(STEERING_RANGE * (st_trim + 1))
        # Encode throttle value to dutycycle in nanosecond
        if is_paused:
            duty_th = THROTTLE_STALL
        else:
            if th_trim > 0:
                duty_th = THROTTLE_STALL + int(THROTTLE_FWD_RANGE * min(th_trim, THROTTLE_LIMIT))
            elif th_trim < 0:
                duty_th = THROTTLE_STALL + int(THROTTLE_REV_RANGE * max(th_trim, -THROTTLE_LIMIT))
            else:
                duty_th = THROTTLE_STALL
        msg = (str(duty_st) + "," + str(duty_th) + "\n").encode('utf-8')
        # Transmit control signals
        ser_pico.write(msg)
        print(f"predicted action: {pred_st, pred_th}")        
        frame_counts += 1
        # Log frame rate
        since_start = time() - start_stamp
        frame_rate = frame_counts / since_start
        print(f"frame rate: {frame_rate}")
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
