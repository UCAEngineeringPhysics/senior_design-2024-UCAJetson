import os
import sys
import json
import torch
from models import MultiModalNet
from hardware import setup_camera, setup_serial, setup_joystick, setup_led, encode_steering, encode_throttle
from torchvision import transforms
import pygame
import cv2 as cv


# SETUP
# Load model
model_path = os.path.join('models', 'MultiModalNet.pth')
model = MultiModalNet()
model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
model.eval()

# Load configs
params_file_path = os.path.join(sys.path[0], 'configs.json')
with open(params_file_path) as params_file:
    params = json.load(params_file)

# Initialize hardware
headlight = setup_led(params['led_pin'])
ser_pico = setup_serial(port='/dev/ttyACM0', baudrate=115200)
cam = setup_camera((120, 160), frame_rate=20)
js = setup_joystick()

to_tensor = transforms.ToTensor()
is_paused = True
frame_counts = 0

# MAIN LOOP
try:
    while True:
        ret, frame = cam.read()
        if frame is None:
            print("No frame received. TERMINATE!")
            break

        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN:
                if js.get_button(params['pause_btn']):
                    is_paused = not is_paused
                    headlight.toggle()
                elif js.get_button(params['stop_btn']):
                    print("E-STOP PRESSED. TERMINATE!")
                    break

        # Predict steering and throttle
        img_tensor = to_tensor(frame).unsqueeze(0)  # Add batch dimension
        pred_st, pred_th = model(img_tensor).squeeze()

        # Encode and send commands
        if not is_paused:
            duty_st = int(encode_steering(pred_st.item(), params))
            duty_th = int(encode_throttle(pred_th.item(), params))
        else:
            duty_st, duty_th = params['steering_center'], params['throttle_stall']

        ser_pico.write(f"{duty_st},{duty_th}\n".encode('utf-8'))
        frame_counts += 1

except KeyboardInterrupt:
    print("Terminated by user.")
finally:
    cam.stop()
    pygame.quit()
    ser_pico.close()
    headlight.off()
    cv.destroyAllWindows()
