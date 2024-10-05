import serial
import pygame
from gpiozero import LED
# from picamera2 import Picamera2
import cv2 as cv

def setup_camera(size=(120, 160), frame_rate=20):
    cam = cv.VideoCapture(0)  # '0' refers to the default camera, adjust if needed
    cam.set(cv.CAP_PROP_FRAME_WIDTH, size[1])
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, size[0])
    cam.set(cv.CAP_PROP_FPS, frame_rate)
    return cam


def setup_serial(port, baudrate=115200):
    ser = serial.Serial(port=port, baudrate=baudrate)
    print(f"Serial connected on {ser.name}")
    return ser

def setup_joystick():
    pygame.joystick.init()
    js = pygame.joystick.Joystick(0)
    js.init()
    return js

def setup_led(pin):
    led = LED(pin)
    led.off()
    return led

def encode_steering(steering_value, params):
    return params['steering_center'] - params['steering_range'] + int(params['steering_range'] * (steering_value + 1))

def encode_throttle(throttle_value, params):
    throttle_limit = min(max(throttle_value, -1), 1)
    if throttle_limit > 0:
        return params['throttle_stall'] + int(params['throttle_fwd_range'] * throttle_limit)
    elif throttle_limit < 0:
        return params['throttle_stall'] + int(params['throttle_rev_range'] * throttle_limit)
    else:
        return params['throttle_stall']
