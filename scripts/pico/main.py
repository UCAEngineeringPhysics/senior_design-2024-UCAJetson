"""
Upload this script to the pico board, then rename it to main.py.
Read dutycycle in nanoseconds via USB BUS.
"""
import sys
import select
from time import sleep
from machine import Pin, PWM

# SETUP
steering = PWM(Pin(16))
steering.freq(50)
throttle = PWM(Pin(15))
throttle.freq(50)
sleep(3)  # ESC calibrate
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)
event = poller.poll()

# LOOP
try:
    while True:
        # read data from serial
        for msg, _ in event:
            buffer = msg.readline().rstrip().split(',')
            if len(buffer) == 2:
                ns_st, ns_th = int(buffer[0]), int(buffer[1])
                print(f"steering: {ns_st}\nthrottle: {ns_th}")
                steering.duty_ns(ns_st)
                throttle.duty_ns(ns_th)
except:
    pass
finally:
    throttle.duty_ns(1210000)
    sleep(1)
    throttle.deinit()
    steering.duty_ns(1500000)
    sleep(1)
    steering.deinit()
