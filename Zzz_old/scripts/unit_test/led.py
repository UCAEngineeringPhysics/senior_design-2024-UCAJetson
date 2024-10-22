import sys
import os
import json
from time import sleep
from gpiozero import LED

# Load configs
params_file_path = os.path.join(os.path.dirname(sys.path[0]), 'configs.json')
params_file = open(params_file_path)
params = json.load(params_file)

# SETUP
led = LED(params['led_pin'])

for _ in range(10):
    led.on()
    sleep(1)
    led.off()
    sleep(1)
