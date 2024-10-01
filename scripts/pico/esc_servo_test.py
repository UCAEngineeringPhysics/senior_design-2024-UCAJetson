from machine import Pin, PWM
from time import sleep

# SETUP
servo = PWM(Pin(16))
servo.freq(50)

# LOOP
try:
    print("Turning Right")
    for i in range(15000, 20000, 100): 
        servo.duty_ns(i)
        print(i)
        sleep(0.2)
    print("Turning Left")
    for i in reversed(range(1500000, 2000000, 10000)): 
        servo.duty_ns(i)
        print(i)
        sleep(0.2)
except:
    print("Exception")
finally:
    servo.duty_ns(0)
    servo.deinit()

