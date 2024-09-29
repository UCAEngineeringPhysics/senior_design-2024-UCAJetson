from machine import Pin, PWM, reset
from time import sleep

# SETUP
motor = PWM(Pin(15))
motor.freq(10000)
sleep(3)

# LOOP
max = 65535
try:
    for i in range(0,max,1500):
        motor.duty_u16(i)
        print("Waiting 1 second...")
        sleep(0.2)
    # for i in range(1210000, 1800000, 5000): # forward up
    #     motor.duty_ns(i)
    #     print(i)
    #     sleep(0.2)
    # for i in reversed(range(1210000, 1800000, 5000)): # forward down
    #     motor.duty_ns(i)
    #     print(i)
    #     sleep(0.2)
    # for i in reversed(range(1090000, 1210000, 10000)): # reverse up
    #     motor.duty_ns(i)
    #     print(i)
    #     sleep(0.5)
    # for i in range(1090000, 1210000, 10000): # reverse down
    #     motor.duty_ns(i)
    #     print(i)
    #     sleep(0.5)
except:
    print("Exception")
    pass
finally:
    motor.duty_ns(0)
    sleep(1)
    motor.deinit()
    # reset()
    


