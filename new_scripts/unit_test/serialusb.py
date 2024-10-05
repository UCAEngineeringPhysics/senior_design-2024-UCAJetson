import serial
from time import sleep

# SETUP
ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200)
print(ser.name)

# LOOP
for i in range(10):
    msg = f"Hello from RPi: {i}\n".encode('utf-8')
    ser.write(msg)
    # print("Transmitting")
    sleep(1)
    if ser.inWaiting() > 0:
        pico_data = ser.readline()
        pico_data = pico_data.decode('utf-8', 'ignore')
        print(pico_data)
ser.close()
