import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)