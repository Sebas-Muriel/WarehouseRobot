import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    i = 1
    while True:
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        # if line == "1":
        #      ser.write("#".encode('ascii'))
        #      print("here")

        ser.write(b"1/r20/r")