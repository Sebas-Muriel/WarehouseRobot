import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    i = 1
    packet = bytearray()
    packet.append(0x07)
    while True:
        
        # if line == "1":
        #      ser.write("#".encode('ascii'))
        #      print("here")

        ser.write(packet)
        #ser.write("0".encode('ascii'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)