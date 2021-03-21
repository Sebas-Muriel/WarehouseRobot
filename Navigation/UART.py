import pymongo
import sys
import re
import math
import time
import serial
import RPi.GPIO as GPIO


stopTick = bytearray()
stopTick.append(0x02)
leftTick = bytearray()
leftTick.append(0x06)
rightTick = bytearray()
rightTick.append(0x0A)
upTick = bytearray()
upTick.append(0x0E)
downTick = bytearray()
downTick.append(0x12)

stopNode = bytearray()
stopNode.append(0x01)
leftNode = bytearray()
leftNode.append(0x05)
rightNode = bytearray()
rightNode.append(0x09)
upNode = bytearray()
upNode.append(0x0D)
downNode = bytearray()
downNode.append(0x11)

reset = bytearray()
reset.append(0x00)


SRDY = 17 #yellow
MRDY = 27 #orange
sending = False
recieving = False

ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)
read_serial = "0"
count = 0

def main():
    PinSetup()
    startTime = time.time()
    UART_send_repeat(leftNode)
    UART_send_repeat(rightNode)
    while(1):
        read_serial = ser.readline().decode('utf-8', 'ignore').rstrip()
        print(read_serial)

def PinSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MRDY, GPIO.OUT)
    GPIO.setup(SRDY, GPIO.IN)
    GPIO.output(MRDY, GPIO.HIGH)
    return

def UART_send(message):
    GPIO.output(MRDY, GPIO.LOW)
    while(GPIO.input(SRDY) == GPIO.HIGH):
        pass
    ser.write(message)
    GPIO.output(MRDY, GPIO.HIGH)
    while(GPIO.input(SRDY) == GPIO.LOW):
        pass
    return

def UART_send_repeat(message):
    count = 0
    startTime = time.time()
    while(1):
        currentTime = time.time()
        if (currentTime - startTime > .001):
            count = count + 1
            UART_send(message)
            startTime = time.time()
        if count == 3:
            break
    return


if __name__ == '__main__':
    main()