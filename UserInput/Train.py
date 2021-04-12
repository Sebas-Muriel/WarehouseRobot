#test
# need these installed on py: sudo apt-get install python-serial
#if i2c-1 not giving permissions run this command:
#sudo chmod a+rw /dev/i2c-*
#export DISPLAY=:0
#Must plug in display first then turn on Pi to get camera to work
import pymongo
import sys
import re
import math
import time
import serial
import json
import cv2
import RPi.GPIO as GPIO
import adafruit_servokit
import cv2 as cv
import numpy as np
from pyzbar.pyzbar import decode


class ServoKit(object):
    default_angle = 90

    def __init__(self, num_ports):
        print("Initializing the servo...")
        self.kit = adafruit_servokit.ServoKit(channels=16)
        self.num_ports = num_ports
        self.resetAll()
        print("Initializing complete.")

    def setAngle(self, port, angle):
        if angle < 0:
            self.kit.servo[port].angle = 0
        elif angle > 180:
            self.kit.servo[port].angle = 180
        else:
            self.kit.servo[port].angle = angle
    
    def getAngle(self, port):
        return self.kit.servo[port].angle

    def reset(self, port):
        self.kit.servo[port].angle = self.default_angle

    def resetAll(self):
        for i in range(self.num_ports):
            self.kit.servo[i].angle = self.default_angle


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

stopMotor = bytearray()
stopMotor.append(0x00)
level1Motor = bytearray()
level1Motor.append(0x20)
level2Motor = bytearray()
level2Motor.append(0x40)
level3Motor = bytearray()
level3Motor.append(0x80)
pickupMotor = bytearray()
pickupMotor.append(0x60)


SRDY = 17 #yellow
MRDY = 27 #orange

ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)

servoKit = ServoKit(4)
# servoKit.setAngle(0, 110)
# servoKit.setAngle(0, 145)
firstFLG = 0
START = ""
startSerial = ""

def main(grid):
    myclient = MongoDBconnection()
    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]

    mycol_nav.drop()
    mycol_packages.drop()
    gridDim = grid.split('x')
    mycol_nav.insert_many(insertGrid(gridDim[1],gridDim[0]))

    item = dict()


    print("Beginning Training/Calibration\n\n")

    # startSerial = ser.readline().decode('utf-8', 'ignore').rstrip()
    START = "C3"
    Dir = "Left"

    for collection in mycol_nav.find():
        if collection["Name"] == START:
            Nav = collection
            break

    ser.write(leftTick)
    while(1):
        sensBool = readIRsensors()
        if sensBool == 2:
            #Move up to Read Pacakge
            UART_send_repeat(upTick)
            while(1):
                if readIRsensors() == 1:
                    break
            #Read QR code for both heights
            for i in range(0,2):
                if i == 0:
                    servoKit.setAngle(0, 110)
                else:
                    servoKit.setAngle(0, 145)
                QR = readQR()
                if QR != "":
                    QR = json.loads(QR)

                    if (Dir == "Right"):
                        item["Left"] = Nav["Name"]
                        item["Right"] = Nav["Right"]
                    else:
                        item["Left"] = Nav["Left"]
                        item["Right"] = Nav["Name"]
                    item["Level"] = i+1
                    item["Item"] = QR["Item"] 
                    mycol_packages.insert_one(item)
            #Go back on main line
            UART_send_repeat(downNode)
            while readIRsensors != 1:
                pass
            #Continue going the direction
            if Dir == "Right":
                UART_send_repeat(rightTick)
            else:
                UART_send_repeat(leftTick)

        elif sensBool == 1:
            #Get node in the direction that it is moving
            if Dir == "Right":
                tempNav = Nav["Right"]
                for collection in mycol_packages.find():
                    if collection["Name"] == tempNav:
                        Nav = collection
                        break
            else:
                tempNav = Nav["Left"]
                for collection in mycol_packages.find():
                    if collection["Name"] == tempNav:
                        Nav = collection
                        break
            
            if (endOfTraining(Nav, Dir) == True):
                print('End of Training going back to start')
                nodeMove(convertToGrid(Nav["Item"]), convertToGrid(START))
                return
            elif (moveUpLogic(Nav, Dir) == True):
                UART_send_repeat(downNode)
                while(readIRsensors() != 1):
                        pass
                #Swap direction
                if (Dir == "Left"):
                    Dir = "Right"
                    UART_send_repeat(rightTick)
                else:
                    Dir = "Left"
                    UART_send_repeat(leftTick)
            else:
                if Dir == "Left":
                    UART_send_repeat(leftTick)
                else:
                    UART_send_repeat(rightTick)

def PinSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MRDY, GPIO.OUT)
    GPIO.setup(SRDY, GPIO.IN)
    GPIO.output(MRDY, GPIO.HIGH)

def readIRsensors():
    read_serial = ser.readline().decode('utf-8', 'ignore').rstrip()
    print(read_serial)
    if read_serial == "1":
        return 1
    elif read_serial == "2":
        return 2
    elif read_serial == "3":
        return 3
    else:
        return 0


   

#Connects to the mongoDB client and prints out the available databases and collections
def MongoDBconnection():
    print("Connecting to Database")
    myclient = pymongo.MongoClient("mongodb://localhost:27017/")
    print("Connected...")
    print("Databases Available")
    print(myclient.list_database_names())
    print("Collections Available in WarehouseMap")
    mydb = myclient["WarehouseMap"]
    print(mydb.list_collection_names())
    return myclient

def readQR():
    # set up camera object
    cap = cv.VideoCapture(0)
    # cap.set(3,640)
    # cap.set(4,480)
    myData = ""
    startTime = time.time()
    while True:
        ret, frame = cap.read()
        currentTime = time.time()

        for barcode in decode(frame):
            #print(barcode.data)
            myData = barcode.data.decode('utf-8')
            print(myData)
            pts = np.array([barcode.polygon],np.int32)
            cv.polylines(frame,[pts],True,(255,0,0),5)
            pts2 = barcode.rect
            cv.putText(frame,myData,(pts2[0],pts2[1]), cv.FONT_HERSHEY_COMPLEX,1,(255,0,0),2)

        cv.imshow('In',frame)
        if currentTime - startTime >= 3:
            break
        if myData != "":
            return myData
    return myData

def insertGrid(numCol, numRow):
    mongoInput = list(dict())
    item = dict()

    for i in range(1, numRow + 2):
        for j in range(1,numCol + 2):
            item["Name"] = str(chr(ord('@') + i)) + str(j)
            if j == 1:
                item["Down"] = "null"  
            else:
                item["Down"] = str(chr(ord('@') + (i) )) + str(j-1)

            if i == 1:
                item["Left"] = "null" 
            else:
                item["Left"] = str(chr(ord('@') + i-1)) + str(j)

            if j == numCol + 1:
                item["Up"] = "null"
            else:
                item["Up"] = str(chr(ord('@') + i)) + str(j+1)

            if i == numRow + 1:
                item["Right"] = "null"
            else:
                item["Right"] = str(chr(ord('@') + (i+1) )) + str(j)

            mongoInput.append(dict(item))
    return mongoInput

def endOfTraining(readQR2, Dir):
    if (Dir == "Left"):
        if (readQR2["Left"] == "null" and readQR2["Down"] == "null"):
            return True
    else:
        if (readQR2["Right"] == "null" and readQR2["Down"] == "null"):
            return True
    return False

def moveUpLogic(readQR2, Dir):
    if (Dir == "Left"):
        if (readQR2["Left"] == "null"):
            return True
    else:
        if (readQR2["Right"] == "null"):
            return True
    return False

#Breaks up the node into the letters and numbers and returns a list
def splitNode(node):
    match = re.match(r"([a-z]+)([0-9]+)", node, re.I)
    if match:
        items = match.groups()
    items = list(items)
    return items

#Converts nodes such as A0 t0 (0,0) their number equivalent 
#in the number format.
#Only supports A-Z not AA AB etc
def convertToGrid(node):
    split = splitNode(node)
    temp = list(split[0])
    split[0] = ord(temp[0]) - 65
    split[1] = int(split[1])
    return split

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
        if count == 4:
            break
    return

def nodeMove(start, end):
    count = 0

    if start[0] == end[0]:
        if start[1] != end[1]:
            if (start[1] > end[1]):
                UART_send_repeat(downNode)
            else:
                UART_send_repeat(upNode)
    else:
        if (start[0] > end[0]):
            UART_send_repeat(leftNode)
        else:
            UART_send_repeat(rightNode)
    Horizontal = True
    while(1):
        print(start)
        if start[0] == end[0]:
            Horizontal = False
            if start[1] == end[1]:
                break
        if Horizontal == True:
            #Go Horizontal first
            if (start[0] > end[0]):
                navMove("Left", start)
            else:
                navMove("Right", start)
        else:
            #Go Vertically  
            if (start[1] > end[1]):
                navMove("Down", start)
            else:
                navMove("Up", start)

        if start[0] == end[0]:
            if count == 0:
                if (start[1] > end[1]):
                    UART_send_repeat(downNode)
                else:
                    UART_send_repeat(upNode)
            count = count + 1
            Horizontal = False
    UART_send_repeat(stopTick)
    return

if __name__ == "__main__":
    main(sys.argv[1])
