#test
# need these installed on py: sudo apt-get install python-serial
import pymongo
import sys
import re
import math
import time
import serial
import json
import cv2
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

ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)



firstFLG = 0
START = "A0"
END = "B0"
startSerial = ""

def main(package):
    END = package
    PinSetup()
    time.sleep(2)
    myclient = MongoDBconnection()

    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]

    endingPoint = {}
    currentPoint = convertToGrid(START)
    # endingPoint = CreatePath(mycol_nav, mycol_packages, package)
    endingPoint["End"] = convertToGrid(END)
    print("Ending Point", endingPoint)
    print("Starting Point", currentPoint)
    #While loop reading the Intersections/QRs until it gets to the package. 
    Horizontal = True
    print("Beginning Package Pickup\n\n", currentPoint, sep= "")
    
    count = 0
    UART_send_repeat(rightNode)
    while(1):
        if currentPoint[0] == endingPoint["End"][0]:
            Horizontal = False
            if currentPoint[1] == endingPoint["End"][1]:
                break
        if Horizontal == True:
            #Go Horizontal first
            if (currentPoint[0] > endingPoint["End"][0]):
                navMove("Left", currentPoint)
            else:
                navMove("Right", currentPoint)
        else:
            #Go Vertically  
            if (currentPoint[1] > endingPoint["End"][1]):
                navMove("Down", currentPoint)
            else:
                navMove("Up", currentPoint)

        if currentPoint[0] == endingPoint["End"][0]:
            if count == 0:
                if (currentPoint[1] > endingPoint["End"][1]):
                    UART_send_repeat(downNode)
                else:
                    UART_send_repeat(upNode)
            count = count + 1
            Horizontal = False

        # print(currentPoint)
    #Tell Arduino to stop
    UART_send_repeat(stopTick)
    # print("Robot must go", endingPoint["Direction"], "to get to the package")
    UART_send_repeat(reset)
    return

def PinSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MRDY, GPIO.OUT)
    GPIO.setup(SRDY, GPIO.IN)
    GPIO.output(MRDY, GPIO.HIGH)


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

#Returns an array of the last point that the robot needs to travel
def CreatePath(Nav, QRs, item):
    for collection in QRs.find():
        if collection["Item"] == item:
            itemLoc = collection
            break
    #itemLoc = QRs.find({}, {"_id": 0, "Item" :item, "Left": 1, "Right": 1})
    print(itemLoc)

    itemLoc["Left"]  = convertToGrid(itemLoc["Left"])
    itemLoc["Right"] = convertToGrid(itemLoc["Right"])
    return chooseInitialPath(itemLoc["Left"], itemLoc["Right"])

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

#Calculates the distance between the two starting points
#Picks the closest one to the start point
def chooseInitialPath(left, right):
    helper = {}
    startSplit = convertToGrid(START)
    leftval = math.sqrt(((left[0]-startSplit[0])**2 + (left[1]-startSplit[1])**2))
    rightval = math.sqrt(((right[0]-startSplit[0])**2 + (right[1]-startSplit[1])**2))
    if (leftval >= rightval):
        helper["End"] = right
        helper["Direction"] = "Left"
        return helper
    else:
        helper["End"] = left
        helper["Direction"] = "Right"
        return helper

#Todo Create IR sensor reading code
#line = ser.readline().decode('utf-8').rstrip()
def readIRsensors():
    read_serial = ser.readline().decode('utf-8', 'ignore').rstrip()
    print(read_serial)
    if read_serial == "1":
        return True
    else:
        return False

#Todo Create code to move robot when searching for a box inbetween nodes
def searchMove(searchDirection):
    if(searchDirection == "Left"):
        UART_send_repeat(leftTick)
    if(searchDirection == "Right"):
        UART_send_repeat(rightTick)
    return

#function for navigating to the end node from start
def navMove(navDirection, currentPoint):
    if (readIRsensors() == True):
        if(navDirection == "Left"):
            currentPoint[0] -= 1
            UART_send_repeat(leftNode)
        if(navDirection == "Right"):
            currentPoint[0] += 1
            UART_send_repeat(rightNode)
        if(navDirection == "Up"):
            currentPoint[1] += 1
            UART_send_repeat(upNode)
        if(navDirection == "Down"):
            currentPoint[1] -= 1
            UART_send_repeat(downNode)
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
        if count == 4:
            break
    return

if __name__ == "__main__":
    main(sys.argv[1])
