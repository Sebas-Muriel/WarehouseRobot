#test
# need these installed on py: sudo apt-get install python-serial
import pymongo
import sys
import re
import math
import time
import serial
import cv2


leftTick = bytearray()
leftTick.append(0x03)
leftNode = bytearray()
leftNode.append(0x02)
rightTick = bytearray()
rightTick.append(0x05)
rightNode = bytearray()
rightNode.append(0x04)
upTick = bytearray()
upTick.append(0x07)
upNode = bytearray()
upNode.append(0x06)
downTick = bytearray()
downTick.append(0x09)
downNode = bytearray()
downNode.append(0x08)
stopTick = bytearray()
stopTick.append(0x01)
stopNode = bytearray()
stopNode.append(0x00)
reset = bytearray()
reset.append(0x80)


ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)
firstFLG = 0
START = ""
startSerial = ""

def main(package):
    myclient = MongoDBconnection()
    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]
    readQR = dict()
    itemsList = list(dict())
    item = dict()
    Dir = "Right"

    print("Beginning Training/Calibration\n\n")


    startSerial = ser.readline().decode('utf-8', 'ignore').rstrip()
    readQR = 0; #readQR returns the data
    START = readQR["Name"]
    mycol_nav.insert_one(readQR)
    ser.write(rightTick)
    count = 0
    while(1):
        if readIRsensors == True:
            count += 1
            for i in range(0,2):
                if (Dir == "Right"):
                    item["Left"] = readQR["Name"]
                    item["Right"] = readQR["Right"]
                else:
                    item["Left"] = readQR["Left"]
                    item["Right"] = readQR["Name"]
                item["Level"] = i
                item["Item"] = "Test1" #read from QR
                #Append item JSON to the whole list
                mycol_packages.insert_one(item)
                #itemsList.append(item)
                #move motor up to the next level
        if count == 4:
            readQR = 0 #read the node qr

            if (endOfTraining(readQR, Dir) == True):
                print('End of Training going back to start')
                returnToStart(START, readQR["Name"])
            elif (moveUpLogic(readQR, Dir) == True):
                ser.write(upNode)
                while(readIRsensors() == False):
                    pass
                #Swamp direction
                if (Dir == "Left"):
                    Dir = "Right"
                    ser.write(rightTick)
                else:
                    Dir = "Left"
                    ser.write(leftTick)
                readQR = 0 #readQR
                mycol_nav.insert_one(readQR)
            else:
                mycol_nav.insert_one(readQR)
            count = 0
                

   

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

def readIRsensors():
    read_serial = ser.readline().decode('utf-8', 'ignore').rstrip()
    print(read_serial)
    if read_serial == "1":
        return True
    else:
        return False

def endOfTraining(readQR2, Dir):
    if (Dir == "Left"):
        if (readQR2["Left"] == "null" and readQR2["Up"] == "null"):
            return True
    else:
        if (readQR2["Right"] == "null" and readQR2["Up"] == "null"):
            return True
    return False

def returnToStart(start, end):
    st = convertToGrid(start)
    ed = convertToGrid(end)
    return

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

if __name__ == "__main__":
    main(sys.argv[1])
