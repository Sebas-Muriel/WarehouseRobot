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



firstFLG = 0
START = ""
startSerial = ""

def main(package):
    myclient = MongoDBconnection()
    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]

    mycol_nav.drop()
    mycol_packages.drop()
    mycol_nav.insert_many(insertGrid(3,3))
    return

        
   

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
