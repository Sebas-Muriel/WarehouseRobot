#test
# need these installed on py: sudo apt-get install python-serial
import pymongo
import sys
import re
import math
import time
import serial
#import cv2


Left = '5'
Right = '2'
Up = '3'
Down = '4'
Stop = '0'
nodeMode = '10'
tickMode = '11'


ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)
firstFLG = 0
START = "D2"

def main(package):
    myclient = MongoDBconnection()
    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]
    currentPoint = convertToGrid(START)
    endingPoint = CreatePath(mycol_nav, mycol_packages, package)
    print("Ending Point", endingPoint)
    print("Starting Point", currentPoint)
    #While loop reading the Intersections/QRs until it gets to the package. 
    Horizontal = True
    print("Beginning Package Pickup\n\n", currentPoint, sep= "")


    #Decided which direction to go at start
    if currentPoint[0] == endingPoint["End"][0]:
        if currentPoint[1] == endingPoint["End"][1]:
            break
        else:
            if (currentPoint[1] > endingPoint["End"][1]):
                ser.write((Down + nodeMode).encode('ascii'))
            else:
                ser.write((Up + nodeMode).encode('ascii'))
    else:
        if (currentPoint[0] > endingPoint["End"][0]):
            ser.write((Left + nodeMode).encode('ascii'))
        else:
            ser.write((Right + nodeMode).encode('ascii'))


    while(1):
        if currentPoint[0] == endingPoint["End"][0]:
            Horizontal = False
            if currentPoint[1] == endingPoint["End"][1]:
                break
        if Horizontal == True:
            #Go Horizontal first
            #if IR sensor hits intersection
            if(readIRsensors() == True):
                if (currentPoint[0] > endingPoint["End"][0]):
                    navMove("Left", currentPoint)
                    ser.write(Left.encode('ascii'))
                else:
                    navMove("Right", currentPoint)
                    ser.write(Right.encode('ascii'))
        else:
            #Go Vertically  
            #if IR sensor hits intersection
            if (readIRsensors() == True):
                if (currentPoint[1] > endingPoint["End"][1]):
                    navMove("Down", currentPoint)
                    ser.write(Down.encode('ascii'))
                else:
                    navMove("Up", currentPoint)
                    ser.write(Up.encode('ascii'))

        if currentPoint[0] == endingPoint["End"][0]:
            Horizontal = False

        print(currentPoint)
    #Tell Arduino to stop
    ser.write(Stop.encode('ascii'))
    print("Robot must go", endingPoint["Direction"], "to get to the package")

    # searchPackage = False
    # while(searchPackage):

    #     #if the package is to the left, move left
    #     #if(endingPoint["Direction"] == Left):
    #         #searchMove(Left)
    #     #if the package is to the right, move right
    #     #elif(endingPoint["Direction"] == Right):
    #         #searchMove(Right)
    #     #readQR()
    #     #if QR code contains package
    #         #pickupPackage()
    #         #searchPackage = 0
    #     #returnToStart(currentPoint)
    #     return
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
line = ser.readline().decode('utf-8').rstrip()
def readIRsensors():
    read_serial = ser.readline().decode('utf-8').rstrip()
    print(read_serial)
    if read_serial == "1":
        return True
    else:
        return False

#Todo Read QR code code
def readQR():
    # set up camera object
    cap = cv2.VideoCapture(0)

    # QR code detection object
    detector = cv2.QRCodeDetector()

    while True:
        # get the image
        _, img = cap.read()
        # get bounding box coords and data
        data, bbox, _ = detector.detectAndDecode(img)

        # if there is a bounding box, draw one, along with the data
        if(bbox is not None):
            for i in range(len(bbox)):
                cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color=(255,
                         0, 255), thickness=2)
            cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            if data:
                print("Package: ", data)
        # display the image preview
        cv2.imshow("Package detection", img)
        if(cv2.waitKey(1) == ord("q")):
            break
    # free camera object and exit
    cap.release()
    cv2.destroyAllWindows()
    return

#Todo Create code to move robot when searching for a box inbetween nodes
def searchMove(searchDirection):
    if(searchDirection == Left):
        ser.write(Left.encode('ascii'))
        #move left a little bit
    if(searchDirection == Right):
        ser.write(Right.encode('ascii'))
        #move right a little bit
    return

#function for navigating to the end node from start
def navMove(navDirection, currentPoint):
    if(navDirection == "Left"):
        currentPoint[0] -= 1
    if(navDirection == "Right"):
        currentPoint[0] += 1
    if(navDirection == "Up"):
        currentPoint[1] += 1
    if(navDirection == "Down"):
        currentPoint[1] -= 1
    return

#function to pickup the package
def pickupPackage():
    #move forward
    #lift forklift
    #move back
    return

#function to drop off the package
def dropPackage():
    #turn 180 degrees
    #go forward
    #lower forklift
    #back up
    #turn 180 degrees
    return

#function to return to start
# def returnToStart(currentPoint):
#     #if not at an intersection
#         #move to an intersection
#     while(currentPoint[0] != 0):
#         if(currentPoint[0] > 0):
#             navMove["Left"]
#         else:
#             navMove["Right"]
#     while(currentPoint[1] != 0):
#         if(currentPoint[1] > 0):
#             navMove["Down"]
#         else:
#             navMove["Up"]
#     return

if __name__ == "__main__":
    main(sys.argv[1])
