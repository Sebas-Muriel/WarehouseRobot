#test
# need these installed on py: sudo apt-get install python-serial
#if i2c-1 not giving permissions run this command:
#sudo chmod a+rw /dev/i2c-*
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
import cv2


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


SRDY = 17 #yellow
MRDY = 27 #orange

ser = serial.Serial('/dev/ttyACM0',9600, timeout= 1)
servoKit = ServoKit(4)



firstFLG = 0
START = "B2"
startSerial = ""

def main(package):
    PinSetup()
    time.sleep(2)
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
    
    count = 0

    if currentPoint[0] == endingPoint["End"][0]:
        if currentPoint[1] != endingPoint["End"][1]:
            if (currentPoint[1] > endingPoint["End"][1]):
                UART_send_repeat(downNode)
            else:
                UART_send_repeat(upNode)
    else:
        if (currentPoint[0] > endingPoint["End"][0]):
            UART_send_repeat(leftNode)
        else:
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

        print(currentPoint)
    #Tell Arduino to stop
    UART_send_repeat(stopTick)
    print("Robot must go", endingPoint["Direction"], "to get to the package")

    searchMove(endingPoint["Direction"])

    searchPackage = False
    while(1):
        sensorBool = readIRsensors();
        # Move in the direcetino of the package
        if (sensorBool == 2 and searchPackage == False):
            UART_send_repeat(stopTick)
            QR = readQR()
            if (QR != "null"):
                QR = json.loads(QR)
                QR["Item"] = QR["Item"].replace(" ", "")
                print(QR)
                
                if (QR["Item"] == package):
                    print("here")
                    searchPackage = True
                    ser.flush()
                    break
                        #pickup
            searchMove(endingPoint["Direction"])
                        #searchPackage = True
                #move motor up
                    #if QR code contains package
                        #pickup
                        #searchPackage = True
            # returnToStart(currentPoint)
        if (sensorBool == 1):
            break
        
    #Reach the intersection
    UART_send_repeat(stopTick)
    UART_send_repeat(reset)

    print("beginning Going back to Start")
    endingPoint["End"] = convertToGrid(START)
    if (endingPoint["Direction"] == "Left"):
        currentPoint[0] -= 1
    else:
        currentPoint[0] += 1

    print("starting Point", currentPoint, "ending Point: ", endingPoint["End"])

    #Decided which direction to go at start
    if currentPoint[0] == endingPoint["End"][0]:
        if currentPoint[1] != endingPoint["End"][1]:
            if (currentPoint[1] > endingPoint["End"][1]):
                UART_send_repeat(downNode)
            else:
                UART_send_repeat(upNode)
    else:
        if (currentPoint[0] > endingPoint["End"][0]):
            UART_send_repeat(leftNode)
        else:
            UART_send_repeat(rightNode)
    
    Horizontal = True
    count = 0
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
                print("Going up")
                navMove("Up", currentPoint)

        if currentPoint[0] == endingPoint["End"][0]:
            if count == 0:
                if (currentPoint[1] > endingPoint["End"][1]):
                    UART_send_repeat(downNode)
                else:
                    UART_send_repeat(upNode)
                count = count + 1
            Horizontal = False

        print(currentPoint)
    #Tell Arduino to stop
    UART_send_repeat(stopTick)
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
    if itemLoc["Level"] == 1:
        servoKit.setAngle(0, 90)
    else:
        servoKit.setAngle(0, 45)


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
        return 1
    elif read_serial == "2":
        return 2
    else:
        return 0

#Todo Read QR code code
def readQR():
    # set up camera object
    cap = cv2.VideoCapture(-1)

    # QR code detection object
    detector = cv2.QRCodeDetector()
    startTime = time.time()

    while True:
        currentTime = time.time()
        if (currentTime - startTime > 5):
            data = "null"
            cap.release()
            cv2.destroyAllWindows()
            return data
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
                cap.release()
                cv2.destroyAllWindows()
                return data
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
    if(searchDirection == "Left"):
        UART_send_repeat(leftTick)
    if(searchDirection == "Right"):
        UART_send_repeat(rightTick)
    return

#function for navigating to the end node from start
def navMove(navDirection, currentPoint):
    if (readIRsensors() == 1):
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
