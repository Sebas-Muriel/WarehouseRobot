import pymongo
import sys
import re



start = "A0"

def main(package):
    myclient = MongoDBconnection()
    mydb = myclient["WarehouseMap"]
    mycol_packages = mydb["QRs"]
    mycol_nav = mydb["Navigation"]
    CreatePath(mycol_nav, mycol_packages, package)
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

def CreatePath(Nav, QRs, item):
    itemLoc = QRs.find_one({}, {"_id": 0, "Name" :item, "Left": 1, "Right": 1})
    print(itemLoc)
    convertToGrid(start)
    #chooseInitialPath(itemLoc.Left, itemLoc.Right)
    # splitStart = splitNode(start)
    # print(splitStart)
    return

#Breaks up the node into the letters and numbers and returns a list
def splitNode(node):
    match = re.match(r"([a-z]+)([0-9]+)", node, re.I)
    if match:
        items = match.groups()
    items = list(items)
    return items

#Converts nodes such as A0 t0 (0,0) their number equivalent 
def convertToGrid(node):
    split = splitNode(node)
    temp = list(split[0])
    split[0] = ord(temp[0]) - 65
    split[1] = int(split[1])
    print(split)

def chooseInitialPath(left, right):
    leftSplit = splitNode(left)
    rightSplit = splitNode(right)
    startSplit = splitNode(start)
    
    return

if __name__ == "__main__":
    main(*sys.argv[1:])
