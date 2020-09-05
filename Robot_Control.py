#Copyright 2020 James Beadle

#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at

#    http://www.apache.org/licenses/LICENSE-2.0

#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.

# Robot Control Code
# James Beadle May 2020

# This code comunicates with an Arduino to control the movement of three stepper motors -
# the exact position of each stepper motor can be controled, as well as calibrated
# This code is also connected to a camera, which can be instructed to take and save pictures
# This code also activates a neural network running in tensorflow, to classify pictures in a
# specified directory
# This code also collects all of the classification data, and uses it to identify individual plant
# leaves, and their location in 3D space.

#location of the neural network initiation function
classifierFile = #include the directory of the file Object_detection_image_dir_to_txt.py, which should be located in the Object detection folder of the Tensorflow API
classifierFunction = "classifyDirectory"

#set working folders
workingDir = #"include the path of the workingDirectory"#
imageDir = #"include name of the image directory inside the working directory"#

#import libraries
import os
import numpy as np
import serial
import time
import cv2
import random
import sys
#include the classification python file and its relevant paths
import importlib.util
spec = importlib.util.spec_from_file_location("classifyDirectory", classifierFile + "/Object_detection_image_dir_to_txt.py")
classifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(classifier)
sys.path.append(#"include path of Tensorflow API Object Detection folder"#)
sys.path.append(#"include path of Tensorflow API Research Folder"#)




#data object class
class allDataAcquisition:
    def __init__(self):
        self.dataPoints = []
        
    #add or remove a new data point object
    def addDataPoint(self, imageID, xMin, xMax, yMin, yMax, imageWidth, imageHeight, tolerance):
        pointPosition = [(xMin + xMax)/2, (yMin + yMax)/2]
        withinWidth = (xMin > tolerance*imageWidth) and (xMax < imageWidth - tolerance*imageWidth)
        withinHeight = (yMin > tolerance*imageHeight) and (yMax < imageHeight - tolerance*imageHeight)
        if (withinWidth == True) and (withinHeight == True):
            self.dataPoints.append(dataPointAcquisition(imageID, xMin, xMax, yMin, yMax))
        return       

    ###Get Functions
    def getNumberOfPoints(self):
        return len(self.dataPoints)
    
    #get the number of images
    def getNumberOfImages(self):
        maxImageID = 0
        for dataPoint in self.dataPoints:
            if (dataPoint.getImageID() > maxImageID):
                maxImageID = dataPoint.getImageID()
        return (maxImageID + 1)

    #get the point IDs of all the points in an image
    def getPointIDsOfPointsInImage(self, imageID):
        pointIDs = []
        for dataPoint in self.dataPoints:
            if (dataPoint.getImageID() == imageID):
                pointIDs.append(dataPoint.getPointID())
        return pointIDs

    #get the point IDs of all the points in a class
    def getPointIDsOfPointsOfSameClass(self, Class):
        pointIDs = []
        for dataPoint in self.dataPoints:
            if (dataPoint.getClass() == Class):
                pointIDs.append(dataPoint.getPointID())
        return pointIDs

    #find point ID of a point in an image of a particular class
    def getPointIDOfClassInImage(self, Class, imageID):
        for dataPoint in self.dataPoints:
            if ((dataPoint.getClass() == Class) and (dataPoint.getImageID() == imageID)):
                return dataPoint.getPointID()
        return None

    #return the class of a point
    def getClass(self, pointID):
        return self.dataPoints[pointID].getClass()

    #return the image ID a point is in
    def getImageID(self, pointID):     
        return self.dataPoints[pointID].getImageID()        

    #return the width and height of a point
    def getWidthAndHeight(self, pointID):
        return self.dataPoints[pointID].getWidthAndHeight()
            
    def getBoundingBoxPosition(self, pointID):
        return self.dataPoints[pointID].getPosition()

    #return the center position of a point
    def getCenterPosition(self, pointID):      
        return self.dataPoints[pointID].getCenterPosition()

    #find the highest class number
    def getHighestClass(self):
        highestClass = 0
        for dataPoint in self.dataPoints:
            try:
                if (dataPoint.getClass() > highestClass):
                    highestClass = dataPoint.getClass()
            except:
                pass
        return highestClass

    def getDepth(self, pointID):
        return self.dataPoints[pointID].getDepth()

    def getRelativePosition(self, pointID):
        return self.dataPoints[pointID].getRelativePosition()

    ###Set Functions

    #set the class of a point
    def setClass(self, pointID, Class):
        return self.dataPoints[pointID].setClass(Class)

    #set the depth of a point
    def setAllDepthsOfAClass(self, Class, depth):
        for dataPoint in self.dataPoints:
            if (dataPoint.getClass() == Class):       
                dataPoint.setDepth(depth)

    def setRelativePosition(self, pointID, X, Y):
        return self.dataPoints[pointID].setRelativePosition(X, Y)

    ###Debug

    #print all the datapoints data in a format that can be imported into excel
    def deBug(self):
        for dataPoint in self.dataPoints:
            print(str(dataPoint.getCenterPosition()[0]) + '\t' + str(dataPoint.getCenterPosition()[1]) + '\t' + str(dataPoint.getClass()) + '\t' + str(dataPoint.getImageID()) + '\t' + str(dataPoint.getDepth()) + '\t' + str(dataPoint.getRelativePosition()))


#Data point class
class dataPointAcquisition:
    pointID = 0
    def __init__(self, imageID, xMin, xMax, yMin, yMax):
        self.imageID = int(imageID)
        self.xMin = xMin
        self.yMin = yMin
        self.xMax = xMax
        self.yMax = yMax
        self.Class = None
        self.depth = None
        self.RelativeXPosition = None
        self.RelativeYPosition = None
        self.pointID = dataPointAcquisition.pointID
        dataPointAcquisition.pointID += 1

    ###Get functions 

    def getCenterPosition(self):
        return [(self.xMin + self.xMax)/2, (self.yMin + self.yMax)/2]

    def getPosition(self):
        return [self.xMin, self.yMin, self.xMax, self.yMax]
    
    def getWidthAndHeight(self):
        return [abs(self.xMax - self.xMin), abs(self.yMax - self.yMin)]

    def getImageID(self):
        return self.imageID

    def getClass(self):
        return self.Class

    def getPointID(self):
        return self.pointID

    def getDepth(self):
        return self.depth

    def getRelativePosition(self):
        return [self.RelativeXPosition, self.RelativeYPosition]

    ###Set functions

    def setClass(self, Class):
        self.Class = Class

    def setDepth(self, depth):
        self.depth = depth

    def setRelativePosition(self, X, Y):
        self.RelativeXPosition = X
        self.RelativeYPosition = Y

#delete every file in the image directory folder
def clearImageDirectory(workingDir, imageDir):
    files = os.listdir(workingDir + imageDir)
    for file in files:
        os.remove(workingDir + imageDir + "/" + file)

#scan the plant by taking several pictures in a straight line with equal spacing over it
def scanPlant(ser, start, end, steps, rTimeOut, cTimeOut, attempts, camera, workingDir, imageDir):
    
    commands = []
    #subdivide the start and end positions
    for i in range(steps+1):
        x = int(start[0] + i*((end[0]-start[0])/steps))
        y = int(start[1] + i*((end[1]-start[1])/steps))
        z = int(start[2] + i*((end[2]-start[2])/steps))
        
        key = random.randint(65,90)
        commands.append([98,key,x//64,x%64,y//64,y%64,z//64,z%64,101])
    
    #send each command in turn
    for i in range(steps+1):
        success = transmitData(ser, commands[i], rTimeOut, cTimeOut, attempts)
        if success == False:
            return False
        #take a picture
        if camera == True:
            cam = cv2.VideoCapture(0)
            ret, image = cam.read()

            if ret:
                cv2.imwrite(workingDir + imageDir + "/" + str(i) + '.jpg',image)
                cam.release()
        else:
            print("Picture Taken")
    return True
                
#transmit a command        
def transmitData(ser, txCommand, rTimeOut, cTimeOut, attempts):       
    txCommandStatus = 'pending'
    #for each attempt
    while attempts != 0:
        if txCommandStatus == 'pending':
            #send the command
            ser.write(bytes(txCommand))
            txCommandStatus = 'sent'
            dataRecieved = waitOnCommand(rTimeOut)
        else:
            #wait for return command
            dataRecieved = waitOnCommand(cTimeOut)
        #if no data is recieved
        if dataRecieved == False:
            txCommandStatus = 'pending'
            attempts = attempts - 1
        else:
            #decode the command sent
            rxCommands = decodeCommand(ser.readline())
            recievedCommand = False
            for j in range(len(rxCommands)):
                if (rxCommands[j][0] == txCommand[1]):
                    #if it was a recieved command
                    if (rxCommands[j][1] == 114):
                        txCommandStatus = 'recieved'
                        recievedCommand = True
                    #if it was a complete command
                    elif (rxCommands[j][1] == 99):
                        txCommandStatus = 'complete'
                        return True
                    #if it was a failed command
                    elif (rxCommands[j][1] == 102):
                        txCommandStatus = 'failed'
                        return False
            #have another attempt
            if recievedCommand == False: 
                txCommandStatus == 'pending'
                attempts = attempts - 1
    return False

#wait until a command is recieved
def waitOnCommand(timeOut):
    checkRate = 10
    for j in range(timeOut*checkRate):
        time.sleep(1/checkRate)
        if (ser.in_waiting != 0):
            return True
    print("timeout")
    return False

#decide the commands in the serial buffer
def decodeCommand(serialData):
    commands = []
    for i in range(len(serialData) - 3):    
        if serialData[i] == 98:
            if serialData[i+3] == 101:
                commands.append([serialData[i+1],serialData[i+2]])
    return commands

#decode the neural networks classification data
def scrapedatatxt(workingDir, imageWidth, imageHeight, tolerance):
    allData = allDataAcquisition()
    file = open(workingDir + imageDir + "/classifications.txt", "r")
    data = eval(file.read())
    for i in range(len(data)): #for each image
        xMins = []
        xMaxs = []
        yMins = []
        yMaxs = []      
        for j in range(len(data[i])): #for each point
            yMins.append(data[i][j][0]*imageHeight)
            xMins.append(data[i][j][1]*imageWidth)
            yMaxs.append(data[i][j][2]*imageHeight)
            xMaxs.append(data[i][j][3]*imageWidth)
            
        for j in range(len(xMins)):
            #add points to the data point object
            allData.addDataPoint(i, xMins[j], xMaxs[j], imageHeight-yMaxs[j], imageHeight-yMins[j], imageWidth, imageHeight, tolerance)
    return allData

def setAllClasses(allData, kp, ks, probabilityLimit):
    #get the first image and classify each point
    firstImagePointIDs = allData.getPointIDsOfPointsInImage(0)
    for pointNumber in range(len(allData.getPointIDsOfPointsInImage(0))):
        allData.setClass(firstImagePointIDs[pointNumber], pointNumber)

    #loop through each image, attempting to classify all the points
    for imageID in range(1, allData.getNumberOfImages()):
        
        #get the bounding box data of the previous image
        previousImagePointID = allData.getPointIDsOfPointsInImage(imageID-1)
        
        previousImageCenterPositionData = []
        previousImageDimensionData = []
        for pointID in previousImagePointID:
            previousImageCenterPositionData.append(allData.getCenterPosition(pointID))
            previousImageDimensionData.append(allData.getWidthAndHeight(pointID))
        
        #loop through each point in the current image
        for subjectPointID in allData.getPointIDsOfPointsInImage(imageID):
            
            #get the bounding box data of the subject point
            subjectPointBoundingBoxSizeData = allData.getWidthAndHeight(subjectPointID)
            subjectImageCenterPositionData = allData.getCenterPosition(subjectPointID)


            #make a judgement on each pair of points
            probability = []
            for previousImagePointIndex in range(len(previousImageCenterPositionData)):
                #if the point is behind the subject, reject it as it shouldnt have moved in that direction
                if previousImageCenterPositionData[previousImagePointIndex][1] < subjectImageCenterPositionData[1]:
                    probability.append(0)
                else:
                    centerPosition = abs(subjectImageCenterPositionData[0] - previousImageCenterPositionData[previousImagePointIndex][0])
                    BoundingBoxXSize = abs(subjectPointBoundingBoxSizeData[0] - previousImageDimensionData[previousImagePointIndex][0])
                    BoundingBoxYSize = abs(subjectPointBoundingBoxSizeData[1] - previousImageDimensionData[previousImagePointIndex][1])
                    BoundingBoxSize = BoundingBoxXSize + BoundingBoxYSize
                    
                    probability.append(1/(ks*BoundingBoxSize + kp*centerPosition))
            
            #find the best match if there is one
            bestProbabilityIndex = findTheBestProbability(probability)

            #set the class if one is found, else make a new one       
            if ((bestProbabilityIndex != -1) and (probability[bestProbabilityIndex] > probabilityLimit)):
                allData.setClass(subjectPointID, allData.getClass(previousImagePointID[bestProbabilityIndex]))
            else:
                newClass = allData.getHighestClass() + 1
                allData.setClass(subjectPointID, newClass)
                
        #test if two leafs have been given the same class in one image
        currentImagePointIDs = allData.getPointIDsOfPointsInImage(imageID)
        for i in range(len(currentImagePointIDs)):
            currentImagePointClasses = []
            for pointID in currentImagePointIDs:
                currentImagePointClasses.append(allData.getClass(pointID))

            #if there is a duplicate class
            if (currentImagePointClasses.count(currentImagePointClasses[i]) > 1):

                #find all the point IDs of points in an image with the same class as currentImagePointIDs[i]
                pointIDIssues = []
                for j in range(len(currentImagePointClasses)):
                    if (currentImagePointClasses[j] == currentImagePointClasses[i]):
                        pointIDIssues.append(currentImagePointIDs[j])
                        
                #see if past occurances can be used to find the conrect classification, through y position          
                penultimatePointID = allData.getPointIDOfClassInImage(currentImagePointClasses[i], imageID-1)
                antepenultimatePointID = allData.getPointIDOfClassInImage(currentImagePointClasses[i], imageID-2)
                if ((penultimatePointID != None) and (antepenultimatePointID != None)):
                    penultimateDist = allData.getCenterPosition(penultimatePointID)[1]
                    antepenultimateDist = allData.getCenterPosition(antepenultimatePointID)[1]
                    displacement = antepenultimateDist - penultimateDist
                    potentialDisplacements = []
                    for j in range(currentImagePointClasses.count(currentImagePointClasses[i])):
                        potentialDisplacements.append(penultimateDist - allData.getCenterPosition(currentImagePointIDs[i])[1])
                    #calculate the error in each points position when compared to previous positions 
                    error = []
                    for j in range(len(potentialDisplacements)):
                        error.append(abs(potentialDisplacements[j]-displacement))
                    correctPoint = np.argmin(error)
                    for j in range(len(potentialDisplacements)):
                        if j != correctPoint:
                            newClass = allData.getHighestClass() + 1
                            allData.setClass(pointIDIssues[j], newClass)
                            
                #if there are no previous points then they have to be assumed to be from seperate leafs
                else:
                    for pointID in pointIDIssues:
                        newClass = allData.getHighestClass() + 1
                        allData.setClass(pointID, newClass)
                        
    return allData

def findTheBestProbability(probability):
    #find the index of the list which has the highest probability
    maxprobability = 0
    probabilityIndex = -1
    for i in range(len(probability)):
        if (probability[i] > maxprobability):
            maxprobability = probability[i]
            probabilityIndex = i
    return probabilityIndex 

def findClassPointsDisplacements(allData, Class):
    #find the y position of each point of a particular class in every image
    pointPosition = []
    #go through every image looking for a particular class
    for imageID in range(allData.getNumberOfImages()):
        pointIDs = allData.getPointIDsOfPointsInImage(imageID)
        found = False
        #go through each point in that image
        for pointID in pointIDs:
            #if a point is of the particular class, record its vertical distance
            if (allData.getClass(pointID) == Class):
                found = True
                pointPosition.append(allData.getCenterPosition(pointID)[1])
        #if a class does not exist in an image, add a flag to show this
        if (found == False):
            pointPosition.append('-')
            
            
    return findAverageDisplacement(pointPosition)

def findAverageDisplacement(pointPosition):
    #create two pointers to denote the position of two points to compare
    i = 0
    j = 1
    distances = []
    #while the second pointer is not larger than the number of points
    while j < len(pointPosition):
        #if both pointers point to an float, calculate the difference
        if (pointPosition[i] != '-'):
            if (pointPosition[j] != '-'):
                distances.append((pointPosition[i]-pointPosition[j])/(j-i))
                #increase both pointers
                i = j
                j = j + 1
            else:
                #increment the second pointer
                j = j + 1
        else:
            #increase both pointers
            i = j
            j = j + 1

    #find the mean difference
    total = 0 
    if (len(distances) != 0):
        for i in range(len(distances)):
            total = total + distances[i]
        average = total/len(distances)
        return average
    else:
        return -1

def calculateDepth(allData, cameraMovement, coefficient, power):
    #calculate the depth of each point
    for Class in range(allData.getHighestClass()+1): 
        displacement = findClassPointsDisplacements(allData, Class)
        if (displacement != -1):
            allData.setAllDepthsOfAClass(Class,(displacement/(coefficient*cameraMovement))**(1/power))
    return allData

def calculateDisplacement(allData, imageWidth, imageHeight, horisontalFOV, verticalFOV):
    #calculate the position of a point
    for imageID in range(allData.getNumberOfImages()):
        pointIDs = allData.getPointIDsOfPointsInImage(imageID)
        for pointID in pointIDs: 
            D = allData.getDepth(pointID)
            if D != None:
                PdX = allData.getCenterPosition(pointID)[0] - (imageWidth/2)
                PmX = imageWidth/2
                phiX = horisontalFOV/2
                        
                PdY = allData.getCenterPosition(pointID)[1] - (imageHeight/2)
                PmY = imageHeight/2
                phiY = verticalFOV/2
                        
                angularDeviationX = np.arcsin((PdX*np.sin(phiX))/PmX)
                angularDeviationY = np.arcsin((PdY*np.sin(phiY))/PmY)

                displacementDeviationX = D*np.tan(angularDeviationX)
                displacementDeviationY = D*np.tan(angularDeviationY)
                
                allData.setRelativePosition(pointID, displacementDeviationX, displacementDeviationY)
    return allData

def findLeafStepPositions(allData, cameraMovement, yrange):
    scanpositions = []
    if allData.getNumberOfPoints() != 0:
        for Class in range(allData.getHighestClass()+1):
            pointIDs = allData.getPointIDsOfPointsOfSameClass(Class)
            positions = []
            for pointID in pointIDs:
                if allData.getDepth(pointID) == None:
                    break
                relativePosition = allData.getRelativePosition(pointID)
                depth = allData.getDepth(pointID)
                xposition = mmToStepConversion(relativePosition[1])+mmToStepConversion(cameraMovement)*allData.getImageID(pointID)
                yposition = mmToStepConversion(relativePosition[0])+yrange/2                
                zposition = mmToStepConversion(depth)
                positions.append([xposition,yposition,zposition])
            total = [0,0,0]
            #find the average position of all the points for a particular leaf
            if len(positions) == 0:
                continue
            for i in range(len(positions)):
                total[0] = total[0] + positions[i][0]
                total[1] = total[1] + positions[i][1]
                total[2] = total[2] + positions[i][2]
            for i in range(3):
                total[i] = total[i]/len(positions)
            #take account for the distance in position of the camera to the probing arm
            total[0] = total[0] - mmToStepConversion(60)
            total[2] = total[2] - 200
            scanpositions.append(total)
    return scanpositions

def removeDuplicatePredictions(scanpositions):
    #prevent high time complexity operation
    if len(scanpositions) < 10:
        for i in range(len(scanpositions)):
            dupeFlag = [i]
            for j in range(len(scanpositions)):
                if i < j:
                    distance = ((scanpositions[i][0] - scanpositions[j][0])**2 + (scanpositions[i][1] - scanpositions[j][1])**2 + (scanpositions[i][2] - scanpositions[j][2])**2)**(0.5)
                    if distance < 160: #if two points are less than 25mm away from each other
                        dupeFlag.append(j)          
            xtot = 0
            ytot = 0
            ztot = 0
            if len(dupeFlag) > 1:
                #average all the similer points
                for j in range(len(dupeFlag)):
                    xtot = xtot + scanpositions[dupeFlag[j]][0]
                    ytot = ytot + scanpositions[dupeFlag[j]][1]
                    ztot = ztot + scanpositions[dupeFlag[j]][2]               
                
                scanpositions[i][0] = xtot / len(dupeFlag)
                scanpositions[i][1] = ytot / len(dupeFlag)
                scanpositions[i][2] = ztot / len(dupeFlag)
                
                #remove the duplicate
                for j in range(len(dupeFlag)-1):
                    scanpositions.pop(dupeFlag[j+1])
                    
    return scanpositions
                        
                    

#move the motors to position the sensor over each leaf
def measureplant(scanpositions, ser, rTimeOut, cTimeOut, attempts, xrange, yrange, zrange):
    #for each detected leaf
    for i in range(len(scanpositions)):
        key = random.randint(65,90)
        x = int(scanpositions[i][0])
        y = int(scanpositions[i][1])
        z = int(scanpositions[i][2])
        #test if its out of range
        if (x < 0) or (x > xrange) or (y < 0) or (y > yrange) or (z < 0) or (z > zrange):
            print("Out of range")
            continue
        #move above the leaf before attempting to make contact
        position1 = [98,key,x//64,x%64,y//64,y%64,0,0,101]
        position2 = [98,key,x//64,x%64,y//64,y%64,z//64,z%64,101]
        success = transmitData(ser, position1, rTimeOut, cTimeOut, attempts)
        if success == False:
            return False
        success = transmitData(ser, position2, rTimeOut, cTimeOut, attempts)
        if success == False:
            return False
        print("Take Measurement")
        success = transmitData(ser, position1, rTimeOut, cTimeOut, attempts)
        if success == False:
            return False           
    return True 


#convert between steps and mm
def mmToStepConversion(mm):
    return int(mm*500/78)
def StepTommConversion(step):
    return int(step*78/500)   

######################CODE STARTS########################
    
#variables
kp = 1
ks = 0.01
probabilityLimit = 0.07
coefficient = 381
power = -0.95
displyImage = 4
imageWidth  = 640
imageHeight = 480
horisontalFOV = 1.18
verticalFOV = 0.97
tolerance = 0.001
ser = serial.Serial('/dev/ttyUSB0',57600,timeout=0)
camera = True
xrange = 1200
yrange = 1400
zrange = 2000
scanStartPos = [0,yrange/2,0]
scanEndPos = [xrange,yrange/2,0]
scanSteps = 6
cameraMovement = StepTommConversion(xrange/scanSteps)
recievedTimeOut = 5
completedTimeOut = 20
calibratedTimeOut = 50
transmissionAttempts = 3

#allow arduino to reset
time.sleep(2)

#remove any previous images
clearImageDirectory(workingDir, imageDir)

#calibrate motors
calibrated = transmitData(ser, bytes([98,random.randint(65,90),99,101]), recievedTimeOut, calibratedTimeOut, transmissionAttempts)

#move the motors to take pictures
picturesTaken = scanPlant(ser, scanStartPos, scanEndPos, scanSteps, recievedTimeOut, completedTimeOut, transmissionAttempts, camera,  workingDir, imageDir)

#classify leaves in the pictures
classificationsMade = classifier.classifyDirectory(workingDir + imageDir)

#collect all the classification data
allData = scrapedatatxt(workingDir, imageWidth, imageHeight, tolerance)

#give each classificaion a leaf lable
allData = setAllClasses(allData, kp, ks, probabilityLimit)
allData.deBug()

#calculate the depth of each leaf
allData = calculateDepth(allData, cameraMovement, coefficient, power)

#calculate the x and y position of each leaf
allData = calculateDisplacement(allData, imageWidth, imageHeight, horisontalFOV, verticalFOV)

#convert the position label of each point to a step position for each leaf
scanpositions = findLeafStepPositions(allData, cameraMovement, yrange)

#Remove any duplicate detections
scanpositions = removeDuplicatePredictions(scanpositions)
print(scanpositions)

#move the probe to each detected leaf
measureplant(scanpositions, ser, recievedTimeOut, completedTimeOut, transmissionAttempts, xrange, yrange, zrange)
#calibrate
transmitData(ser, bytes([98,random.randint(65,90),99,101]), recievedTimeOut, calibratedTimeOut, transmissionAttempts)