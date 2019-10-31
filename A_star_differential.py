'''
#  Copyright 2019 Koyal Bhartia
#  @file    A* - Differential.py
#  @author  Koyal Bhartia
#  @date    26/04/2019
#  @version 1.0
#
#  @brief This is the code to solve the 8-puzzle
#
# @Description This code has functions which returns all the possible paths that can be
# traversed given any goal matrix. Given any input matrix it gives the path to the goal matrix
#
'''
import argparse
import numpy as np
import os, sys
from numpy import linalg as LA
from numpy import linalg as la
from matplotlib import pyplot as plt
import math
from PIL import Image
import random

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2


def checkBoundary(x_min,y_min,x_max,y_max):
    x_minTot,y_minTot=x_min-TotClear,y_min-TotClear
    x_maxTot,y_maxTot=x_max+TotClear,y_max+TotClear
    if (x_minTot)<0: x_minTot=x_min
    if (x_maxTot)>image_width: x_maxTot=x_max
    if (y_minTot)<0: y_minTot=y_min
    if (y_maxTot)>image_height: y_maxTot=y_max
    return x_minTot,x_maxTot,y_minTot,y_maxTot


def CreateMap():

    def circle(x,y,x_center,y_center,radius):
        if (x-x_center)**2+(y-y_center)**2<(radius+TotClear)**2:
            image[1010-1-y,x,:]=0

    def rectangle(x,y,x_min,y_min,x_max,y_max):
        x_minTot,x_maxTot,y_minTot,y_maxTot=checkBoundary(x_min,y_min,x_max,y_max)
        if x in range (int(x_minTot),int(x_maxTot)) and y in range(int(y_minTot),int(y_maxTot)):
            image[1010-1-y,x,:]=0
    
    for x in range (1110):
        for y in range(1010):
           circle(x,y,390,965,81/2) 
           circle(x,y,438,736,81/2) 
           circle(x,y,438,274,81/2) 
           circle(x,y,390,45,81/2)  
           circle(x,y,149.95,830.05,159.9/2) 
           circle(x,y,309.73,830.05,159.9/2) 
           rectangle(x,y,149.95,750.1,309.73,910) 
           rectangle(x,y,438,315,529,315+183) 
           rectangle(x,y,438+91,35+152+78,438+91+183,35+152+78+76) 
           rectangle(x,y,1110-636,35,1110-636+274,35+152)            
           rectangle(x,y,1110-425,0,1110,35)  
           rectangle(x,y,1110-183,35,1110,35+76) 
           rectangle(x,y,1110-117-31-183,35,1110-31-183,35+58) 
           rectangle(x,y,1110-58,1010-313-76-55.5-117-86-67.25-117,1110,1010-313-76-55.5-117-86-67.25) 
           rectangle(x,y,438+91+183+72.5,35+152+80,438+91+183+72.5+152,35+152+80+117) 
           rectangle(x,y,1110-91,1010-313-76-55.5-117-86,1110,1010-313-76-55.5-117) 
           rectangle(x,y,1110-58,1010-313-76-55.5-117,1110,1010-313-76-55.5) 
           rectangle(x,y,1110-366,1010-313-76,1110,1010-313) 
           rectangle(x,y,1110-192-86,1010-183,1110-192,1010) 
           rectangle(x,y,1110-84-43,1010-91,1110-84,1010) 
           
   #cv2.imshow('image',image)
   #cv2.waitKey(1000)


#Fucntion to get the neighbours of any parent node
def fetchCoordinates(parent_x,parent_y,parent_theta):
    global UpLeft1_X,UpLeft1_Y,UpLeft1_theta,UpLeft2_X,UpLeft2_Y,UpLeft2_theta
    global UpRight_X,UpRight_Y,UpRight_theta,DownRight_X,DownRight_Y,DownRight_theta
    global DownLeft1_X,DownLeft1_Y,DownLeft1_theta,DownLeft2_X,DownLeft2_Y,DownLeft2_theta
    global S1_X,S1_Y,S1_theta,S2_X,S2_Y,S2_theta

    def calc_xytheta(left,right):
        dx=((math.pi*wheelDiameter/2)*(left+right)*math.cos(parent_theta)*dt)/60
        dy=((math.pi*wheelDiameter/2)*(left+right)*math.sin(parent_theta)*dt)/60
        dtheta=((math.pi*wheelDiameter/length)*(right-left)*dt)/60
        return dx, dy, dtheta
    
    def update(left,right):
        dx,dy,dtheta=calc_xytheta(left,right)
        X,Y,theta=parent_x+dx,parent_y+dy,parent_theta+dtheta
        return X,Y,theta
  
    UpLeft1_X,UpLeft1_Y,UpLeft1_theta=update(0,RPM1)
    UpLeft2_X,UpLeft2_Y,UpLeft2_theta=update(0,RPM2)
    UpRight_X,UpRight_Y,UpRight_theta=update(RPM1,RPM2)
    DownRight_X,DownRight_Y,DownRight_theta=update(RPM2,RPM1)
    DownLeft1_X,DownLeft1_Y,DownLeft1_theta=update(RPM1,0)
    DownLeft2_X,DownLeft2_Y,DownLeft2_theta=update(RPM2,0)
    S1_X,S1_Y,S1_theta=update(RPM1,RPM1)
    S2_X,S2_Y,S2_theta=update(RPM2,RPM2)

def HeuristicChildGoal(childx,childy):
    dist=np.sqrt(math.pow(childx-goalx,2)+math.pow(childy-goaly,2))
    return dist

def HeuristicChildParent(childx,childy,parentx,parenty):
    dist=np.sqrt(math.pow(childx-parentx,2)+math.pow(childy-parenty,2))
    return dist

def nearNodes(exploreNodes,childx,childy):
    buffer=0.5
    flag_n=1
    for i in range(len(exploreNodes)):
        if flag_n==1:
            if((childx > exploreNodes[i,2]-buffer and childx < exploreNodes[i,2]+buffer) and (childy > exploreNodes[i,3]-buffer and childy < exploreNodes[i,3]+buffer)):
                flag_n=0 
            else:
                flag_n=1
    return flag_n

#Function to update the matrix conataiing all node Info
#The node, cost and the parent of each node
def updatecostnParentNode(exploreNodes,parentx,parenty,goalx,goaly):
    img_array=[]
    def updateMatrix(newcost,exploreNodes,child,flag,childx,childy,child_theta,RPMLeft,RPMRight):
        buffer_n=3
        if (int(childx) in range(0,1110)) and (int(childy) in range(0,1010)) and flag==0:
            print("child",childx,childy)
            if image[int(childy),int(childx),0] == 255:
                print("Expected goal x range",goalx-buffer_n,goalx+buffer_n)
                print("Expected goal y range",goaly-buffer_n,goaly+buffer_n)
                if (nearNodes(exploreNodes,childx,childy)):
                    print(childx,childy,'exists')
                    if((childx > goalx-buffer_n and childx < goalx+buffer_n) and (childy > goaly-buffer_n and childy < goaly+buffer_n)):
                        flag=1
                        print('Found goal')
                    else:
                        #print(newcost,"cost")
                        totcost=newcost+HeuristicChildGoal(childx,childy)
                        newNodes=np.mat([parent,child,childx,childy,child_theta,newcost,totcost,RPMLeft,RPMRight])
                        exploreNodes=np.vstack((exploreNodes,newNodes))
                        image[int(childy),int(childx)]=150
                        child+=1 
                        
        return child,flag,exploreNodes

    count=0
    parent=0
    child=1
    flag=0
    global newcost
    minIndex=0
    parent_theta=0

    while(flag==0 and len(exploreNodes)<500000):
        print("child no",child)
        image[int(parenty),int(parentx)]=200
        fetchCoordinates(parentx,parenty,parent_theta)
        print("Parent:", parentx,parenty)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(UpLeft1_X,UpLeft1_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,UpLeft1_X,UpLeft1_Y,UpLeft1_theta,0,RPM1)        

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(UpLeft2_X,UpLeft2_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,UpLeft2_X,UpLeft2_Y,UpLeft2_theta,0,RPM2)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(UpRight_X,UpRight_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,UpRight_X,UpRight_Y,UpRight_theta,RPM1,RPM2)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(S1_X,S1_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,S1_X,S1_Y,S1_theta,RPM1,RPM1)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(S2_X,S2_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,S2_X,S2_Y,S2_theta,RPM2,RPM2)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(DownRight_X,DownRight_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,DownRight_X,DownRight_Y,DownRight_theta,RPM2,RPM1)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(DownLeft1_X,DownLeft1_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,DownLeft1_X,DownLeft1_Y,DownLeft1_theta,RPM1,0)

        newcost=exploreNodes[minIndex,5]+HeuristicChildParent(DownLeft2_X,DownLeft2_Y,parentx,parenty)
        child,flag,exploreNodes=updateMatrix(newcost,exploreNodes,child,flag,DownLeft2_X,DownLeft2_Y,DownLeft2_theta,RPM2,0)

        exploreNodes[minIndex,6]=np.inf

        minIndex=exploreNodes[:,6].argmin()
        print(minIndex)
        parentx,parenty=exploreNodes[minIndex,2],exploreNodes[minIndex,3]
        parent_theta=exploreNodes[minIndex,4]
        #dist=HeuristicStart
        #newcost=exploreNodes[minIndex,5].astype(float)+dist
        #cv2.imwrite("img%d.jpg"%count,image)
        cv2.imshow('Path trace in progress',image)
        cv2.waitKey(10)
        count+=1
        parent+=1
        #image1=image.copy()
        #img_array.append(image1)

    return exploreNodes,img_array

#def calcxystep()

def drawPath(position1,position2):
    steps=25
    #dt=dt/steps
    x,y=exploreNodes[position1,2],exploreNodes[position1,3]
    theta=exploreNodes[position1,4]
    RPMLeft,RPMRight=exploreNodes[position1,7],exploreNodes[position1,8]
    for i in range(steps):
        x=x+((math.pi*wheelDiameter/2)*(RPMLeft+RPMRight)*math.cos(theta)*(dt/steps))/60
        y=y+((math.pi*wheelDiameter/2)*(RPMLeft+RPMRight)*math.sin(theta)*(dt/steps))/60
        theta=theta+((math.pi*wheelDiameter/length)*(RPMRight-RPMLeft)*(dt/steps))/60
        #image[int(1010-1-y),int(x),:]=[0,0,255]
        image[int(y),int(x),:]=[0,0,255]
        #image[int(y+1),int(x),:]=[0,255,0]
        #image[int(y-1),int(x),:]=[0,255,0]
        #image[int(y),int(x-1),:]=[0,255,0]
        #image[int(y),int(x+1),:]=[0,255,0]
    
def NodePath(exploreNodes):
    node_path=[]
    node_position=exploreNodes.shape[0]-1
    node_path.append(node_position)
    while(node_position!=0):
        print(node_position,"node_position")
        node_position=int(exploreNodes[node_position,0])
        node_path.append(node_position)
    steps=np.zeros((len(node_path),3))
    movement=np.zeros((len(node_path),2))
    for i in range(len(node_path)):
        steps[i,0],steps[i,1],steps[i,2]=exploreNodes[node_path[i],2],exploreNodes[node_path[i],3],exploreNodes[node_path[i],4]
        movement[i,0],movement[i,1]=exploreNodes[node_path[i],7],exploreNodes[node_path[i],8]
        position=node_path[i]
        positionx=exploreNodes[position,2]
        positiony=exploreNodes[position,3]

        drawPath(node_path[i],node_path[i-1])
        
        #image[int(1010-1-positiony),int(positionx)]=[0,0,255]
        image[int(positiony),int(positionx)]=[0,0,255]
        #image1=image.copy()
        #img_array.append(image1)   
    steps=np.flip(steps,0)
    movement=np.flip(movement,0)
    cv2.imwrite("FinalPath.jpg",image)
    return steps,movement

def video(img_array):
    video=cv2.VideoWriter('A*_Rigid.avi',cv2.VideoWriter_fourcc(*'DIVX'), 38,(image_width,image_height))
    #print(len(img_array),"len")
    for i in range(len(img_array)):
        video.write(img_array[i])
    video.release()



#if __name__ == '__main__':


resolution=1
RobotDiameter=35.4
startx=0
starty=0
goalx=149
goaly=249
Clearance=0
robot_radius=RobotDiameter/2
dt=0.25
dtheta=0
RPM1=25
RPM2=15
wheelDiameter=7.6
radius=7.6/2
#left=RPM1
#right=RPM2
length=23
#--------------------------------------------------------------------
#Taking user inputs
print("A* Algorithm - Differentail Robot")
print("Assume the follwoing axis")
print("*--->x")
print("`")
print("`")
print("y")
#----------------------------------------------------------------------
#Create Map
print("Creating image considering turtlebot2 radius")
image_height=1010
image_width=1110
#image=255*np.array(np.ones((image_width,image_height)),dtype=np.uint8)
image=255*np.ones((image_height,image_width,3),dtype=np.uint8) 
TotClear=int(np.round((robot_radius+Clearance),1))
CreateMap()
print("Map created")
cv2.namedWindow('Map_Created',cv2.WINDOW_NORMAL)
cv2.imshow("Map_Created",image)
print("Please press 0 to close the image")
cv2.waitKey(0)

#--------------------------------------------------------------------
#Other user inputs - start
flag_u=0
while(flag_u!=1):
    print("Please enter the start coordinates in the range 0-1010 for X and 0-1110 for Y")
    print("It will be scaled according to the resolution accordingly")
    startx=input("Start node, X coordinate:")
    starty=input("Start node, Y coordinate:")
    try:
        startx=int(startx)
        starty=int(starty)
        if(startx in range(0,image_width) and starty in range(0,image_height)):
            startx=int(startx*resolution)
            starty=int(starty*resolution)
            #starty=int(1010-1-starty)
            if(image[starty,startx][0]==0):
                raise ValueError
            #elif(image[startx,starty][0]+image[startx,starty][1]+image[startx,starty][2])==255:
            #   raise ValueError
        else:
            raise ValueError
        flag_u=1
    except ValueError:
        print("Please enter valid coordinates which do not lie on obstacle")
#-------------------------------------------------------------------------
#Other user inputs - goal
flag_u=0
while(flag_u!=1):
    print("Similarly enter the goal node of the robot:")
    goalx=input("Goal node of the robot, X coordinate:")
    goaly=input("Goal node of the robot, Y coordinate:")
    try:
        goalx=int(goalx)
        goaly=int(goaly)
        if(goalx in range(0,image_width) and goaly in range(0,image_height)):
            goalx=int(goalx*resolution)
            goaly=int(goaly*resolution)
            #goaly=int(1010-1-goaly)
            if(image[goaly,goalx][0]==0):
                raise ValueError
            #elif (image[goalx,goaly][0]+image[goalx,goaly][1]+image[goalx,goaly][2])==255:
            #    raise ValueError
        else:
            raise ValueError
        flag_u=1
    except ValueError:
        print("Please enter valid coordinates which do not lie on obstacle")
#-------------------------------------------------------------------------

# A* algorithm initializations
parentx=startx
parenty=starty
#goaly=1010-1-goaly
#image[int(parentx),int(parenty)]=175
#image[int(goalx),int(goaly)]=175
#exploreNodes=[None]*np.empty([1000000,9])  
#RPMmatrix=[None]*np.empty([1000000,2])  

exploreNodes=np.mat([0,0,parentx,parenty,0,0,0,0,0]).astype(float)
RPMmatrix=np.mat([0,0])

#-------------------------------------------------------------------
# The A* algorithm
exploreNodes,img_array=updatecostnParentNode(exploreNodes,parentx,parenty,goalx,goaly)
steps,movement=NodePath(exploreNodes)
#video(img_array)
print("Steps",steps)
print("Movement",movement)