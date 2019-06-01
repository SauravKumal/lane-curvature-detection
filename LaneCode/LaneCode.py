import vrep
import sys
import time
import numpy as np
import cv2
import imutils
import math
import matplotlib.pyplot as plt

vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Get the client ID

if clientID!=-1:  #check if client connection successful
	print('Connected to remote API server')
else:
	print('Connection not successful')
	sys.exit('Could not connect')
	
errorCode, cameraHandle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
vrep.simxGetVisionSensorImage(clientID, cameraHandle, 0, vrep.simx_opmode_streaming)
errorCode, leftMotorHandle = vrep.simxGetObjectHandle(clientID,'dr20_leftWheelJoint_',vrep.simx_opmode_oneshot_wait)
errorCode, rightMotorHandle = vrep.simxGetObjectHandle(clientID,'dr20_rightWheelJoint_',vrep.simx_opmode_oneshot_wait)
print('Setting up the camera system...')
lastFrame = None;
err = 0;
	
while(err != 1):
	err, lastFrame = get_image(clientID, cameraHandle)
print('Camera setup successful.')

while True:
	err, img = get_image(clientID, cameraHandle)
	#--------process image--------------------------
	transformedImage = cv2.flip(img,0)
	transformedImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2RGB)
	hsvImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2HSV)
	orgImage = np.copy(transformedImage)
	grayImage = cv2.cvtColor(transformedImage,cv2.COLOR_BGR2GRAY)
	lowerYellow = np.array([20,100,100],dtype="uint8")
	upperYellow = np.array([30,255,255],dtype="uint8")
	yellowMask = cv2.inRange(hsvImage,lowerYellow,upperYellow)
	yellowMaskedImage = cv2.bitwise_and(grayImage,yellowMask)
	blurredImage = cv2.GaussianBlur(yellowMaskedImage,(9,9),1)
	cannyImage = cv2.Canny(blurredImage,50,150)
	#-------region of interest----------------------
	polygon = np.array([(0,511),(0,330),(511,330),(511,511)],np.int32)
	mask = np.zeros_like(cannyImage)
	cv2.fillPoly(mask,[polygon],255)
	maskedImage = cv2.bitwise_and(cannyImage,mask)
	#----------find lines--------------------------
	lines = cv2.HoughLinesP(maskedImage, 2, np.pi/180,100,np.array([]),minLineLength=4, maxLineGap=5)
	#-----------get average lines for the two lanes --------------------
	leftLinesParam =[]
	rightLinesParam =[]
	leftLines =[]
	rightLines =[]
	leftLane=[]
	rightLane=[]
	if lines is not None:
		for line in lines:
			x1,y1,x2,y2 =line.reshape(4)
			slope,intercept = 0,0
			if (x2-x1)!=0:
				slope = (y2-y1)/(x2-x1)
				intercept = y2 - slope * x2
				if slope<0:
					leftLinesParam.append((slope,intercept))
					leftLines.append((line))
				elif slope >0:
					rightLinesParam.append((slope,intercept))
					rightLines.append((line))
				else:
					pass
			else:
					pass
					
		maxPower = 5
		kp=3
		if len(leftLinesParam)!=0 and len(rightLinesParam) !=0:
			avgLeftLineParam = np.average(leftLinesParam,axis=0)
			avgRightLineParam = np.average(rightLinesParam,axis=0)
			leftLane = make_coordinates(orgImage, avgLeftLineParam)
			rightLane = make_coordinates(orgImage, avgRightLineParam)
			avgSlope = avgLeftLineParam[0] + avgRightLineParam[0]
			
			if avgSlope <-0.34:
				print(avgSlope,"Moving right")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,abs(avgSlope*kp)+maxPower, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,maxPower, vrep.simx_opmode_oneshot)
			elif avgSlope >0.34:
				print(avgSlope,"Moving left")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,maxPower, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,abs(avgSlope*kp)+maxPower, vrep.simx_opmode_oneshot)
			else:
				print("Moving forward")
				errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,maxPower, vrep.simx_opmode_oneshot)
				errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,maxPower, vrep.simx_opmode_oneshot)
				
			linesImage = display_lines(orgImage,[leftLane,rightLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)

		elif len(leftLinesParam)!=0 and len(rightLinesParam) ==0:
			print(slope,"Moving right")
			avgLeftLineParam = np.average(leftLinesParam,axis=0)
			leftLane = make_coordinates(orgImage, avgLeftLineParam)
			slope = avgLeftLineParam[0]
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,abs(slope*kp)+maxPower, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,maxPower, vrep.simx_opmode_oneshot)
			linesImage = display_lines(orgImage,[leftLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)

		elif len(leftLinesParam)==0 and len(rightLinesParam) !=0:
			print(slope,"Moving left")
			avgRightLineParam = np.average(rightLinesParam,axis=0)
			rightLane = make_coordinates(orgImage, avgRightLineParam)
			slope = avgRightLineParam[0]
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,maxPower, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,abs(slope*kp)+maxPower, vrep.simx_opmode_oneshot)
			linesImage = display_lines(orgImage,[rightLane])
			tst = cv2.addWeighted(orgImage,0.8,linesImage,1,1)
			cv2.imshow('view',tst)
			cv2.waitKey(1)
			
		else:
			print("stopped no line lane found")
			errorCode=vrep.simxSetJointTargetVelocity(clientID,leftMotorHandle,0, vrep.simx_opmode_oneshot)
			errorCode=vrep.simxSetJointTargetVelocity(clientID,rightMotorHandle,0, vrep.simx_opmode_oneshot)