# -*- coding: utf-8 -*-
"""
Name: Ryan Louie
Student No.: A0149643X
Date: Feb 11th 2016
Version: 0.0.1
"""
#Import Libraries:
import vrep #import library for VREP API
import time
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting


def format_vrep_image(resolution, image):
    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1],resolution[0],3])
    return im

def threshold_for_red(im):
    RED_CHANNEL = 0
    thresh_im = im[:,:,RED_CHANNEL] > 150 # where the red channel is pretty red
    return thresh_im

def find_object_center(binary_image):
    """
    Parameters
    ----------
    binary_image: numpy array, shape (m,n)

    Returns
    -------
    center_x, center_y: the location of the center as integer.  If None, then it didn't find the center
    """
    if np.any(binary_image):
        height, width = binary_image.shape

        # horizontal histogram has counts of white pixels in a column, where indexes represent columns
        horzhist = np.zeros(width)
        for col in range(width):
            horzhist[col] = np.sum(binary_image[:,col])
        # get column which had the most white pixels (that would be the center of a ball)
        center_x = np.argmax(horzhist)

        verthist = np.zeros(height)
        for row in range(height):
            verthist[row] = np.sum(binary_image[row, :])
        # get row which had the most white pixels (that would be the center of a ball)
        center_y = np.argmax(verthist)

        return center_x, center_y
    else:
        return None, None

def find_how_close(binary_image):
    return float(np.sum(binary_image)) / np.dot(*binary_image.shape)

def calculateWheelVelocities(how_much_to_turn, how_close = 0, k_turn=0.025, k_forward=0.5):
    """ calculates right and left wheel velocities given how much to turn (ball horizontal position)
    and how fast to go (how close is the ball?)

    Parameters
    ----------
    how_much_to_turn:
        if large and positive, turn right sharply
        if large and negative, turn left sharply

    how_close:
        if small, go faster and catch up to ball
        if large, go slower because the ball is pretty close

    Returns: vl, vr
    """
    how_fast_to_go = 1.0 / how_close
    vl = float(k_turn) * how_much_to_turn + float(k_forward) * how_fast_to_go
    vr = - float(k_turn) * how_much_to_turn + float(k_forward) * how_fast_to_go
    return vl, vr

def testWheelVel():
    norm_center_xs = np.linspace(0, 1, 10)
    turns = norm_center_xs - 0.5
    print turns
    for how_much_to_turn in turns:
        how_fast_to_go = 100
        vl, vr = calculateWheelVelocities(
            how_much_to_turn, how_fast_to_go,
            k_turn=100, k_forward=0.3)
        print vl, vr

def robot_code(clientID):
    # initialize handles and variables
    _, bodyElements=vrep.simxGetObjectHandle(
        clientID, 'ePuck_bodyElements', vrep.simx_opmode_oneshot_wait)
    _, leftMotor=vrep.simxGetObjectHandle(
        clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
    _, rightMotor=vrep.simxGetObjectHandle(
        clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)
    _, ePuck=vrep.simxGetObjectHandle(
        clientID, 'ePuck', vrep.simx_opmode_oneshot_wait)
    _, ePuckBase=vrep.simxGetObjectHandle(
        clientID, 'ePuck_base', vrep.simx_opmode_oneshot_wait)
    proxSens=[]
    for i in range(8):
        _, oneProxSens = vrep.simxGetObjectHandle(clientID, 'ePuck_proxSensor %d' % i, vrep.simx_opmode_oneshot_wait)
        proxSens.append(oneProxSens)
    maxVel=120*np.pi/180
    velLeft=0
    velRight=0

    # STOP
    ErrorCode = vrep.simxSetJointTargetVelocity(
        clientID,leftMotor,0,vrep.simx_opmode_oneshot_wait)
    ErrorCode = vrep.simxSetJointTargetVelocity(
        clientID,rightMotor,0,vrep.simx_opmode_oneshot_wait)

    t = time.time()
    while (time.time() - t) < 100:
        print "Time Elapsed = ", time.time() - t

"""
    #initialize variables
    ErrorCode = 0
    LeftVelocity = 150
    RightVelocity = 100

    #Get motor handles
    ErrorCode, LeftJointHandle = vrep.simxGetObjectHandle(clientID,'dr12_leftJoint_',vrep.simx_opmode_oneshot_wait)  #Left motor
    ErrorCode, RightJointHandle = vrep.simxGetObjectHandle(clientID,'dr12_rightJoint_',vrep.simx_opmode_oneshot_wait) #Right motor

    #Get bumper handles
    ErrorCode, BumperSensorHandle = vrep.simxGetObjectHandle(clientID,'dr12_bumperForceSensor_',vrep.simx_opmode_oneshot_wait) #Bumper
    ErrorCode, state, forceVector, torqueVector = vrep.simxReadForceSensor(clientID, BumperSensorHandle,vrep.simx_opmode_streaming) #Get bumper force reading, first run
    ErrorCode, state, forceVector, torqueVector = vrep.simxReadForceSensor(clientID, BumperSensorHandle,vrep.simx_opmode_streaming) #Get bumper force reading, first run

    #Get camera handles
    ErrorCode, CamHandle = vrep.simxGetObjectHandle(clientID,'VisionSensor',vrep.simx_opmode_oneshot_wait); #Camera
    ErrorCode, resolution, image = vrep.simxGetVisionSensorImage(clientID,CamHandle,0,vrep.simx_opmode_oneshot_wait); #Get image, first run

    # Plotting
    fig, axes = plt.subplots(2, 1)

    t = time.time()
    #vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot) #Pause simulation, allow longer computation time without simulation running
    #vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot) #Resume simulation
    while (time.time()-t)<1000:
        remaining = time.time()-t
        # print "Time Remaining=", remaining
        ErrorCode,resolution,image = vrep.simxGetVisionSensorImage(clientID,CamHandle,0,vrep.simx_opmode_oneshot_wait) # Get image
        ErrorCode, state, forceVector, torqueVector = vrep.simxReadForceSensor(clientID, BumperSensorHandle,vrep.simx_opmode_buffer) # Get bumper force reading

        # Processing Image
        im = format_vrep_image(resolution, image) # original image
        thresh_im = threshold_for_red(im) # threhold binary image
        center_x, center_y = find_object_center(thresh_im)

        if center_x and center_y:
            norm_center_x = float(center_x) / resolution[0]

            # P control: The normalized center of screen is 0.5
            how_much_to_turn = (norm_center_x - 0.5)

            percent_of_picture_is_ball = find_how_close(thresh_im)

            if percent_of_picture_is_ball > 0.1:
                vl, vr = calculateWheelVelocities(
                    how_much_to_turn, percent_of_picture_is_ball,
                    k_turn=100, k_forward=0)
            else:
                vl, vr = calculateWheelVelocities(
                    how_much_to_turn, percent_of_picture_is_ball,
                    k_turn=100, k_forward=5)
        else:
            vl, vr = 0,0

        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,vl*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed
        ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,vr*math.pi/180,vrep.simx_opmode_oneshot_wait) #set Left wheel speed

        #Set Drive to robot
        time.sleep(0.05) #sleep 50ms
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,LeftJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
    ErrorCode = vrep.simxSetJointTargetVelocity(clientID,RightJointHandle,0,vrep.simx_opmode_oneshot_wait) #stop motor
"""

def main():
    #Initialisation for Python to connect to VREP
    print 'Python program started'
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #Timeout=5000ms, Threadcycle=5ms
    if clientID!=-1:
        print 'Connected to V-REP'
    else:
        print 'Failed connecting to V-REP'
        vrep.simxFinish(clientID)


    if clientID!=-1:

        robot_code(clientID)

    vrep.simxFinish(clientID)
    print 'Program ended'

if __name__ == '__main__':
    main()