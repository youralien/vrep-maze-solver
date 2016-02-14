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
import skimage.transform
import skimage.filters
from scipy.spatial.distance import cityblock

# interactive plotting
plt.ion()

def format_vrep_image(resolution, image):
    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1],resolution[0],3])
    return im

def image_vert_flip(im):
    return im[::-1,:,:]

def prox_sens_initialize(clientID):
    """ Initialize proximity sensor. Maybe helpful later """
    proxSens=[]
    for i in range(8):
        _, oneProxSens = vrep.simxGetObjectHandle(clientID, 'ePuck_proxSensor%d' % (i+1), vrep.simx_opmode_streaming)
        proxSens.append(oneProxSens)
    return proxSens

def prox_sens_read(clientID, proxSens):
    """ Read the proximity sensor
    clientID: vrep clientID
    proxSens: array of proxSensor handles
    """
    outputs = []
    keys = ('returnCode','detectionState','detectedPoint','detectedObjectHandle','detectedSurfaceNormalVector')
    # NOTE: take norm of deteected point to get the distance
    for i in range(8):
        proxOut=vrep.simxReadProximitySensor(clientID, proxSens[i], vrep.simx_opmode_streaming)
        outputs.append(dict(zip(keys, proxOut)))
    return outputs

def odom2pixelmap(x, y, odom_range, map_side_length):
    """ Converts xyz in odom to the estimated pixel in the map.
    In odom, (0, 0, 0) is in the middle of the pixelmap

    Parameters
    ----------
    odom_range: the max magnitude of the odom vector along one of the axis
    map_side_length: the length of one side of the map

    Returns
    -------
    (m, n) coordinates of the pixels
    """

    # flip odom to have 0 be top left hand corner (currently bottom left)
    y *= -1

    # transform odom to no negative coordinates
    x += odom_range
    y += odom_range

    # scale from odom coords to the map coords
    odom_side_length = odom_range * 2
    scaling_factor = (float(map_side_length) / odom_side_length)
    x *= scaling_factor
    y *= scaling_factor

    return int(round(y)), int(round(x))

def vomega2bytecodes(v, omega, g, L=0.216):
    """
    Turns v and omega into control codes for differential drive robot
    v: forward velocity
    omega: angular velocity
    g: gain constant
    L: length of axle, default .216
        >>> right = -1.9914
        >>> left = -2.2074
        >>> right - left
        0.21599999999999975
    """
    v_comm = v
    v_diff = omega * L / 2.0
    ctrl_sig_left = (v_comm - v_diff) / float(g)
    ctrl_sig_right = (v_comm + v_diff) / float(g)
    return ctrl_sig_left, ctrl_sig_right

class ThetaRange(object):
    @staticmethod
    def normalize_angle_pos(angle):
        return ((angle % (2*np.pi)) + 2*np.pi) % (2*np.pi)

    @staticmethod
    def normalize_angle(angle):
        """ Constrains the angles to the range [0, pi) U [-pi, 0) """
        a = ThetaRange.normalize_angle_pos(angle)
        if a >= np.pi:
            a -= 2*np.pi
        return a

def ray_trace(paths, m, n, fr, compute_next_cell):
    """
    paths: a map of binary values, where True denotes pixels where robot can go (not walls)
    m: current position, row pixel
    n: current position, col pixel
    fr: firing range of your ray trace
    compute_next_cell: a function handle to how to compute
    """
    hit_wall = False
    count = 1
    while (not hit_wall and count <= fr):
        next_cell = compute_next_cell(m,n,count)
        if paths[next_cell] == False:
            hit_wall = True
        else:
            count += 1
    return count

def pseudoLidarSensor(paths, m, n, fr, original_four):

    if original_four:
        functions = [
            lambda m, n, count: (m, n+count),
            lambda m, n, count: (m-count, n),
            lambda m, n, count: (m, n-count),
            lambda m, n, count: (m+count, n)
        ]
        lidarValues = [
            ray_trace(paths, m, n, fr, func)
            for func in functions
        ]

    else:
        functions = [
            lambda m, n, count: (m, n+count),
            lambda m, n, count: (m-count,n+count),
            lambda m, n, count: (m-count, n),
            lambda m, n, count: (m-count,n-count),
            lambda m, n, count: (m, n-count),
            lambda m, n, count: (m+count,n-count),
            lambda m, n, count: (m+count, n),
            lambda m, n, count: (m+count,n+count)
        ]
        lidarValues = [
            ray_trace(paths, m, n, fr, func)
            for func in functions
        ]
        # but the diagnols should have sqrt(2) more weight
        for i in range(len(lidarValues)):
            if i % 2 == 1:
                lidarValues[i] *= np.sqrt(2)

    return lidarValues

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    """ where rho is the Radius, and phi is the angle """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def mapTheta2worldTheta(mapTheta, northTheta):
    """ converts the mapTheta where

    mapTheta = 0 is "east"
    mapTheta = pi / 2 is "north"
    and mapTheta defined on [0, 2*np.pi)

    to the worldTheta, where
    "north" is given by northTheta
    and worldTheta defined on [0, pi) U [-pi, 0) """

    # to convert mapTheta pi/2 which is north, to northTheta, we have to do add a bias
    bias = northTheta - np.pi / 2

    worldTheta = ThetaRange.normalize_angle(mapTheta + bias)
    return worldTheta

def test_mapTheta2worldTheta():
    """ if north Theta given by 0 ..."""
    northTheta = 0
    # east
    assert (mapTheta2worldTheta(0, northTheta) == -np.pi / 2)
    # north
    assert (mapTheta2worldTheta(1*np.pi/2, northTheta) == 0)
    # west
    assert (mapTheta2worldTheta(2*np.pi/2, northTheta) == np.pi / 2)
    # south
    assert (mapTheta2worldTheta(3*np.pi/2, northTheta) == -np.pi)

test_mapTheta2worldTheta()

def robot_code(clientID, verbose=False):
    # initialize ePuck handles and variables
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
    # proxSens = prox_sens_initialize(clientID)
    # initialize odom of ePuck
    _, xyz = vrep.simxGetObjectPosition(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)
    _, eulerAngles = vrep.simxGetObjectOrientation(
        clientID, ePuck, -1, vrep.simx_opmode_streaming)
    # NOTE: this assumes that simulation and this code is started roughly the same time

    # initialize overhead cam
    _, overheadCam=vrep.simxGetObjectHandle(
        clientID, 'Global_Vision', vrep.simx_opmode_oneshot_wait)
    _, resolution, image = vrep.simxGetVisionSensorImage(
        clientID,overheadCam,0,vrep.simx_opmode_oneshot_wait)

    # initialize goal handle + odom
    _, goal=vrep.simxGetObjectHandle(
        clientID, 'Goal_Position', vrep.simx_opmode_oneshot_wait)
    _, goalPose = vrep.simxGetObjectPosition(
        clientID, goal, -1, vrep.simx_opmode_streaming)

    # STOP
    _ = vrep.simxSetJointTargetVelocity(
        clientID,leftMotor,0,vrep.simx_opmode_oneshot_wait)
    _ = vrep.simxSetJointTargetVelocity(
        clientID,rightMotor,0,vrep.simx_opmode_oneshot_wait)

    worldNorthTheta = None

    t = time.time()
    while (time.time() - t) < 100:
        if verbose: print "Time Elapsed = ", time.time() - t;

        # global map
        _,resolution,image = vrep.simxGetVisionSensorImage(clientID,overheadCam,0,vrep.simx_opmode_oneshot_wait) # Get image
        im = format_vrep_image(resolution, image) # original image
        im = image_vert_flip(im)
        resize_length = int(im.shape[0]/2)
        im = skimage.transform.resize(im, (resize_length, resize_length, 3))

        _, xyz = vrep.simxGetObjectPosition(
            clientID, ePuck, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            clientID, ePuck, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]

        # initialize worldNorthTheta for the first time
        if worldNorthTheta is None:
            worldNorthTheta = theta

        map_side_length = 2.55
        m, n = odom2pixelmap(x, y, map_side_length, im.shape[0])
        # acquire pixel location of goal
        _, goalPose = vrep.simxGetObjectPosition(
            clientID, goal, -1, vrep.simx_opmode_buffer)
        goal_m, goal_n = odom2pixelmap(goalPose[0], goalPose[1], map_side_length, im.shape[0])
        # FIXME: temp goal
        goal_m, goal_n = (40,6)

        walls = im[:,:,0] > 0.25
        # no_doors = im[:,:,1] * walls > 0.25
        blurred_map = skimage.filters.gaussian_filter(walls, sigma=2)
        paths = blurred_map < 0.15

        # what we are doing here is creating a map of pixels which has values
        # (255 - (distance_from_goal)
        # which would be useful in an A* search using (manhattan) distance as a heuristic
        distance_from_goal = np.zeros((im.shape[0], im.shape[1]))
        goal_pixel_vector = np.array([goal_m, goal_n])
        for row in range(im.shape[0]):
            for col in range(im.shape[1]):
                current_pixel_vector = np.array([row, col])
                distance_from_goal[row,col] = cityblock(goal_pixel_vector, current_pixel_vector)

        window_size = 21           # odd
        fr = (window_size - 1) / 2 # fire range
        lidarValues = pseudoLidarSensor(paths, m, n, fr, original_four=False)
        # print "lidarValues =", np.array(lidarValues).reshape(-1,1)

        #############################
        # Potential Field Algorithm #
        #############################
        numLidarValues = len(lidarValues)
        lidarAngles = [np.pi / numLidarValues * index for index in range(numLidarValues)]

        # Objects repulsion should be inverse of distance
        # Small Distances should be very high repulsion
        k_repulse = 1000.0
        def force_repulsion(k_repulse, rho, rho_0):
            """
            k_repulse: positive scaling factor
            rho: distance from point to obstacle
            rho_0: distance of influence
            """
            if rho <= rho_0:
                return -1.0 * k_repulse*(1.0/rho - 1.0/rho_0)*(1.0/rho**2)
            else:
                return 0
        # repulsionVectors = [np.array(pol2cart(k_repulse/(val - 0.75)**2, angle)) for val, angle in zip(lidarValues, lidarAngles)]
        repulsionVectors = np.vstack([np.array(pol2cart(force_repulsion(k_repulse, val, 3), angle)) for val, angle in zip(lidarValues, lidarAngles)])
        attractionVal, attractionAngle = cart2pol(
            goal_n - n, # cols counts same     to normal horz axes
            m - goal_m  # rows counts opposite to normal vert axes
        )
        # Objects attraction should be inverse proportional to distance
        # Small Distances should be very high attraction
        k_attract = 1000.0
        attractionVector = np.array(pol2cart(k_attract/attractionVal, attractionAngle))

        # print np.vstack((repulsionVectors, attractionVector))
        finalVector = np.sum(np.vstack((repulsionVectors, attractionVector)),   axis=0)
        # finalVector = np.sum(np.vstack(repulsionVectors),   axis=0)
        print "finalVector: ", finalVector
        finalValue, finalAngle = cart2pol(finalVector[0], finalVector[1])
        # print "finalVal, finalAngle: ", (finalValue, finalAngle*180/np.pi)

        # proportional control on angular velocity
        k_angular = 0.25
        # if you are at theta0, and you want to move to theta1, there is two cases
        # case1: theta1 > theta0
        # you want to turn left, which is equivalent to omega > 0
        # so, omega = theta1 - theta0 > 0
        # and vice versa
        delta_theta = mapTheta2worldTheta(finalAngle, worldNorthTheta) - theta
        omega = k_angular * delta_theta
        print "Omega: ", round(omega,1)

        g = 1
        print "distance_from_goal: ", distance_from_goal[m,n]
        if distance_from_goal[m,n] == 1:
            forward_vel = 0
        else:
            forward_vel = 1.0

        # if np.abs(omega - 0) < 0.2:
        #     forward_vel = 0.5
        # else:
        #     forward_vel = 0.0
        #     omega *= 5
        # control the motors
        ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g)
        _ = vrep.simxSetJointTargetVelocity(
            clientID,leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
        _ = vrep.simxSetJointTargetVelocity(
            clientID,rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity

        # plt.imshow(walls, cmap='gray')
        # plt.imshow(no_doors)
        # plt.imshow(blurred_map)
        # plt.imshow(paths)
        im[goal_m,goal_n] = np.array((1.0, 1.0, 1.0))
        im[m,n,:] = np.array((255.0/255.0,192/255.0,203/255.0))
        plt.imshow(im)
        plt.pause(0.1)
        time.sleep(0.05) #sleep 50ms

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
        try:
            robot_code(clientID)
        except IndexError, e:
            pass

    vrep.simxFinish(clientID)
    print 'Program ended'

if __name__ == '__main__':
    main()