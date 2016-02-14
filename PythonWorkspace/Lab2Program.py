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
        lidarValues = [
            ray_trace(paths, m, n, fr,
                lambda m, n, count: (m, n+count)),
            ray_trace(paths, m, n, fr,
                lambda m, n, count: (m-count, n)),
            ray_trace(paths, m, n, fr,
                lambda m, n, count: (m, n-count)),
            ray_trace(paths, m, n, fr,
                lambda m, n, count: (m+count, n))
        ]

    else:
        print("No support for more than 4 lidar directions for now")

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

    # northTheta = -np.pi / 2
    # # east
    # assert (mapTheta2worldTheta(0, northTheta) == -np.pi / 2)
    # # north
    # assert (mapTheta2worldTheta(1*np.pi/2, northTheta) == 0)
    # # west
    # assert (mapTheta2worldTheta(2*np.pi/2, northTheta) == np.pi / 2)
    # # south
    # assert (mapTheta2worldTheta(3*np.pi/2, northTheta) == -np.pi)

test_mapTheta2worldTheta()

def best_theta_map(paths, pathTowardsGoal, m, n, fr, original_four):
    if original_four:
        cardinal_dir = pseudoLidarSensor(paths, m, n, fr, original_four)
        theta_map = np.pi / 2 * np.argmax(cardinal_dir)
    else:
        northeast_idx = (np.arange(m-1,m-fr-1,-1), np.arange(n,n+fr))
        northwest_idx = (np.arange(m,m+fr)       , np.arange(n,n+fr))
        southwest_idx = (np.arange(m,m+fr)       , np.arange(n-1,n-fr-1,-1))
        southeast_idx = (np.arange(m-1,m-fr-1,-1), np.arange(n-1,n-fr-1,-1))
        east      = np.sum(pathTowardsGoal[m        ,n   :n+fr])
        northeast = np.sum(pathTowardsGoal[northeast_idx])
        north     = np.sum(pathTowardsGoal[m-fr:m   ,n  ])
        northwest = np.sum(pathTowardsGoal[northwest_idx])
        west      = np.sum(pathTowardsGoal[m,n-fr:n   ])
        southwest = np.sum(pathTowardsGoal[southwest_idx])
        south     = np.sum(pathTowardsGoal[m   :m+fr,n  ])
        southeast = np.sum(pathTowardsGoal[southeast_idx])
        cardinal_dir = np.array([east, northeast, north, northwest, west, southwest, south, southeast]) # matching unit circle
        theta_map = np.pi / 4 * np.argmax(cardinal_dir)
    print cardinal_dir
    return theta_map


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
    proxSens = prox_sens_initialize(clientID)
    maxVel=120*np.pi/180
    velLeft=0
    velRight=0
    # initialize odom of ePuck
    # FIXME: xyz is not a pose. xytheta is.
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
        goal_m, goal_n = (58,48)

        walls = im[:,:,0] > 0.25
        no_doors = im[:,:,1] * walls > 0.25
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
        pathTowardsGoal = (paths * 255) - distance_from_goal

        window_size = 21           # odd
        fr = (window_size - 1) / 2 # fire range
        lidarValues = pseudoLidarSensor(paths, m, n, fr, original_four=True)
        print "lidarValues =", lidarValues

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
                return k_repulse*(1.0/rho - 1.0/rho_0)*(1.0/rho**2)
            else:
                return 0
        # repulsionVectors = [np.array(pol2cart(k_repulse/(val - 0.75)**2, angle)) for val, angle in zip(lidarValues, lidarAngles)]
        repulsionVectors = np.vstack([np.array(pol2cart(force_repulsion(k_repulse, val, 7), angle)) for val, angle in zip(lidarValues, lidarAngles)])
        attractionVal, attractionAngle = cart2pol(
            goal_n - n, # cols counts same     to normal horz axes
            m - goal_m  # rows counts opposite to normal vert axes
        )
        # Objects attraction should be inverse proportional to distance
        # Small Distances should be very high attraction
        k_attract = 50.0
        attractionVector = np.array(pol2cart(k_attract/attractionVal, attractionAngle))

        print np.vstack((repulsionVectors, attractionVector))
        finalVector = np.sum(np.vstack((repulsionVectors, attractionVector)),   axis=0)
        # finalVector = np.sum(np.vstack(repulsionVectors),   axis=0)
        print "finalVector: ", finalVector
        finalValue, finalAngle = cart2pol(finalVector[0], finalVector[1])
        print "finalVal, finalAngle: ", (finalValue, finalAngle*180/np.pi)

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
        forward_vel = 0.5

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