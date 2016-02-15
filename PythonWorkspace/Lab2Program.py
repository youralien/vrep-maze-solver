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
import math
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

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = ThetaRange.normalize_angle(a)
    b = ThetaRange.normalize_angle(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

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

    northTheta = -np.pi / 2
    # east
    assert (mapTheta2worldTheta(0, northTheta) == -np.pi)
    # north
    assert (mapTheta2worldTheta(1*np.pi/2, northTheta) == -np.pi / 2)
    # west
    assert (mapTheta2worldTheta(2*np.pi/2, northTheta) == 0)
    # south
    assert (mapTheta2worldTheta(3*np.pi/2, northTheta) == np.pi / 2)

# test_mapTheta2worldTheta()

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

def test_force_repulsion():
    k_repulse = 10.0
    for i, acting_dist in enumerate(range(2,7)):
        y = [force_repulsion(k_repulse, val, acting_dist) for val in np.arange(1,7)]
        plt.subplot(4,1,i)
        plt.plot(y)

    plt.show()
    raw_input("")

# test_force_repulsion()

class Lab2Program:

    def __init__(self):
        self.initialize_vrep_client()
        self.initilize_vrep_api()
        self.define_constants()

    def initialize_vrep_client(self):
        #Initialisation for Python to connect to VREP
        print 'Python program started'
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #Timeout=5000ms, Threadcycle=5ms
        if self.clientID!=-1:
            print 'Connected to V-REP'
        else:
            print 'Failed connecting to V-REP'
            vrep.simxFinish(self.clientID)

    def initilize_vrep_api(self):
        # initialize ePuck handles and variables
        _, self.bodyelements=vrep.simxGetObjectHandle(
            self.clientID, 'ePuck_bodyElements', vrep.simx_opmode_oneshot_wait)
        _, self.leftMotor=vrep.simxGetObjectHandle(
            self.clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
        _, self.rightMotor=vrep.simxGetObjectHandle(
            self.clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)
        _, self.ePuck=vrep.simxGetObjectHandle(
            self.clientID, 'ePuck', vrep.simx_opmode_oneshot_wait)
        _, self.ePuckBase=vrep.simxGetObjectHandle(
            self.clientID, 'ePuck_base', vrep.simx_opmode_oneshot_wait)
        # proxSens = prox_sens_initialize(self.clientID)
        # initialize odom of ePuck
        _, self.xyz = vrep.simxGetObjectPosition(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_streaming)
        _, self.eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_streaming)

        # initialize overhead cam
        _, self.overheadCam=vrep.simxGetObjectHandle(
            self.clientID, 'Global_Vision', vrep.simx_opmode_oneshot_wait)
        _, self.resolution, self.image = vrep.simxGetVisionSensorImage(
            self.clientID,self.overheadCam,0,vrep.simx_opmode_oneshot_wait)

        # initialize goal handle + odom
        _, self.goalHandle=vrep.simxGetObjectHandle(
            self.clientID, 'Goal_Position', vrep.simx_opmode_oneshot_wait)
        _, self.goalPose = vrep.simxGetObjectPosition(
            self.clientID, self.goalHandle, -1, vrep.simx_opmode_streaming)

        # STOP Motor Velocities.  Not sure if this is necessary
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.leftMotor,0,vrep.simx_opmode_oneshot_wait)
        _ = vrep.simxSetJointTargetVelocity(
            self.clientID,self.rightMotor,0,vrep.simx_opmode_oneshot_wait)

    def define_constants(self):
        self.map_side_length = 2.55
        # FIMXE: hard coded goals
        self.GOALS = [(40+2,6), (40, 6+2), (40,21), (30,22), (29,10), (27,5), (20,8), (20,33), (20, 48), (5,55)]
        self.worldNorthTheta = None

    def global_map_preprocess(self, resolution, image):
        im = format_vrep_image(resolution, image) # original image
        im = image_vert_flip(im)
        resize_length = int(im.shape[0]/2)
        im = skimage.transform.resize(im, (resize_length, resize_length, 3))
        return im

    def global_map_process(self, im):
        walls = im[:,:,0] > 0.25
        # no_doors = im[:,:,1] * walls > 0.25
        # blurred_map = skimage.filters.gaussian_filter(walls, sigma=2)
        paths = walls < 0.15
        return paths

    def robot_pose_get(self):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]

        return (x, y, theta)

    def lidar_scan_get(self, window_size=21):
        """ gets lidar scan range values """
        fr = (window_size - 1) / 2 # fire range
        lidarValues = pseudoLidarSensor(self.paths, self.pose[0], self.pose[1], fr, original_four=False)
        return lidarValues

    def repulsion_vectors_compute(self, lidarValues, k_repulse=10.0):
        numLidarValues = len(lidarValues)
        lidarAngles = [np.pi / numLidarValues * index for index in range(numLidarValues)]

        repulsionVectors = [np.array(pol2cart(force_repulsion(k_repulse, np.sqrt(val), 2), angle)) for val, angle in zip(lidarValues, lidarAngles)]

        return repulsionVectors

    def attraction_vector_compute(self, k_attract=100.0):
        self.attractionVal, self.attractionAngle = cart2pol(
            self.goal_pose_pixel[1] - self.pose_pixel[1], # cols counts same     to normal horz axes
            self.pose_pixel[0] - self.goal_pose_pixel[0]  # rows counts opposite to normal vert axes
        )
        attractionVector = np.array(pol2cart(k_attract*self.attractionVal, self.attractionAngle))
        return attractionVector

    def robot_code(self):
        t = time.time()

        self.curr_goal = 0
        while (time.time() - t) < 200:

            # global map
            _,resolution,image = vrep.simxGetVisionSensorImage(self.clientID,self.overheadCam,0,vrep.simx_opmode_oneshot_wait) # Get image
            im = self.global_map_preprocess(resolution, image)

            self.pose = self.robot_pose_get()
            x, y, theta = self.pose

            # initialize worldNorthTheta for the first time
            if self.worldNorthTheta is None:
                self.worldNorthTheta = self.pose[2]

            self.pose_pixel = np.array(odom2pixelmap(x, y, self.map_side_length, im.shape[0]))
            m, n = self.pose_pixel

            self.goal_pose_pixel = np.array(self.GOALS[self.curr_goal])
            goal_m, goal_n = self.goal_pose_pixel

            self.paths = self.global_map_process(im)
            lidarValues = self.lidar_scan_get(window_size=21)


            #############################
            # Potential Field Algorithm #
            #############################
            repulsionVectors = self.repulsion_vectors_compute(lidarValues, k_repulse=10.0)
            attractionVector = self.attraction_vector_compute(k_attract=100.0)

            finalVector = np.sum(np.vstack((repulsionVectors, attractionVector)), axis=0)
            finalUnitVector = finalVector / np.linalg.norm(finalVector)
            print "finalUnitVector: ", finalUnitVector

            # TODO: do we use the unit vector (for direction) only, or the finalVector?
            finalValue, finalAngle = cart2pol(finalUnitVector[0], finalUnitVector[1])

            # Direct yourself to the goal
            goal_distance = np.linalg.norm(self.goal_pose_pixel - self.pose_pixel)
            goal_theta_diff = angle_diff(mapTheta2worldTheta(self.attractionAngle, self.worldNorthTheta),theta)
            print "Angle Diff ", goal_theta_diff

            # proportional control on angular velocity
            k_angular = 20.0 * (1.0 / (goal_distance + 1.0))  # turn faster when you are closer

            delta_theta = angle_diff(mapTheta2worldTheta(finalAngle, self.worldNorthTheta), theta)
            omega = k_angular * delta_theta
            print "Omega: ", round(omega,1)

            print "distance_from_goal: ", goal_distance
            if goal_distance <= 4:
                # Achieved Goal!
                # forward_vel = 0
                self.curr_goal += 1
            else:
                forward_vel = 1.0
            g = 1

            # if np.abs(omega - 0) < 0.2:
            #     forward_vel = 0.5
            # else:
            #     forward_vel = 0.0
            #     omega *= 5
            # control the motors
            ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g)
            _ = vrep.simxSetJointTargetVelocity(
                self.clientID,self.leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
            _ = vrep.simxSetJointTargetVelocity(
                self.clientID,self.rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity

            # plt.imshow(walls, cmap='gray')
            # plt.imshow(no_doors)
            # plt.imshow(blurred_map)
            # plt.imshow(paths)
            im[goal_m,goal_n] = np.array((1.0, 1.0, 1.0))
            im[m,n,:] = np.array((255.0/255.0,192/255.0,203/255.0))
            plt.imshow(im)
            plt.pause(0.1)
            # time.sleep(0.05) #sleep 50ms

    def run(self):
        if self.clientID!=-1:
            self.robot_code()


        vrep.simxFinish(self.clientID)
        print 'Program ended'

if __name__ == '__main__':
    obj = Lab2Program()
    obj.run()