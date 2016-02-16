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
from scipy.spatial.distance import cityblock, euclidean
import signal
import sys
from json import load, dump

from datastructs import PriorityQueueSet, Tree
from idash import IDash
from ringbuffer import RingBuffer

sys.setrecursionlimit(100000)

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

def test_angle_diff():
    angles = np.linspace(-np.pi, np.pi, 64)
    theta1s = np.linspace(-np.pi, np.pi, 8)
    for count, theta1 in enumerate(theta1s):
        diffs = []
        for theta0 in angles:
            diffs.append(angle_diff(theta0, theta1))
        plt.subplot(4,2,count)
        plt.plot(diffs)
    plt.pause(15)

# test_angle_diff()

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
    """ NOTE: arctan2 returns phi in the range [-pi, pi] """
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    """ where rho is the Radius, and phi is the angle """
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def test_pol2cart2pol():
    # pol2cart
    rho = 1
    angles = np.linspace(0, 2*np.pi, 64)
    xs, ys = zip(*[pol2cart(rho, angle) for angle in angles])

    plt.subplot(3,1,1)
    plt.plot(angles)

    # cart2pol
    new_rhos, new_angles = zip(*[cart2pol(x,y) for x, y in zip(xs, ys)])
    plt.subplot(3,1,2)
    plt.plot(new_angles)
    plt.subplot(3,1,3)
    plt.plot([ThetaRange.normalize_angle(ang) for ang in angles])
    plt.pause(15)

# test_pol2cart2pol()

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

def worldTheta2mapTheta(worldTheta, northTheta):
    # to convert mapTheta pi/2 which is north, to northTheta, we have to do add a bias
    bias = ThetaRange.normalize_angle(northTheta - np.pi / 2)

    worldTheta = ThetaRange.normalize_angle(worldTheta - bias)
    return worldTheta

def test_worldTheta2mapTheta():
    """ if north Theta given by 0 ..."""
    northTheta = 0
    # east
    print worldTheta2mapTheta(-np.pi / 2, northTheta)
    assert ThetaRange.normalize_angle(0) == worldTheta2mapTheta(-np.pi / 2, northTheta)
    # north
    assert ThetaRange.normalize_angle(1*np.pi/2) == worldTheta2mapTheta(0, northTheta)
    # west
    assert ThetaRange.normalize_angle(2*np.pi/2) == worldTheta2mapTheta(np.pi / 2, northTheta)
    # south
    assert ThetaRange.normalize_angle(3*np.pi/2) == worldTheta2mapTheta(-np.pi, northTheta)

    northTheta = -np.pi / 2
    # east
    assert ThetaRange.normalize_angle(0) == worldTheta2mapTheta(-np.pi, northTheta)
    # north
    assert ThetaRange.normalize_angle(1*np.pi/2) == worldTheta2mapTheta(-np.pi / 2, northTheta)
    # west
    assert ThetaRange.normalize_angle(2*np.pi/2) ==worldTheta2mapTheta(0, northTheta)
    # south
    assert ThetaRange.normalize_angle(3*np.pi/2) == worldTheta2mapTheta(np.pi / 2, northTheta)

# test_worldTheta2mapTheta()

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

    plt.pause(15)

# test_force_repulsion()

def tupstring2tuple(tupstring):
    """ from '(5,3)' to 5, 3 as integers """
    m, n = tupstring.strip('()').split(',')
    return int(m), int(n)

class GracefulKiller:
    kill_now = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self,signum, frame):
        self.kill_now = True

class Grid:
    def __init__(self, grid):
        self.grid = grid
        self.shape = grid.shape

    def __getitem__(self, arr):
        return self.grid[arr[0], arr[1]]

    def __setitem__(self, idx, val):
        self.grid[idx] = val

class AStarBinaryGrid:
    def __init__(self, binary_grid=None, heuristic=cityblock):
        """
        binary grid: True denoting a valid grid location for the path
        heuristic: function handle, which takes two vectors and computes a distance heuristic
        """
        if binary_grid is None:
            self.grid = Grid(np.load('binary_grid.npy'))
        else:
            assert len(binary_grid.shape) == 2
            self.grid = Grid(binary_grid)

        self.heuristic = lambda x, y: heuristic(np.array(x), np.array(y))

    def neighbors_get(self, arr):
        directions = [
              [0, 1]
            , [1, 0]
            , [0, -1]
            , [-1, 0]
        ]
        np.random.shuffle(directions)
        for direction_vect in directions:
            neighbor_cell = np.array(arr) + np.array(direction_vect)
            if self.grid[neighbor_cell]:
                yield (neighbor_cell[0], neighbor_cell[1])

    def calculate_path(self, start, finish):
        assert (len(start) == 2)
        assert (len(finish) == 2)
        start = (start[0], start[1])
        finish = (finish[0], finish[1])
        assert self.grid[start] # start is valid
        assert self.grid[finish] # finish is valid
        g_cost = Grid(np.zeros(self.grid.shape))

        g_cost[start] = 0

        # Priority Queues:
        # The lowest valued entries are retrieved first (the lowest valued entry is the one returned by sorted(list(entries))[0]).
        # A typical pattern for entries is a tuple in the form: (priority_number, data).
        to_visit = PriorityQueueSet()
        to_visit.put((
            self.heuristic(start, finish),
            start
        ))
        dead_nodes = []
        path = []
        path.append(start)
        tree = Tree()
        nested_tree = tree

        while not(to_visit.empty()):
            (priority_number, current) = to_visit.get()
            if self.heuristic(current, finish) == 0.0:
                print "Goal Found!"
                print "nested_tree: \n", nested_tree
                break
            nested_tree = nested_tree[str(current)]
            for neigh in self.neighbors_get(current):
                # let leaves equal pixel tuples; they will replaces in the next call

                condA = (neigh not in dead_nodes) # not dead, still valid
                condB = (not to_visit.contains(neigh)) # we don't have it on our todo list yet
                condC = (g_cost[current] + 1 < g_cost[neigh]) # we can get to this neighbor in less cost via our current path
                if condA and (condB or condC):
                    nested_tree[str(neigh)] = Tree()
                    g_cost[neigh] = g_cost[current] + 1 # cost of neighbor increases by one, since its one move farther away from start
                    to_visit.put((
                        g_cost[current] + 1 + self.heuristic(neigh, finish), # one move farther from start + heuristic distance from finish
                        neigh
                    ))
            dead_nodes.append(current)


        filepath = 'GOALS.json'
        def rememberChain(t, chain):
            if len(t) > 0:
                for node in t.iterkeys():
                    # DONT IMMEDIATELY append to the chain, since this chain will continue with every neighbor node!
                    # chain.append(node)
                    GOALS = [tupstring2tuple(tupstring) for tupstring in chain]
                    self.plot(GOALS)
                    if node == str(finish):
                        print "FOUND THE FUCKING CHAIN WHY CANT I RETURN IT: \n",chain
                        dump(chain, open(filepath, 'w'))
                    else:
                        print len(chain)
                        rememberChain(t[node], chain + (node,))


        def nestedIndexTracker(t, chain):
            if len(t) > 0:
                evalStatement = "t"
                for indexer in chain:
                    evalStatement += "['%s']"%indexer
                subtree = eval(evalStatement)
                for node in subtree.iterkeys():
                    # DONT IMMEDIATELY append to the chain, since this chain will continue with every neighbor node!
                    # chain.append(node)
                    GOALS = [tupstring2tuple(tupstring) for tupstring in chain]
                    self.plot(GOALS)
                    if node == str(finish):
                        print "FOUND THE FUCKING CHAIN WHY CANT I RETURN IT: \n",chain
                        dump(chain, open(filepath, 'w'))
                    else:
                        print len(chain)
                        nestedIndexTracker(t, chain + (node,))

        rememberChain(tree, ())
        # nestedIndexTracker(tree, ())
        # tree.selfParams(finish, filepath)
        # tree.rememberChain([])
        goal_strings = load(open(filepath, 'r'))
        GOALS = [tupstring2tuple(tupstring) for tupstring in goal_strings]
        return GOALS

    def plot(self, pixels_to_mark=None):
        im = self.grid.grid*1.0
        if pixels_to_mark is None:
            pass
        else:
            for pix in pixels_to_mark:
                im[pix] = 0.5
        plt.clf()
        plt.imshow(im)
        plt.pause(0.05)

astar = AStarBinaryGrid(heuristic=cityblock)
start_pix = (55,6)
finish_pix = (5,55)
GOALS = astar.calculate_path(start_pix, finish_pix)
astar.plot(GOALS)

class Lab2Program:

    def __init__(self):
        self.initialize_vrep_client()
        self.initilize_vrep_api()
        self.define_constants()
        self.killer = GracefulKiller()
        self.idash = IDash(framerate=0.05)

    def initialize_vrep_client(self):
        #Initialisation for Python to connect to VREP
        print 'Python program started'
        count = 0
        num_tries = 10
        while count < num_tries:
            vrep.simxFinish(-1) # just in case, close all opened connections
            self.clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #Timeout=5000ms, Threadcycle=5ms
            if self.clientID!=-1:
                print 'Connected to V-REP'
                break
            else:
                "Trying again in a few moments..."
                time.sleep(3)
                count += 1
        if count >= num_tries:
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
        # self.GOALS = [(40+2,6), (40, 6+2), (40,21), (35, 19), (30,22),  (29,10), (27,5), (20,8), (20,33), (20, 48), (5,55)]
        self.GOALS = None
        self.worldNorthTheta = None
        self.maxVelocity = 2.0
        self.history_length = 5
        self.theta_history = RingBuffer(self.history_length)
        self.e_theta_h = RingBuffer(self.history_length)
        self.blurred_paths = None
        self.path_skip = 8

    def global_map_preprocess(self, resolution, image):
        im = format_vrep_image(resolution, image) # original image
        im = image_vert_flip(im)
        resize_length = int(im.shape[0]/2)
        im = skimage.transform.resize(im, (resize_length, resize_length, 3))
        return im

    def global_map_process(self, im):
        walls = im[:,:,0] > 0.25
        paths = walls < 0.15
        if self.blurred_paths is None:
            print "Computed"
            blurred_map = skimage.filters.gaussian_filter(walls, sigma=2)
            blurred_paths = blurred_map < 0.15
            # np.save('binary_grid.npy', blurred_paths)
            return paths, blurred_paths
        else:
            return paths, self.blurred_paths

    def robot_pose_get(self):
        _, xyz = vrep.simxGetObjectPosition(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_buffer)
        _, eulerAngles = vrep.simxGetObjectOrientation(
            self.clientID, self.ePuck, -1, vrep.simx_opmode_buffer)
        x, y, z = xyz
        theta = eulerAngles[2]
        self.theta_history.append(theta)

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
            # theta = self.theta_history.weighted_average(scheme='last', mathfunc=lambda x: np.exp(x))

            # initialize worldNorthTheta for the first time
            if self.worldNorthTheta is None:
                self.worldNorthTheta = self.pose[2]
                _ = [self.theta_history.append(self.pose[2]) for _ in range(self.history_length)]

            self.pose_pixel = np.array(odom2pixelmap(x, y, self.map_side_length, im.shape[0]))
            m, n = self.pose_pixel


            self.paths, self.blurred_paths = self.global_map_process(im)

            # calculate intermediate goals once
            if self.GOALS is None:
                raw_input("")
                # acquire pixel location of goal
                _, finishPose = vrep.simxGetObjectPosition(
                    self.clientID, self.goalHandle, -1, vrep.simx_opmode_buffer)
                self.finish_pixel = odom2pixelmap(finishPose[0], finishPose[1], self.map_side_length, im.shape[0])
                contiguousPath = AStarBinaryGrid(self.blurred_paths).calculate_path(self.pose_pixel, self.finish_pixel)
                self.GOALS = contiguousPath[::self.path_skip]
                # SKIP THIS FIRST LOOP AND CONTINUE
                continue
            else:
                self.goal_pose_pixel = np.array(self.GOALS[self.curr_goal])
                goal_m, goal_n = self.goal_pose_pixel
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

            error_theta = angle_diff(mapTheta2worldTheta(finalAngle, self.worldNorthTheta), theta)
            self.e_theta_h.append(error_theta)

            k_angular_p = 2.0 * self.maxVelocity
            k_angular_D = 0.25
            k_angular_S = 1.0
            k_angular_I = 2.5
            omega = k_angular_p * (
                  error_theta
                + k_angular_D / k_angular_S * (self.e_theta_h[-1] - self.e_theta_h[-2])
                + k_angular_S / k_angular_I * sum(self.e_theta_h)
            )
            print "Omega: ", round(omega,1)

            #############################
            # StateController Algorithm #
            #############################
            # if desired heading is not directly in front
            if np.abs(error_theta) > np.pi / 3:
                # turn in place
                forward_vel = self.maxVelocity
                # omega *= 0.25
            else:
                # Direct yourself to the goal
                goal_distance = np.linalg.norm(self.goal_pose_pixel - self.pose_pixel)
                print "distance_from_goal: ", goal_distance
                if goal_distance <= 4:
                    # Achieved Goal!
                    forward_vel = self.maxVelocity * 0.25 # Slow down to prepare for the next one
                    self.curr_goal += 1
                else:
                    forward_vel = self.maxVelocity

            # control the motors
            ctrl_sig_left, ctrl_sig_right = vomega2bytecodes(forward_vel, omega, g=1)
            _ = vrep.simxSetJointTargetVelocity(
                self.clientID,self.leftMotor,ctrl_sig_left,vrep.simx_opmode_oneshot_wait) # set left wheel velocity
            _ = vrep.simxSetJointTargetVelocity(
                self.clientID,self.rightMotor,ctrl_sig_right,vrep.simx_opmode_oneshot_wait) # set right wheel velocity

            self.idash.add(lambda: self.plot_maze(self.blurred_paths*1.0, m, n, goal_m, goal_n))
            self.idash.add(lambda: self.plot_maze(im, m, n, goal_m, goal_n))
            def plot_current_and_desired_heading():
                self.plot_unit_quiver(finalUnitVector, 'r')
                self.plot_unit_quiver(pol2cart(1, worldTheta2mapTheta(theta, self.worldNorthTheta)), 'k')
                plt.title("Error Theta: %f" % error_theta)
            self.idash.add(plot_current_and_desired_heading)
            self.idash.add(self.plot_theta_history)

            self.idash.plotframe()

            if self.killer.kill_now:
                self.clean_exit()

    def plot_maze(self, im, m, n, goal_m, goal_n):
        """ plots the maze, with robot pose and goal pose visualized """
        if len(im.shape) == 3:
            goal_pixel_values = np.array((1.0, 1.0, 1.0))
            robot_pixel_values = np.array((255.0/255.0,192/255.0,203/255.0))
        elif len(im.shape) == 2:
            goal_pixel_values = 0.5
            robot_pixel_values = 0.5
        im[goal_m,goal_n] = goal_pixel_values
        im[m,n] = robot_pixel_values
        plt.imshow(im)

    def plot_unit_quiver(self, vector, color):
        X, Y = (0, 0)
        U, V = (vector[0], vector[1])
        plt.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1,color=color)
        plt.xlim([-1.1,1.1])
        plt.ylim([-1.1,1.1])

    def plot_theta_history(self, expansion=5):
        plt.plot([theta for theta in self.theta_history])
        if len(self.theta_history) < expansion:
            plt.xlim([0, expansion])
        else:
            plt.xlim([0, len(self.theta_history)])
        ylim = np.pi + 0.5
        plt.ylim([-ylim, ylim])

    def plot_all_goals(self, im):
        # display all goals
        for goal_idx in range(len(self.GOALS)):
            im[self.GOALS[goal_idx][0], self.GOALS[goal_idx][1]] = np.array((1.0, 1.0, 1.0))

    def clean_exit(self):
        _ = vrep.simxStopSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
        vrep.simxFinish(self.clientID)
        print 'Program ended'
        sys.exit(0)

    def run(self):
        if self.clientID!=-1:
            _ = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_oneshot_wait)
            self.robot_code()

        self.clean_exit()

if __name__ == '__main__':
    obj = Lab2Program()
    obj.run()
