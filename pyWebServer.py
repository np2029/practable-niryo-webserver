# Practable.io - Niryo Ned 2 Robotic Manipulator Middleware Control Server
# Written by Nathan Page As part of a MEng Software Engineering Degree at Heriot Watt University

# This program integrates a Niryo Ned 2 Robotic Manipulator with the the Practable.io remote lab system.
# For more information, visit https://practable.io
# -------------------------------------------------------------------------------------------------------

# imports
import json
import datetime
import queue
import time
import math
import os

from time import sleep
from enum import Enum

import pyniryo as pn  # I appologise an advance for the confusion this will cause
import numpy as np
import threading

import websockets
from websockets.sync.client import connect
from scipy.spatial.transform import Rotation as R


# import the custom robotic manipulator class used for collision detection
from roboticManipulator import RoboticManipulator
from roboticManipulator import Collider
from roboticManipulator import AlignedCuboidCollider
from roboticManipulator import CuboidCollider
from roboticManipulator import SphereCollider

# =============
# | CONSTANTS |
# =============
PROGRAM_START_TIME = time.gmtime()

# IP address for the robot
# ROBOT_IP = "10.10.10.10"      # wifi hotspot
ROBOT_IP = "169.254.200.200"    # ethernet cable

# number of attempts to connect to the arm
# NO_CONNECTION_ATTEMPTS_ARM = 3

# number of connection attempts to the practable websocket
# NO_CONNECTION_ATTEMPTS_PRACTABLE = 3

# seconds between sets of connection attempts
CONNECTION_ATTEMPT_COOLDOWN_PRACTABLE = 30

# localhost websocket port to send and recieve data to/from Practable.io
PRACTABLE_WEBSOCKET_ADDRESS = "ws://localhost:8888/ws/data"

# directory to store logs
LOG_DIRECTORY = os.path.join(
    os.path.split(os.path.realpath(__file__))[0],
    "logs"
    )


# TCP limits in meters
TCP_LIMIT_UPPER_X = 0.490
TCP_LIMIT_LOWER_X = -0.490

TCP_LIMIT_UPPER_Y = 0.490
TCP_LIMIT_LOWER_Y = -0.490

# NOTE: the bottom collider of the arm sits at z = 0.0043
# If the lower limit is set higher than this, it will always fail if that collider exists
# as the base colliders are static, you can safely remove them during real use
TCP_LIMIT_UPPER_Z = 0.490
TCP_LIMIT_LOWER_Z = 0.0005 # 0 is barely safe on a flat table. Recomend >= 0.0005

# callsigns for the different threads. Prepend these to every print
# master thread
CS_M = "M > "
# practable thread
CS_P = "P > "
# arm thread
CS_A = "A > "

VALID_COMMANDS = {
    "move_tcp":{
        "x":float,
        "y":float,
        "z":float,
        "roll":float,
        "pitch":float,
        "yaw":float
    },
    "move_jp":{
        "j0":float,
        "j1":float,
        "j2":float,
        "j3":float,
        "j4":float,
        "j5":float
    },
    "update_jog_jp":{
        "j0":float,
        "j1":float,
        "j2":float,
        "j3":float,
        "j4":float,
        "j5":float
    },
    "gripper_open":{},
    "gripper_close":{},
    "gripper_toggle":{},
    "gripper_control":{
        "position":int,
        "speed":int,
        "max_torque":int,
        "hold_torque":int
    },
    "signal":{
        "text":str
    },
    "calibrate":{},
    "go_home":{},
    "mutable_move_tcp":{
        "x":float,
        "y":float,
        "z":float,
        "roll":float,
        "pitch":float,
        "yaw":float
    },
    "mutable_move_jp":{
        "j0":float,
        "j1":float,
        "j2":float,
        "j3":float,
        "j4":float,
        "j5":float
    },
    # NOTE: this command is intended for internal use only.
    # It will execute fine if it is recieved, but it should not be made available
    "idle":{}
}

# global command queue
COMMAND_QUEUE = queue.Queue()

# global log file write queue
LOG_QUEUE = queue.Queue()

# to be updated when the robot is initialised and turned off
ROBOT_SETUP_COMPLETE = False

# Threading Events:
# events used in thread initialisation
armThreadInit = threading.Event()
pthreadInit = threading.Event()

# used for testing if the arm is hanging on its connection step
armConnectionHangTestEvent = threading.Event()

# event used in the master thread loop
masterThreadEvent = threading.Event()

# event used to flag the practable thread isn't working
# used when sending messages
practableErrorEvent = threading.Event()


# log file things
# Enum class to make sure logged events are consistant
class LogEvents(Enum):
    # Threads
    THREAD_STARTED = "THREAD_STARTED"
    THREAD_DIED = "THREAD_DIED"

    # Practable packets
    PRACTABLE_CONNECTION_ATTEMPT = "PRACTABLE_CONNECTION_ATTEMPT"
    PRACTABLE_CONNECTION_SUCCESS = "PRACTABLE_CONNECTION_SUCCESS"

    PRACTABLE_PACKET_RECIEVED = "PRACTABLE_PACKET_RECIEVED"
    PRACTABLE_PACKET_SENT = "PRACTABLE_PACKET_SENT"

    PRACTABLE_ERROR_BAD_PACKET = "PRACTABLE_ERROR_BAD_PACKET"

    # Arm commands
    ARM_CONNECTION_ATTEMPT = "ARM_CONNECTION_ATTEMPT"
    ARM_CONNECTION_ATTEMPT_TIMEOUT = "ARM_CONNECTION_ATTEMPT_TIMEOUT"
    ARM_CONNECTION_SUCCESS = "ARM_CONNECTION_SUCCESS"
    ARM_CONNECTION_HANGING = "ARM_CONNECTION_HANGING"

    ARM_COMMAND_QUEUED = "ARM_COMMAND_QUEUED"
    ARM_COMMAND_STARTED = "ARM_COMMAND_STARTED"
    ARM_COMMAND_COMPLETED = "ARM_COMMAND_COMPLETED"
    ARM_COMMAND_MUTABLE_UPDATE = "ARM_COMMAND_MUTABLE_UPDATE"

    ARM_IDLE = "ARM_IDLE"

    ARM_COMMAND_TIMEOUT = "ARM_COMMAND_TIMEOUT"
    ARM_COMMAND_TIMEOUT_UPDATE = "ARM_COMMAND_TIMEOUT_UPDATE"

    #  Exceptions
    EXCEPTION_GENERAL = "EXCEPTION_GENERAL"
    EXCEPTION_PRACTABLE = "EXCEPTION_PRACTABLE"
    EXCEPTION_ARM = "EXCEPTION_ARM"


# ===========
# | CLASSES |
# ===========




# =============================
# | COLLISION DETECTION SETUP |
# =============================
# need to build the joints first
JOINT_BASE = RoboticManipulator.Joint(
    0, 0, 0, # offsets
    0, 0, 0, # rotation
    None,
    angle=0,
    name="Base"
)

JOINT_SHOULDER = RoboticManipulator.Joint(
    0, 0, 0.1013, # offsets
    0, 0, 0, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_BASE, 
    upperLimit=2.949, 
    lowerLimit=-2.949,
    name="Shoulder"
)

JOINT_ARM = RoboticManipulator.Joint(
    0, 0, 0.065, # offsets
    np.pi/2, 0, 0, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_SHOULDER, 
    upperLimit=0.61, 
    lowerLimit=-1.83,
    name="Arm"
)

JOINT_ELBOW = RoboticManipulator.Joint(
    0.012, 0.221, 0, # offsets
    0, 0, np.pi/2, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_ARM, 
    upperLimit=1.57, 
    lowerLimit=-1.34,
    name="Elbow"
)

JOINT_FOREARM = RoboticManipulator.Joint(
    0.0325, -0.065, 0, # offsets
    np.pi/2, 0, 0, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_ELBOW, 
    upperLimit=2.089,
    lowerLimit=-2.089,
    name="Forearm"
)

JOINT_WRIST = RoboticManipulator.Joint(
    0, 0, 0.17, # offsets
    -np.pi/2, 0, 0, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_FOREARM, 
    upperLimit=1.922,
    lowerLimit=-1.919,
    name="Wrist"
)

JOINT_HAND = RoboticManipulator.Joint(
    0.00925, -0.0197, 0, # offsets
    np.pi/2, 0, 0, # rotation
    RoboticManipulator.AXIS_Z,
    angle=0, 
    parentJoint=JOINT_WRIST, 
    upperLimit=2.53,
    lowerLimit=-2.53,
    name="Hand"
)

JOINT_GRIPPER = RoboticManipulator.Joint(
    0, 0, 0.01016, # offsets
    0, 0, 0, # rotation
    None,
    angle=0, 
    parentJoint=JOINT_HAND, 
    upperLimit=2.53,
    lowerLimit=-2.53,
    name="Gripper"
)

# in-order joint array for convenience
JOINTS_IN_ORDER = [
    JOINT_BASE,
    JOINT_SHOULDER,
    JOINT_ARM,
    JOINT_ELBOW,
    JOINT_FOREARM,
    JOINT_WRIST,
    JOINT_HAND,
    JOINT_GRIPPER,
]

# now we make the links
LINK_BASE_SHOULDER = RoboticManipulator.Link(parentJoint=JOINT_BASE, name="Base -> Shoulder Link")
LINK_SHOULDER_ARM = RoboticManipulator.Link(parentJoint=JOINT_SHOULDER, name="Shoulder -> Arm Link")
LINK_ARM_ELBOW = RoboticManipulator.Link(parentJoint=JOINT_ARM, name="Arm -> Elbow Link")
LINK_ELBOW_FOREARM = RoboticManipulator.Link(parentJoint=JOINT_ELBOW, name="Elbow -> Forearm Link")
LINK_FOREARM_WRIST = RoboticManipulator.Link(parentJoint=JOINT_FOREARM, name="Forearm -> Wrist Link")
LINK_WRIST_HAND = RoboticManipulator.Link(parentJoint=JOINT_WRIST, name="Wrist -> Hand Link")
LINK_HAND_GIRPPER = RoboticManipulator.Link(parentJoint=JOINT_HAND, name="Hand -> Gripper Link")
LINK_GRIPPER = RoboticManipulator.Link(parentJoint=JOINT_GRIPPER, name="Gripper Actual Link")

# and now the colliders
LINK_BASE_SHOULDER.add_collider([
    AlignedCuboidCollider(
        (0, 0, 0.0448), # origin
        0.198, 0.198, 0.081, # x,y,z size
        name="base - main section"
    ),
    AlignedCuboidCollider(
        (0, 0, 0.093885), # origin
        0.159987, 0.159987, 0.015414, # x,y,z size
        name="base - LED ring"
    ),
])

LINK_SHOULDER_ARM.add_collider([
    CuboidCollider(
        (0.999951-1, -0.0001, 0.057294), # origin
        0.096747, 0.1148, 0.114589, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="shoulder - main joint"
    ),
    CuboidCollider(
        (0.921189-1, 0.012574, 0.026108), # origin
        0.060777, 0.120253, 0.053785, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="shoulder - motor box"
    )
])

LINK_ARM_ELBOW.add_collider([
    # rotation:
    # x -> x
    # y -> -z
    # z -> y
    CuboidCollider(
        (2-2, 0.077342, -0), # origin
        0.05264, 0.105281, 0.05264,# x,y,z size
        0, 0, 0, # x,y,z rotation
        name="arm - connecting box"
    ),
    CuboidCollider(
        (2-2, 0.14469, -0), # origin
        0.064, 0.06402, 0.101, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="arm - mid box"
    ),
    CuboidCollider(
        (2.00015-2, 0.154575, -0.06035), # origin
        0.0423, 0.05285, 0.0197, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="arm - side box"
    ),
    CuboidCollider(
        (2.0125-2, 0.2207, -0), # origin
        0.088, 0.088, 0.101, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="arm - top box"
    ),
])

LINK_ELBOW_FOREARM.add_collider([
    # rotation:
    # x -> -y
    # y -> -z
    # z -> x
    CuboidCollider(
        (0.037725, -(3.00788-3), -0), # origin
        0.06005, 0.07777, 0.062, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="elbow"
    ),
])

LINK_FOREARM_WRIST.add_collider([
    # rotation:
    # x -> z
    # y -> -y
    # z -> x
    CuboidCollider(
        (-0.000218, 0, 4.00341-4), # origin
        0.057091, 0.06, 0.043388, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="forearm - socket"
    ),
    CuboidCollider(
        (0.0002, 0, 4.048-4), # origin
        0.04, 0.04, 0.047896, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="forearm - bridge"
    ),
    CuboidCollider(
        (0, 0, 4.13231-4), # origin
        0.05, 0.06, 0.125736, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="forearm - main"
    ),
    CuboidCollider(
        (0, 0.045, 4.158-4), # origin
        0.0285, 0.03, 0.0465, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="forearm - side"
    ),
])

LINK_WRIST_HAND.add_collider([
    # rotation:
    # x -> -y
    # y -> -z
    # z -> x
    CuboidCollider(
        (0, -(4.981-5), 0), # origin
        0.036, 0.06, 0.03, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="wrist"
    ),
])

LINK_HAND_GIRPPER.add_collider([
    # rotation:
    # x -> z
    # y -> -y
    # z -> x
    CuboidCollider(
        (-0.00005, 0, 6.0113-6), # origin
        0.02, 0.02, 0.035, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="hand"
    ),
])

LINK_GRIPPER.add_collider([
    # rotation:
    # x -> z
    # y -> -y
    # z -> x
    CuboidCollider(
        (-0.00025, 0, 7.0616-7), # origin
        0.0235, 0.076999, 0.085355, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="gripper - main"
    ), 
    CuboidCollider(
        (0.014912, -0.000219, 7.04175-7), # origin
        0.006823, 0.082321, 0.024491, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="gripper - upper"
    ), 
    CuboidCollider(
        (-0.0165, 0.0075, 7.04-7), # origin
        0.009, 0.034, 0.02, # x,y,z size
        0, 0, 0, # x,y,z rotation
        name="gripper - lower"
    ),
])

# and finally declare the arm!
RM = RoboticManipulator([
    JOINT_BASE,
    JOINT_SHOULDER,
    JOINT_ARM,
    JOINT_ELBOW,
    JOINT_FOREARM,
    JOINT_WRIST,
    JOINT_HAND,
    JOINT_GRIPPER,
])


# ===========================
# | ENVIRONMENTAL COLLIDERS |
# ===========================
# TODO: get this from a config/setup file
ENVIRONMENTAL_COLLIDERS = [
# testing colliders:
# one placed slightly forward of the default home pose.
# test by moving forward 0.2 
# test with the following command:
# {"command":"mutable_move_tcp", "x":"0.4", "y":"0.0", "z":"0.15", "roll":"0.0", "pitch":"0.0", "yaw":"0.0"}
AlignedCuboidCollider(
    (0.4, 0, 0.15),
    0.1, 0.1, 0.1,
    name="Testing Collider - Forward",
)
]



# ====================
# | HELPER FUNCTIONS |
# ====================

# VERY important function.
# returns whether the given pose is a valid (and SAFE) position.
# get this function right, or be ready to pay 4 grand when someone breaks the arm
# accepts JointPosition or PoseObject objects
# FIXME: this needs to check for custom areas
def verifyPosition(position):
    # check type of position. Need joint angles, so we need a JointsPosition
    # make a new object to not mutate the supplied one
    if type(position) == pn.PoseObject:
        pos = robot.inverse_kinematics(position)
    elif type(position) == pn.JointsPosition:
        pos = position
    else:
        # incorrect object given
        raise TypeError(f"unsupported type {type(position)} for verifyPosition")

    # use pos to update the joint angles
    for i in range(6):
        JOINTS_IN_ORDER[i+1].set_angle(pos[i])

    # we need to check 2 things:
    # 1: the hard TCP bounds limits
    # 2: the environmental colliders

    # to do both, we need the verts of every joint on the arm
    vertList = RM.get_verts()
    armColliders = RM.get_colliders()

    # check TCP bounds first because its quicker
    # also because its quicker, we can check the bounding spheres first
    for c in armColliders:
        cpos = c.get_position()
        if (
           cpos[0] + c.get_bounding_sphere_radius() > TCP_LIMIT_UPPER_X
        or cpos[0] - c.get_bounding_sphere_radius() < TCP_LIMIT_LOWER_X

        or cpos[1] + c.get_bounding_sphere_radius() > TCP_LIMIT_UPPER_Y
        or cpos[1] - c.get_bounding_sphere_radius() < TCP_LIMIT_LOWER_Y

        or cpos[2] + c.get_bounding_sphere_radius() > TCP_LIMIT_UPPER_Z
        or cpos[2] - c.get_bounding_sphere_radius() < TCP_LIMIT_LOWER_Z
        ):
            # collider *might* be exceeding the TCP limits. check vertex by vertex
            for v in c.get_verts():
                if (
                       v[0] > TCP_LIMIT_UPPER_X
                    or v[0] < TCP_LIMIT_LOWER_X

                    or v[1] > TCP_LIMIT_UPPER_Y
                    or v[1] < TCP_LIMIT_LOWER_Y

                    or v[2] > TCP_LIMIT_UPPER_Z
                    or v[2] < TCP_LIMIT_LOWER_Z
                ):
                    # now it has DEFINITELY exceeded the limits. return false
                    return False

    # TCP check complete, now check the colliders
    # invert it as check_collisions returns whether a collision HAS occured,
    # and we want if one HAS NOT occured
    return not RM.check_collisions(ENVIRONMENTAL_COLLIDERS)

# returns true/false on success/failure
def safeMove(pos):
    # check arg type
    if type(pos) == pn.JointsPosition or type(pos) == pn.PoseObject:
        # print(f"safeMove: verifying move to\n{pos}")
        if verifyPosition(pos):
            # print(f"safeMove: move verified, moving to {pos}")
            #try:
            robot.move(pos)  # re-indent when enabling try/except
            #except pn.api.exceptions.NiryoRobotException as e:
                #print(f"safeMove() - ERROR: encountered exception in safemove: {e}")
            return True
        else:
            # print(f"safeMove: Move to position {pos} unsafe, discarded")
            return False
    else:
        raise TypeError(f"unsupported type {type(pos)} for safeMove")

# helper function specifically for sending messages to the practable websocket
# will be used by both the practable thread and arm thread, so should be here
# FIXME: THIS DOESNT TRY AGAIN IF IT FAILS, THAT IS REALLY BAD!!! FIX ASAP!!!
def sendPractableMessage(thread,message):
    if type(message) is not dict:
        # addToLogQueue(thread, f"ERROR IN sendPractableMessage: message {message} is not a dict")
        logEvent(thread, LogEvents.EXCEPTION_GENERAL, TypeError, f"ERROR IN sendPractableMessage: message {message} is not a dict")
        return False

    # convert message into sendable format
    try:  # just in case
        m = json.dumps(message)

    except Exception as e:
        # addToLogQueue(thread, f"ERROR IN sendPractableMessage: dict -> json string conversion failed for message: {message}")
        logEvent(thread, LogEvents.EXCEPTION_GENERAL, e, f"ERROR IN sendPractableMessage: dict -> json string conversion failed for message: {message}")
        return False
    
    try:
        practable_ws.send(m)
        return True
    
    except websockets.ConnectionClosed as e:
        # addToLogQueue(thread, f"ERROR IN sendPractableMessage: Practable connection marked as closed.")
        logEvent(thread, LogEvents.EXCEPTION_PRACTABLE, e, f"ERROR IN sendPractableMessage: Practable connection marked as closed.")
        practableErrorEvent.set()
        return False

    # this should never trigger, but check anyway
    except TypeError as e:
        # addToLogQueue(thread, f"ERROR IN sendPractableMessage: Message {message} has invalid type {type(message)}")
        logEvent(thread, LogEvents.EXCEPTION_GENERAL, TypeError, f"ERROR IN sendPractableMessage: Message {message} has invalid type {type(message)}")
        return False
    
    except Exception as e:
        # addToLogQueue(thread, f"ERROR IN sendPractableMessage: Exception: {e}")
        logEvent(thread, LogEvents.EXCEPTION_GENERAL, e, f"ERROR IN sendPractableMessage: Exception: {e}")
        return False

# to be called on first connection after robot is turned off
def setupRobot():
    global robot
    global gripperOpen
    gripperOpen = True # annoyingly, we need this variable
    robot.open_gripper()
    robot.move(robot.get_home_pose())
    robot.set_home_pose(pn.JointsPosition(0, 0.5, -1.25, 0,0,0))

def logEvent(thread, event, eventData, information):
    # format the message correctly
    # m = f"{time.strftime("%Y-%d-%mT%H:%M:%SZ",time.gmtime())} - {thread} - {message}\n"
    m = f"{time.strftime("%Y-%d-%mT%H:%M:%SZ",time.gmtime())}" + "|"
    m += f"{thread}" + "|"
    m += f"{event}" + "|"
    m += f"{eventData}" + "|"
    m += f"{information}"
    m += "\n"

    LOG_QUEUE.put(m)

    # we also print each message so we have a live feed
    print(m)

# ==================
# | INITIALISATION |
# ==================

# TODO: check and open log file
# TODO: check and read config file

robot = None
#homePose = robot.forward_kinematics(pn.JointsPosition(0,0.5,-1.25,0,0,0))
#robot.set_home_pose(homePose)

practable_ws = None

# move this later
practableThreadKillEvent = threading.Event()

def practableThreadFunction():
    logEvent(CS_P, LogEvents.THREAD_STARTED, CS_P, f"PRACTABLE THREAD STARTED")
    # globals
    global practable_ws

    while not practableThreadKillEvent.is_set():
        # PLAN
        # ws initialised as None.
        # on function enter, check if none.
        # if none, raise connectionclosed 

        # WE NEED 2 LOOPS: 
        # one if connection closed
        # one if connection failed (couldn't connect in the first place)

        try:
            # main code here
            if practable_ws is None:
                # this triggers the code below to begin connection attempts
                raise websockets.ConnectionClosedError(None, None)
            
            # wait for an incoming message
            incoming = practable_ws.recv()
            # print(f"{CS_P}recieved message:\n{incoming}")
            # addToLogQueue(CS_P, f"recieved message:\n{incoming}")
            logEvent(CS_P, LogEvents.PRACTABLE_PACKET_RECIEVED, {incoming}, f"recieved message:\n{incoming}")

            # message verification
            # check if json is valid
            try:
                messageJSON = json.loads(incoming)
            except json.decoder.JSONDecodeError:
                # command was bad json. send a reply stating as such
                # practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: BAD JSON - FAILED TO DECODE"}')
                sendPractableMessage(CS_P, {
                    "replyComm":"NOT_SET",
                    "result":"fail",
                    "displayText":"Error: Invalid command.",
                    "message":"ERROR: BAD JSON - FAILED TO DECODE"
                })
                # print(f"{CS_P}RECEIVED BAD COMMAND: {messageJSON}")
                # addToLogQueue(CS_P, f"RECEIVED BAD COMMAND: {messageJSON}")
                logEvent(CS_P, LogEvents.PRACTABLE_ERROR_BAD_PACKET, messageJSON, f"RECEIVED BAD COMMAND: {messageJSON}")
                continue

            # json is correct, check if command variable exists
            if "command" not in messageJSON:
                # practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE NOT SET FOR RECIEVED COMMAND"}')
                sendPractableMessage(CS_P, {
                    "replyComm":"NOT_SET",
                    "result":"fail",
                    "displayText":"Error: Invalid command.",
                    "message":"ERROR: COMMAND ATTRIBUTE NOT SET FOR RECIEVED COMMAND"})
                # print(f"{CS_P}ERROR: COMMAND NOT SET IN INCOMING JSON: {messageJSON}")
                # addToLogQueue(CS_P, f"ERROR: COMMAND NOT SET IN INCOMING JSON: {messageJSON}")
                logEvent(CS_P, LogEvents.PRACTABLE_ERROR_BAD_PACKET, messageJSON, f"ERROR: COMMAND NOT SET IN INCOMING JSON: {messageJSON}")
                continue

            # command exists, check if its a real command
            if messageJSON["command"] not in VALID_COMMANDS:
                # command not present. reply with error
                # practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"}')
                sendPractableMessage(CS_P, {
                    "replyComm":"NOT_SET",
                    "result":"fail",
                    "displayText":"Error: Invalid command.",
                    "message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"})
                # print(f"{CS_P}ERROR: COMMAND NOT RECOGNISED: {messageJSON}")
                # addToLogQueue(CS_P, f"ERROR: COMMAND NOT RECOGNISED: {messageJSON}")
                logEvent(CS_P, LogEvents.PRACTABLE_ERROR_BAD_PACKET, messageJSON, f"ERROR: COMMAND NOT RECOGNISED: {messageJSON}")
                continue

            # command is real, check the args
            typeCorrectArgs = {}
            for i in messageJSON:
                if i != "command":
                    try:
                        typeCorrectArgs[i] = VALID_COMMANDS[messageJSON["command"]][i](messageJSON[i])
                    except:
                        # type mismatch, invalid command
                        # practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: ARGUMENT TYPE MISMATCH: "'+str(messageJSON[i])+' IS NOT TYPE '+str(VALID_COMMANDS[messageJSON["command"]][i])+'}')
                        sendPractableMessage(CS_P, {
                            "replyComm":"NOT_SET",
                            "result":"fail",
                            "displayText":"Error: Invalid command.",
                            "message":f"ERROR: ARGUMENT TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}"})
                        # print(f"{CS_P}ERROR: COMMAND TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}")
                        # addToLogQueue(CS_P, f"{CS_P}ERROR: COMMAND TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}")
                        logEvent(CS_P, LogEvents.PRACTABLE_ERROR_BAD_PACKET, {messageJSON[i]}, f"ERROR: COMMAND TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}")
                        break

            # need to check again, previous continue just breaks the arg check loop
            if len(typeCorrectArgs) != len(messageJSON)-1:  # -1 to account for missing "command"
                continue

            # now that we have a 100% valid command and args, we can do any final pre-processing
            match messageJSON["command"]:
                # these commands need little/no pre-processing and can be put directly in the queue
                # each command needs a case so that the acknoledgements can be personalised
                case "move_tcp":
                    COMMAND_QUEUE.put(
                        ("move_tcp",
                        pn.PoseObject(
                            typeCorrectArgs["x"],
                            typeCorrectArgs["y"],
                            typeCorrectArgs["z"],
                            typeCorrectArgs["roll"],
                            typeCorrectArgs["pitch"],
                            typeCorrectArgs["yaw"],
                        )
                        )
                    )
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "move_jp":
                    COMMAND_QUEUE.put(
                        ("move_jp",
                        pn.JointsPosition( 
                                typeCorrectArgs["j0"],
                                typeCorrectArgs["j1"],
                                typeCorrectArgs["j2"],
                                typeCorrectArgs["j3"],
                                typeCorrectArgs["j4"],
                                typeCorrectArgs["j5"]
                            )
                            )
                    )
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "update_jog_jp":
                    COMMAND_QUEUE.put(
                        ("update_jog_jp",
                        pn.JointsPosition( 
                                typeCorrectArgs["j0"],
                                typeCorrectArgs["j1"],
                                typeCorrectArgs["j2"],
                                typeCorrectArgs["j3"],
                                typeCorrectArgs["j4"],
                                typeCorrectArgs["j5"]
                            )
                            )
                    )
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "gripper_open":
                    COMMAND_QUEUE.put(("gripper_open",None))
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "gripper_close":
                    COMMAND_QUEUE.put(("gripper_close",None))
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "gripper_toggle":
                    COMMAND_QUEUE.put(("gripper_toggle",None))
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "gripper_control":
                    COMMAND_QUEUE.put(("gripper_control", typeCorrectArgs))
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "signal":
                    # dont do anything with the command queue, just print to console and log
                    print(f"{CS_P}Signal Recieved: {typeCorrectArgs["text"]}")
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "calibrate":
                    COMMAND_QUEUE.put(("calibrate",None))
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "go_home":
                    COMMAND_QUEUE.put(("go_home",None))
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "mutable_move_tcp":
                    COMMAND_QUEUE.put(
                        ("mutable_move_tcp",
                        pn.PoseObject(
                            typeCorrectArgs["x"],
                            typeCorrectArgs["y"],
                            typeCorrectArgs["z"],
                            typeCorrectArgs["roll"],
                            typeCorrectArgs["pitch"],
                            typeCorrectArgs["yaw"],
                        )
                        )
                    )
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "mutable_move_jp":
                    COMMAND_QUEUE.put(
                        ("mutable_move_jp",
                        pn.JointsPosition( 
                                typeCorrectArgs["j0"],
                                typeCorrectArgs["j1"],
                                typeCorrectArgs["j2"],
                                typeCorrectArgs["j3"],
                                typeCorrectArgs["j4"],
                                typeCorrectArgs["j5"]
                            )
                            )
                    )
                    # practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                case "idle":
                    COMMAND_QUEUE.put(("idle", None))
                    sendPractableMessage(CS_P, {
                        "ACK":messageJSON["command"]
                    })

                # this should never trigger, should be filtered out by above. check anyway
                case _:
                    # practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"}')
                    sendPractableMessage(CS_P, {
                        "replyComm":"NOT_SET",
                        "result":"fail",
                        "displayText":"Error: Invalid command.",
                        "message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"
                    })
                    # print(f"{CS_P}congratulations! you did the impossible and triggered the default case in the command match statement! json:\n{messageJSON}")
                    # addToLogQueue(CS_P, f"congratulations! you did the impossible and triggered the default case in the command match statement! json:\n{messageJSON}")
                    logEvent(CS_P, LogEvents.PRACTABLE_ERROR_BAD_PACKET, messageJSON, f"congratulations! you did the impossible and triggered the default case in the command match statement! json:\n{messageJSON}")
                    continue

        # technically we only need to catch ConnectionClosed, but better be safe
        except (websockets.ConnectionClosedOK, websockets.ConnectionClosedError ,websockets.ConnectionClosed) as e:
            # check for first time connection
            if practable_ws is None:
                # addToLogQueue(CS_P, f"practable websocket is None, making first connection...")
                logEvent(CS_P, LogEvents.EXCEPTION_PRACTABLE, "", f"practable websocket is None, making first connection")
            else:
                # log a bunch of stuff. its about to get set to None anyway
                # addToLogQueue(CS_P, f"Practable thread raised {e}. attempting to reconnect...")
                logEvent(CS_P, LogEvents.EXCEPTION_PRACTABLE, e, f"Practable thread raised {e}. attempting to reconnect")

            # reconnect with the websocket
            # this needs its own while loop and try/except
            practable_ws = None
            while practable_ws is None:
                try:
                    logEvent(CS_P, LogEvents.PRACTABLE_CONNECTION_ATTEMPT, PRACTABLE_WEBSOCKET_ADDRESS, f"Attempting to connect to practable address {PRACTABLE_WEBSOCKET_ADDRESS}")
                    practable_ws = connect(PRACTABLE_WEBSOCKET_ADDRESS)
                    # if this doesn't raise an exception, we will reach this
                    logEvent(CS_P, LogEvents.PRACTABLE_CONNECTION_SUCCESS, "", f"Practable thread successfuly connected to websocket {practable_ws}")
                    break

                # there are too many exception types to handle individually, just do them all
                except Exception as e:
                    # addToLogQueue(CS_P, f"Practable connection attempt raised {e}. retrying...")
                    logEvent(CS_P, LogEvents.EXCEPTION_PRACTABLE, e, f"Practable thread raised {e}. attempting to reconnect")

                # above only breaks the loop, need to check again
                if practable_ws is None:
                    sleep(CONNECTION_ATTEMPT_COOLDOWN_PRACTABLE)

                # if by here a connection has been made, the loop will exit
                logEvent(CS_P, LogEvents.PRACTABLE_CONNECTION_SUCCESS, "", f"Practable thread successfuly connected to websocket {practable_ws}")

    # code here runs if the thread is killed
    # print(f"{CS_P}PRACTABLE THREAD KILLED")
    # addToLogQueue(CS_P, f"PRACTABLE THREAD KILLED")
    logEvent(CS_P, LogEvents.THREAD_DIED, CS_P, f"PRACTABLE THREAD DIED")

# move this later
armThreadKillEvent = threading.Event()

def armThreadFunction():
    global robot
    global gripperOpen
    # this needs aditional safety stuff, but I need to test it for now
    bufferedCommand = None
    command = None

    # for the connection function
    exc = None
    connectionAttemptEnded = threading.Event()

    # need this later for threading reasons
    def armConnectHelper():
        global robot
        nonlocal exc
        try:
            robot = pn.NiryoRobot(ROBOT_IP)

        # this can ONLY throw ClientNotConnected exception
        except pn.api.exceptions.ClientNotConnectedException as e:
            exc = e

        # just in case
        except Exception as e:
            exc = e

        # to signify we are not hanging anymore
        finally:
            connectionAttemptEnded.set()

    # for the watcher function
    # need 2 to avoid busy waiting
    commandStarted = threading.Event()
    commandExecuted = threading.Event()

    # we need a thread to see if any of the arm function calls hang
    def armWatcherFunction():
        while True:
            if not commandStarted.wait(timeout=300):
                # reset to base pose if no commands recieved after long time
                # addToLogQueue(CS_A, f"No commands detected for 5 minutes, inserting idle command")
                logEvent(CS_A, LogEvents.ARM_IDLE, "300", f"No commands detected for 5 minutes, inserting idle command")
                COMMAND_QUEUE.put(("idle",None))
                commandStarted.wait()
            commandStarted.clear()
            commandStartTime = time.time()

            # make a local copy of the command to remove race condition
            localCommand = command

            # special case for calibration. No other command should take longer than 10s
            if localCommand is not None and localCommand[0] == "calibrate":
                t = 30
            else:
                t = 10

            # check if command times out
            if not commandExecuted.wait(timeout=t):
                # things to be done ONCE per timed out command
                # addToLogQueue(CS_A, f"ERROR: Command {command} hanging! Network connection is unstable")
                logEvent(CS_A, LogEvents.ARM_COMMAND_TIMEOUT, localCommand, f"ERROR: Command {localCommand} hanging! Network connection likely dead!")
                sendPractableMessage(CS_A, {
                    "status":"hanging"#FIXME: make better once UI is more final
                })

                while not commandExecuted.wait(timeout=30):
                    # things to be done every so often while current command is hanging
                    t = time.time()-commandStartTime
                    # addToLogQueue(CS_A, f"Command {command} has been hanging for {t} seconds")
                    logEvent(CS_A, LogEvents.ARM_COMMAND_TIMEOUT_UPDATE, t, f"Command {localCommand} has been hanging for {t} seconds")
            
            commandExecuted.clear()
            t = time.time()-commandStartTime
            # addToLogQueue(CS_A, f"Command {command} completed in {t} seconds")
            logEvent(CS_A, LogEvents.ARM_COMMAND_COMPLETED, f"command: {localCommand}, time: {t}", f"Command {localCommand} completed in {t} seconds")

            # special case for idle commands. we wait here until next command
            if localCommand[0] == "idle":
                commandStarted.wait()

    # just in case
    commandStarted.clear()
    commandExecuted.clear()

    # setup watcher thread if it doesnt exist already
    # TODO: bugfix if the main arm thread dies but the watcher doesn't
    armWatcherThread = threading.Thread(target=armWatcherFunction, args=[])
    armWatcherThread.daemon = True
    armWatcherThread.start()


    while not armThreadKillEvent.is_set():
        try:
            # check connection exists
            if robot is None:
                # this will trigger code in the except to establish a connection
                raise pn.api.exceptions.ClientNotConnectedException("robot is None")

            # calibrate if needed
            if robot.need_calibration():
                robot.calibrate_auto()
                time.sleep(1)

            # check for collision
            if robot.collision_detected:
                robot.clear_collision_detected()
                # move to a known safe pose
                robot.move(pn.JointsPosition(0, 0.5, -1.25, 0,0,0))

                # and set command to none so whatever did it doesnt happen again
                command = None

            # get the command for this itteration
            # first, check if its still set due to an exception interrupting it
            if command is None:
                # next, check if there is a buffered command
                if bufferedCommand is None:
                    # get a new command
                    command = COMMAND_QUEUE.get()
                else:
                    command = bufferedCommand
                    bufferedCommand = None

            # we now have the command, lets execute it
            (com, args) = command
            # print(f"{CS_A}executing command: {command}")
            # addToLogQueue(CS_A, f"begining execution of command: {command}")
            logEvent(CS_A, LogEvents.ARM_COMMAND_STARTED, command, f"begining execution of command: {command}")
            sendPractableMessage(CS_A, {
                "executing":com,
                "state":"In-Progress"
            })

            # for watcher thread
            commandStarted.set()

            # practable_ws.send(f'{{""}}')  # send to signify a command has started execution
            match com:
                case "move_tcp":
                    safeMove(args)

                case "move_jp":
                    safeMove(args)

                case "update_jog_jp":
                    final = args[0:6]
                    targets = robot.get_joints()[0:6]  # current angle set as default for safety reasons
                    timeout = 0
                    while True:
                        # main block
                        angles = robot.get_joints()[0:6]
                        jog = [0,0,0,0,0,0]

                        # check for updates to the target
                        if COMMAND_QUEUE.qsize() > 0 and bufferedCommand is None:
                            update = COMMAND_QUEUE.get()
                            if update[0] == "update_jog_jp":
                                final = update[1][0:6]
                            else:
                                bufferedCommand = update
            
                        # find remaining angles for all joints
                        remaining = [0,0,0,0,0,0]
                        for i in range(6):
                            remaining[i] = final[i] - angles[i]
            
                        # check if at final destination
                        if np.all(list(map((lambda x: abs(x) <= 0.01), remaining))):
                            # print("REACHED")
                            break
            
                        # check if all joints at targets yet, assigning new ones and jogging if so
                        if np.all(list(map((lambda x, y: abs(x-y) <= 0.01), angles, targets))) or time.time() > timeout:
                            for i in range(6):
                                if abs(remaining[i]) > 0.001:
                                    jog[i] = math.copysign(min(abs(remaining[i]), 0.2), remaining[i])
                                    targets[i] += jog[i]
                            # print(f"NEW TARGETS: {targets}")
                            # print(f"JOGGING: {jog}")
                            robot.jog(pn.JointsPosition(*jog))
                            timeout = time.time()+1
            
                    sleep(0.2)
                    # print(f"FINAL LOCATION: {robot.get_joints()[0]}")

                case "gripper_open":
                    robot.open_gripper()
                    gripperOpen = True

                case "gripper_close":
                    robot.close_gripper()
                    gripperOpen = False

                case "gripper_toggle":
                    if gripperOpen == None:
                        # could assign default, for now just do nothing
                        pass
                    elif gripperOpen:
                        robot.close_gripper()
                        gripperOpen = False
                    else:
                        robot.open_gripper()
                        gripperOpen = True

                case "gripper_control":
                    robot.control_gripper(
                        args["position"],
                        args["speed"],
                        args["max_torque"],
                        args["hold_torque"]
                    )
                    gripperOpen = None

                case "calibrate":
                    robot.calibrate_auto()

                case "go_home":
                    # use the saved home pose so it works when the user changes it
                    # still need to check it's safe,
                    # otherwise users could use an unsafe home pose to bypass the checks
                    safeMove(robot.get_home_pose())

                case "mutable_move_tcp":
                    target = args
                    # check for updates to the target
                    while COMMAND_QUEUE.qsize() > 0 and bufferedCommand is None:
                        update = COMMAND_QUEUE.get()
                        if update[0] == "mutable_move_tcp":
                            target = update[1]
                            # addToLogQueue(CS_A, f"Updated mutable_move_tcp target to {target}")
                            logEvent(CS_A, LogEvents.ARM_COMMAND_MUTABLE_UPDATE, target, f"Updated mutable_move_tcp target to {target}")
                        else:
                            bufferedCommand = update

                    safeMove(target)

                case "mutable_move_jp":
                    target = args
                    # check for updates to the target
                    while COMMAND_QUEUE.qsize() > 0 and bufferedCommand is None:
                        update = COMMAND_QUEUE.get()
                        if update[0] == "mutable_move_jp":
                            target = update[1]
                            # addToLogQueue(CS_A, f"Updated mutable_move_jp target to {target}")
                            logEvent(CS_A, LogEvents.ARM_COMMAND_MUTABLE_UPDATE, target, f"Updated mutable_move_jp target to {target}")
                        else:
                            bufferedCommand = update

                    safeMove(target)

                case "idle":
                    # behaviour when no commands are recieved for an extended period
                    # right now, just go home and open the gripper
                    if not gripperOpen:
                        robot.open_gripper()
                        gripperOpen = True
                    robot.move(robot.get_home_pose())

            # log successful execution
            # addToLogQueue(CS_A, "Command executed successfully")
            sendPractableMessage(CS_A, {
                "executing":com,
                "state":"complete"
            })

            # notify watcher
            commandExecuted.set()

            # need to reset command to None
            command = None

        # main exceptions 

        # NOTE
        # my obersvations from the arm connection method:
        # the arm appears to allow at any given time:
        #   a single fully alive and served connection (MAIN)
        #   2 (more) alive but hung connections (QUEUED)

        # MAIN and QUEUED connections show the message "connected on port xxxx" when made
        # any more attempted connections simply hang with no message (HUNG)

        # if the MAIN connection is closed successfully, 
        # one of the QUEUED connections becomes the MAIN
        # and a new QUEUED slot opens up

        # !!!HOWEVER!!!
        # the TCP connections are NEVER timedout by the arm, 
        # so if the MAIN connection drops on the client pc (unplugged cable, etc)
        # the connection is PERMINANTLY BROKEN because the arm still considers it the MAIN
        # and refuses any others until it is closed sucessfully which is now impossible
        # as the client has droppped the connection 

        # if an attempted connection fails (cable out or arm off), two different exceptions can occur
        # if QUEUED or MAIN connection (message "connected on port xxxx" shows), we get HostNotReachable
        # if HUNG connection (nothing shows, just hangs), we get ClientNotConnected

        # RAISED WHEN:
        #   bad command (move with invalid coords etc)
        except pn.api.exceptions.NiryoRobotException as e:
            # addToLogQueue(CS_A, f"NiryoRobotException on command {command}. Command skipped")
            logEvent(CS_A, LogEvents.EXCEPTION_ARM, f"{e}-{command}", f"NiryoRobotException on command {command}. Command skipped")

            # in this case, nothing is wrong connection wise
            # we simply need to skip the last command
            command = None


        # RAISED WHEN:
        #   bad ip address on connection attempt
        #   cable unplugged on connection attempt
        #   call function on properly disconnected robot variable
        #       can filter for by setting to None
        #       or checking type(robot)
        #   the robot is turned off
        #   QUEUED connection disconnected, sometimes (see above)
        except pn.api.exceptions.ClientNotConnectedException as e:
            # addToLogQueue(CS_A, f"ClientNotConnectedException encountered on command {command}. Attempting to reconnect...")
            logEvent(CS_A, LogEvents.EXCEPTION_ARM, f"{e}-{command}", f"ClientNotConnectedException encountered on command {command}. Attempting to reconnect...")
            # in this case, we need to attempt to re-connect to the arm
            # every time you try to connect to the arm, there is a chance the process will hang
            # therefore, we use another temporary thread to attempt the connection

            # reset robot to None to try to kill old connection
            robot = None

            # we use a while true as we want to keep trying to connect
            while True:

                connectionThread = threading.Thread(target=armConnectHelper, args=[])
                connectionThread.daemon = True
                connectionThread.start()

                while not connectionAttemptEnded.wait(timeout=30):
                    # we are likely hanging indefinitely at this point.
                    # do logging stuff and keep waiting
                    # print(f"{CS_A}CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # addToLogQueue(CS_A, f"CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    logEvent(CS_A, LogEvents.ARM_CONNECTION_ATTEMPT_TIMEOUT, 30, f"CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # log stuff here

                # we have triggered the event, now we need to reset it
                connectionAttemptEnded.clear()

                # the connection attempt has ended, but it could have encountered an exception
                if exc is None:
                    # successfully connected
                    # print(f"{CS_A}SUCCESSFULLY CONNECTED TO ARM")
                    # addToLogQueue(CS_A, f"SUCCESSFULLY CONNECTED TO ARM")
                    logEvent(CS_A, LogEvents.ARM_CONNECTION_SUCCESS, ROBOT_IP, f"SUCCESSFULLY CONNECTED TO ARM AT {ROBOT_IP}")
                    setupRobot()
                    break
                else:
                    # there's an exception, try again in 10 secs
                    # print(f"{CS_A}ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    # addToLogQueue(CS_A, f"ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    logEvent(CS_A, LogEvents.EXCEPTION_ARM, e, f"ARM CONNECTION THREAD ENCOUNTERED EXCEPTION {exc} RETRYING IN 10 SECS")
                    time.sleep(10)

                    # remember to reset exc
                    exc = None
            

        # RAISED WHEN:
        #   call function on previously connected but now disconnected robot
        #       this means the connection is permanently deadlocked. Too bad!
        #   HUNG connection disconnected, sometimes (see above)
        except pn.api.exceptions.HostNotReachableException as e:
            # print(f"{CS_A}HostNotReachable RAISED! VERY LIKELY TO HANG! Attempting to reconnect:")
            # addToLogQueue(CS_A, f"HostNotReachable RAISED! VERY LIKELY TO HANG! Attempting to reconnect:")
            logEvent(CS_A, LogEvents.EXCEPTION_ARM, e, f"HostNotReachable RAISED! VERY LIKELY TO HANG! Attempting to reconnect:")
            
            # the rest is the same as above

            # reset robot to None to try to kill old connection
            robot = None

            # we use a while true as we want to keep trying to connect
            while True:

                connectionThread = threading.Thread(target=armConnectHelper, args=[])
                connectionThread.daemon = True
                connectionThread.start()

                while not connectionAttemptEnded.wait(timeout=30):
                    # we are likely hanging indefinitely at this point.
                    # do logging stuff and keep waiting
                    # print(f"{CS_A}CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # addToLogQueue(CS_A, f"CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    logEvent(CS_A, LogEvents.ARM_CONNECTION_HANGING, 30, f"CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # log stuff here
                
                # we have triggered the event, now we need to reset it
                connectionAttemptEnded.clear()

                # the connection attempt has ended, but it could have encountered an exception
                if exc is None:
                    # successfully connected
                    print(f"{CS_A}SUCCESSFULLY CONNECTED TO ARM")
                    # addToLogQueue(CS_A, f"SUCCESSFULLY CONNECTED TO ARM")
                    logEvent(CS_A, LogEvents.ARM_CONNECTION_SUCCESS, ROBOT_IP, f"SUCCESSFULLY CONNECTED TO ARM")
                    setupRobot()
                    break
                else:
                    # there's an exception, try again in 10 secs
                    # print(f"{CS_A}ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    # addToLogQueue(CS_A, f"ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    logEvent(CS_A, LogEvents.EXCEPTION_ARM, e, f"ARM CONNECTION THREAD ENCOUNTERED EXCEPTION {exc} RETRYING IN 10 SECS")
                    time.sleep(10)

                    # remember to reset exc
                    exc = None

            

        # RAISED WHEN:
        #   currently unknown. susptected to be internal only.
        #   added for posterity and in case it triggers
        except pn.api.exceptions.TcpCommandException as e:
            # print(f"{CS_A}TcpCommandException {e} encountered. HOW DID THIS HAPPEN?")
            # addToLogQueue(CS_A, f"TcpCommandException {e} encountered. HOW DID THIS HAPPEN?")
            logEvent(CS_A, LogEvents.EXCEPTION_ARM, e, f"TcpCommandException encountered. HOW DID THIS HAPPEN?")
            pass

    # code here runs if the thread is killed
    # print(f"{CS_A}ARM THREAD KILLED")
    # addToLogQueue(CS_A, f"ARM THREAD KILLED")
    logEvent(CS_A, LogEvents.THREAD_DIED, CS_A, f"ARM THREAD KILLED")




# ====================
# | MAIN THREAD CODE |
# ====================

practableThread = threading.Thread(target=practableThreadFunction, args=[])
practableThread.start()

armThread = threading.Thread(target=armThreadFunction, args=[])
armThread.start()

# LOG SETUP TODO try catch
# check log directory exists, make if not
if not os.path.exists(LOG_DIRECTORY):
    os.makedirs(LOG_DIRECTORY)

# create logfile and write header
logFileName = os.path.join(LOG_DIRECTORY, f"LOG-{time.strftime("%Y-%d-%m-%H-%M-%S",PROGRAM_START_TIME)}.log")
with open(logFileName, "at") as logFile:
    logFile.write("# ========== Practable - Niryo Control Server ==========\n")
    logFile.write("!HEADER:\n")
    logFile.write(f"startTime: {time.strftime("%Y-%d-%mT%H:%M:%SZ",PROGRAM_START_TIME)}\n")


# need to do SOMETHING with the main thread, otherwise we just instantly close
while True:
    masterThreadEvent.wait(timeout=15)
    # main maintainance loop
    # check health of other threads
    if not practableThread.is_alive():
        # print(f"{CS_M}PRACTABLE THREAD DEAD - RESTARTING...")
        # addToLogQueue(CS_M, f"PRACTABLE THREAD DEAD - RESTARTING...")
        logEvent(CS_M, LogEvents.THREAD_DIED, CS_P, f"PRACTABLE THREAD DEAD - RESTARTING...")
        practableThread = threading.Thread(target=practableThreadFunction, args=[])
        practableThread.start()
        # print(f"{CS_M}PRACTABLE THREAD RESTARTED")
        # addToLogQueue(CS_M, f"PRACTABLE THREAD RESTARTED")
        logEvent(CS_M, LogEvents.THREAD_STARTED, CS_P, f"PRACTABLE THREAD RESTARTED")
        

    if not armThread.is_alive():
        # print(f"{CS_M}ARM THREAD DEAD - RESTARTING...")
        # addToLogQueue(CS_M, f"ARM THREAD DEAD - RESTARTED")
        logEvent(CS_M, LogEvents.THREAD_DIED, CS_A, f"ARM THREAD DEAD - RESTARTED")
        armThread = threading.Thread(target=armThreadFunction, args=[])
        armThread.start()
        # print(f"{CS_M}ARM THREAD RESTARTED")
        # addToLogQueue(CS_M, f"ARM THREAD RESTARTED")
        logEvent(CS_M, LogEvents.THREAD_STARTED, CS_A, f"ARM THREAD RESTARTED")

    # write everything in log queue to log file
    with open(logFileName, "at") as logFile:
        while LOG_QUEUE.qsize() > 0:
            l = LOG_QUEUE.get()
            logFile.write(l)
        logFile.flush()
