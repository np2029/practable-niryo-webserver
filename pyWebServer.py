# Practable.io - Niryo Ned 2 Robotic Manipulator Middleware Control Server
# Written by Nathan Page As part of a MEng Software Engineering Degree at Heriot Watt University

# This program integrates a Niryo Ned 2 Robotic Manipulator with the the Practable.io remote lab system.
# For more information, visit https://practable.io
# -------------------------------------------------------------------------------------------------------

# imports
import json
import datetime
import queue

from time import sleep

import pyniryo as pn  # I appologise an advance for the confusion this will cause
import numpy as np
import threading

from websockets.sync.client import connect
from scipy.spatial.transform import Rotation as R

# =============
# | CONSTANTS |
# =============
# IP address for the robot
# ROBOT_IP = "10.10.10.10"      # wifi hotspot
ROBOT_IP = "169.254.200.200"    # ethernet cable

# number of attempts to connect to the arm
NO_CONNECTION_ATTEMPTS_ARM = 3

# number of connection attempts to the practable websocket
NO_CONNECTION_ATTEMPTS_PRACTABLE = 3

# localhost websocket port to send and recieve data to/from Practable.io
PRACTABLE_WEBSOCKET_ADDRESS = "ws://localhost:8888/ws/data"


# TCP limits in meters
TCP_LIMIT_UPPER_X = 0.490
TCP_LIMIT_LOWER_X = -0.490

TCP_LIMIT_UPPER_Y = 0.490
TCP_LIMIT_LOWER_Y = -0.490

TCP_LIMIT_UPPER_Z = 0.490
TCP_LIMIT_LOWER_Z = 0.0005 # 0 is barely safe on a flat table. Recomend >= 0.0005

# callsigns for the different threads. Prepend these to every print
# master thread
CS_M = "M_ "
# practable thread
CS_P = "P_ "
# arm thread
CS_A = "A_ "

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
    "update_jog_tcp":{
        "x":float,
        "y":float,
        "z":float,
        "roll":float,
        "pitch":float,
        "yaw":float
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
        "ammount":int
    },
    "signal":{
        "text":str
    },
    "calibrate":{},
    "go_home":{}
}

COMMAND_QUEUE = queue.Queue()

# ===========
# | CLASSES |
# ===========


# ====================
# | HELPER FUNCTIONS |
# ====================

# VERY important function.
# returns whether the given pose is a valid (and SAFE) position.
# get this function right, or be ready to pay 4 grand when someone breaks the arm
# accepts JointPosition or PoseObject objects
# FIXME: this needs to check for custom areas
def verifyPosition(position):
    # check type of pos.
    # make a new object to not mutate the supplied one
    if type(position) == type(pn.JointsPosition):
        pos = robot.forward_kinematics(position)
    elif type(position) != type(pn.PoseObject):
        # incorrect object given
        raise TypeError(f"unsupported type {type(pos)} for verifyPosition")

    pos = position
    # must be correct type past this point

    # 1: calculate bounds of the physical gripper from the tcp position
    GRIPPER_WIDTH = 0.08# meters, 80mm
    GRIPPER_HEIGHT = 0.028# meters, 28mm
    gripperBounds = np.array([
        [0,GRIPPER_WIDTH/2,GRIPPER_HEIGHT/2],
        [0,-GRIPPER_WIDTH/2,GRIPPER_HEIGHT/2],
        [0,-GRIPPER_WIDTH/2,-GRIPPER_HEIGHT/2],
        [0,GRIPPER_WIDTH/2,-GRIPPER_HEIGHT/2]
    ])

    # apply roll, pitch, and yaw to bounds
    rot =  R.from_euler("xyz", [pos.roll, pos.pitch, pos.yaw], degrees=False)
    updatedBounds = rot.apply(gripperBounds)

    # add tcp x,y,z as offsets to get real coords
    for i in range(len(updatedBounds)):
        updatedBounds[i][0] += pos.x
        updatedBounds[i][1] += pos.y
        updatedBounds[i][2] += pos.z

    # 2: check the verts of the bounding box against the tcp limits
    print("CHECKING BOUNDS: "+str(updatedBounds))
    print("FOR POSE: "+str(pos))
    for i in range(len(updatedBounds)):
        if (updatedBounds[i][0] < TCP_LIMIT_LOWER_X
            or updatedBounds[i][0] > TCP_LIMIT_UPPER_X

            or updatedBounds[i][1] < TCP_LIMIT_LOWER_Y
            or updatedBounds[i][1] > TCP_LIMIT_UPPER_Y

            or updatedBounds[i][2] < TCP_LIMIT_LOWER_Z
            or updatedBounds[i][2] > TCP_LIMIT_UPPER_Z
            ):
            print("BOUNDS CHECK FAILED")
            return False
    # for loop ended, NOW we can call success
    print("BOUNDS CHECK SUCCEEDED")
    return True

# returns true/false on success/failure
def safeMove(pos):
    # check arg type
    if type(pos) == pn.JointsPosition or type(pos) == pn.PoseObject:
        print(f"safeMove: verifying move to\n{pos}")
        if verifyPosition(pos):
            print(f"safeMove: move verified, moving to {pos}")
            return True
        else:
            print(f"safeMove: Move to position {pos} unsafe, discarded")
            return False
    else:
        raise TypeError(f"unsupported type {type(pos)} for safeMove")


# # verify a command is correct
# def verifyCommandString(com):
#     # check it's actually json
#     try:
#         comJSON = json.loads(com)
#     except json.decoder.JSONDecodeError:
#         return (False,'{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: BAD JSON - FAILED TO DECODE"}')
#     # check the 'command' field is valid
#     # check the number of args
#     # check the data type of args
#     # return success/failure and message


# ==================
# | INITIALISATION |
# ==================

# TODO: checka and open log file
# TODO: check and read config file


print("----- ARM INITIALISATION BEGINING -----")
print("ARM INITIALISATION: Attempting arm connection on ip {ROBOT_IP}")
# attempt to connect to the arm
robot = None
for i in range(NO_CONNECTION_ATTEMPTS_ARM):
    if robot == None:
        try:
            robot = pn.NiryoRobot(ROBOT_IP)

        except pn.api.exceptions.ClientNotConnectedException:
            print(f"WARNING: failed connection attempt {i+1} to {ROBOT_IP}")

# check if connection successful
if (robot != None):
    print(f"ARM INITIALISATION: Connection to {ROBOT_IP} successful")
else:
    print(f"ERROR: Could not connect to {ROBOT_IP}")
    exit(False) # FIXME: don't just exit, restart every 60s or so.

# calibrate arm
print("ARM INITIALISATION: Auto calibrating arm")
robot.calibrate_auto()

# hard code home pose and move there immediately
homePose = robot.forward_kinematics(pn.JointsPosition(0,0.5,-1.25,0,0,0))
robot.set_home_pose(0, 0.5, -1.25, 0, 0, 0)
print("ARM INITIALISATION: Moving to home pose")
robot.move_to_home_pose()

# open gripper and save gripper state
# it should be open anyway, but just to make sure
print("ARM INITIALISATION: Opening gripper")
robot.open_gripper()
gripperOpen = True # annoyingly, we need this variable

print("----- ARM INITIALISATION COMPLETE -----")

print("----- PRACTABLE THREAD INITIALISATION BEGINING -----")

def practableThreadFunction():
    practable_ws = None
    # need to wrap in try/catch so finally always happens
    try:
        print(f"{CS_P}Attempting to connect to {PRACTABLE_WEBSOCKET_ADDRESS}")
        for i in range(NO_CONNECTION_ATTEMPTS_PRACTABLE):
            try:
                practable_ws = connect(PRACTABLE_WEBSOCKET_ADDRESS)
                # if we get here we have a connection.
                pthreadInit.set()
                break

            except Exception as e:
                print(f"{CS_P}Failed attempt {i} at connecting to {PRACTABLE_WEBSOCKET_ADDRESS}")
            
        # test connection success
        if practable_ws is None:
            raise Exception(f"{CS_P}failed {NO_CONNECTION_ATTEMPTS_PRACTABLE} connection attempts. exiting.")
        else:
            # we have a valid (for now) connection. lets use it
            try:
                # main loop. 
                # might need to add an interrupt feature/variable later
                while True:
                    # wait for an incoming message
                    incoming = practable_ws.recv()
                    print(f"{CS_P}recieved message:\n{incoming}")

                    # message verification
                    # check if json is valid
                    try:
                        messageJSON = json.loads(messageJSON)
                    except json.decoder.JSONDecodeError:
                        # command was bad json. send a reply stating as such
                        practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: BAD JSON - FAILED TO DECODE"}')
                        print(f"{CS_P}RECEIVED BAD COMMAND: {messageJSON}")
                        continue

                    # json is correct, check if command variable exists
                    if "command" not in messageJSON:
                        practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE NOT SET FOR RECIEVED COMMAND"}')
                        print(f"{CS_P}ERROR: COMMAND NOT SET IN INCOMING JSON: {messageJSON}")
                        continue

                    # command exists, check if its a real command
                    if messageJSON["command"] not in VALID_COMMANDS:
                        # command not present. reply with error
                        practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"}')
                        print(f"{CS_P}ERROR: COMMAND NOT RECOGNISED: {messageJSON}")
                        continue

                    # command is real, check the args
                    typeCorrectArgs = {}
                    for i in messageJSON:
                        if i != "command":
                            try:
                                typeCorrectArgs[i] = VALID_COMMANDS[i](messageJSON[i])
                            except:
                                # type mismatch, invalid command
                                practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: ARGUMENT TYPE MISMATCH: "'+str(messageJSON[i])+' IS NOT TYPE '+str(VALID_COMMANDS[messageJSON["command"]][i])+'}')
                                print(f"{CS_P}ERROR: COMMAND TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}")
                                continue

                    # need to check again, previous continue just breaks the arg check loop
                    if len(typeCorrectArgs) != len(messageJSON)-1:  # -1 to account for missing "command"
                        continue

                    # NOW we have a 100% valid command and args. we can finally submit the command
                    COMMAND_QUEUE.put()
                    match messageJSON["command"]:
                        case "move_tcp":
                            pass

                        case "move_jp":
                            pass
                            COMMAND_QUEUE.put(
                                (robot.move, 
                                 messageJSON["j0"],
                                 messageJSON["j1"],
                                 messageJSON["j2"],
                                 messageJSON["j3"],
                                 messageJSON["j4"],
                                 messageJSON["j5"]
                                 )
                            )

                        case "update_jog_tcp":
                            pass

                        case "update_jog_jp":
                            pass

                        case "gripper_open":
                            pass

                        case "gripper_close":
                            pass

                        case "gripper_toggle":
                            pass

                        case "gripper_control":
                            pass

                        case "signal":
                            pass

                        case "calibrate":
                            pass

                        case "gohome":
                            pass

                        # this should never trigger, should be filtered out by above. check anyway
                        case _:
                            practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"}')
                            print(f"{CS_P}congratulations! you did the impossible and triggered the default case in the command match statement! json:\n{messageJSON}")
                            continue


            except Exception as e:
                # TODO: handle/log/display errors
                pass

            finally:
                practable_ws.close()

    except Exception as e:
        print("ERROR IN PRACTABLE THREAD INITIALISATION:")
        print(e)
    
    finally:
        # need this to prevent deadlock
        pthreadInit.set()

pthreadInit = threading.Event()
practableThread = threading.Thread(target=practableThreadFunction, args=[])
practableThread.start()
pthreadInit.wait()
print("----- PRACTABLE THREAD INITIALISATION COMPLETE -----")
print("----- ARM THREAD INITIALISATION BEGINING -----")

def armThreadFunction():
    try:
        # this needs aditional safety stuff, but I need to test it for now
        bufferedCommand = None
        armThreadInit.set()
        while True:
            command = COMMAND_QUEUE.get()

            # execute command. implement fully later
            print(f"{CS_A}executing command: {command}")
            command[0](*command[1])

    except:
        # TODO: handle/log/display errors
        pass

    finally:
        armThreadInit.set()

armThreadInit = threading.Event()
armThread = threading.Thread(target=armThreadFunction, args=[])
armThread.start()
armThreadInit.wait()

print("----- ARM THREAD INITIALISATION COMPLETE -----")

# need to do SOMETHING with the main thread, otherwise we just instantly close
while True:
    sleep(3)