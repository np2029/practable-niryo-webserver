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

from time import sleep

import pyniryo as pn  # I appologise an advance for the confusion this will cause
import numpy as np
import threading

import websockets
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

# seconds between sets of connection attempts
CONNECTION_ATTEMPT_COOLDOWN_PRACTABLE = 30

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
    }
}

# global command queue
COMMAND_QUEUE = queue.Queue()

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
    if type(position) == pn.JointsPosition:
        pos = robot.forward_kinematics(position)
    elif type(position) == pn.PoseObject:
        pos = position
    else:
        # incorrect object given
        raise TypeError(f"unsupported type {type(position)} for verifyPosition")

    # must be correct type past this point
    print("verifyPosition: GOT PASSED TYPE CHECK")
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
            #try:
            robot.move(pos)  # re-indent when enabling try/except
            #except pn.api.exceptions.NiryoRobotException as e:
                #print(f"safeMove() - ERROR: encountered exception in safemove: {e}")
            return True
        else:
            print(f"safeMove: Move to position {pos} unsafe, discarded")
            return False
    else:
        raise TypeError(f"unsupported type {type(pos)} for safeMove")

# helper function specifically for sending messages to the practable websocket
# will be used by both the practable thread and arm thread, so should be here
def sendPractableMessage(message):
    if type(message) is not dict:
        print(f"ERROR IN sendPractableMessage: message {message} is not a dict")
        return False

    # convert message into sendable format
    try:  # just in case
        m = json.dumps(message)

    except Exception as e:
        print(f"ERROR IN sendPractableMessage: dict -> json string conversion failed for message: {message}")
        return False
    
    try:
        practable_ws.send(m)
        return True
    
    except websockets.ConnectionClosed as e:
        print(f"ERROR IN sendPractableMessage: Practable connection marked as closed.")
        practableErrorEvent.set()
        return False

    # this should never trigger, but check anyway
    except TypeError as e:
        print(f"ERROR IN sendPractableMessage: Message {message} has invalid type {type(message)}")
        return False
    
    except Exception as e:
        print(f"ERROR IN sendPractableMessage: Exception: {e}")
        return False

# to be called on first connection after robot is turned off
def setupRobot():
    global robot
    global gripperOpen
    gripperOpen = True # annoyingly, we need this variable
    robot.open_gripper()
    robot.move(robot.get_home_pose())
    robot.set_home_pose(pn.JointsPosition(0, 0.5, -1.25, 0,0,0))

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
            print(f"{CS_P}recieved message:\n{incoming}")

            # message verification
            # check if json is valid
            try:
                messageJSON = json.loads(incoming)
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
                        typeCorrectArgs[i] = VALID_COMMANDS[messageJSON["command"]][i](messageJSON[i])
                    except:
                        # type mismatch, invalid command
                        practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: ARGUMENT TYPE MISMATCH: "'+str(messageJSON[i])+' IS NOT TYPE '+str(VALID_COMMANDS[messageJSON["command"]][i])+'}')
                        print(f"{CS_P}ERROR: COMMAND TYPE MISMATCH: {messageJSON[i]} IS NOT TYPE {VALID_COMMANDS[messageJSON["command"]][i]}")
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
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

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
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

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
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                case "gripper_open":
                    COMMAND_QUEUE.put(("gripper_open",None))
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                case "gripper_close":
                    COMMAND_QUEUE.put(("gripper_close",None))
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept
                    
                case "gripper_toggle":
                    COMMAND_QUEUE.put(("gripper_toggle",None))
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                case "gripper_control":
                    COMMAND_QUEUE.put(("gripper_control", typeCorrectArgs))

                case "signal":
                    # dont do anything with the command queue, just print to console and log
                    print(f"{CS_P}Signal Recieved: {typeCorrectArgs["text"]}")
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                case "calibrate":
                    COMMAND_QUEUE.put(("calibrate",None))
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                case "go_home":
                    COMMAND_QUEUE.put(("go_home",None))
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

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
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

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
                    practable_ws.send(f'{{"this is": "an acknoledgement"}}')  #FIXME: send an acknoledgement of reciept

                # this should never trigger, should be filtered out by above. check anyway
                case _:
                    practable_ws.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE VALUE NOT RECOGNISED"}')
                    print(f"{CS_P}congratulations! you did the impossible and triggered the default case in the command match statement! json:\n{messageJSON}")
                    continue

        # technically we only need to catch ConnectionClosed, but better be safe
        except (websockets.ConnectionClosedOK, websockets.ConnectionClosedError ,websockets.ConnectionClosed):
            # check for first time connection
            # if practable_ws is None:
            #     # dont log anything. remember
            # else:
            #     # log a bunch of stuff. its about to get set to None anyway

            # reconnect with the websocket
            # this needs its own while loop and try/except
            practable_ws = None
            while practable_ws is None:
                for i in range(NO_CONNECTION_ATTEMPTS_PRACTABLE):
                    try:
                        practable_ws = connect(PRACTABLE_WEBSOCKET_ADDRESS)
                        # if this doesn't raise an exception, we will reach this
                        break

                    # there are too many exception types to handle individually, just do them all
                    except Exception as e:
                        pass

                # above only breaks the loop, need to check again
                if practable_ws is None:
                    sleep(CONNECTION_ATTEMPT_COOLDOWN_PRACTABLE)

                # if by here a connection has been made, the loop will exit

    # code here runs if the thread is killed
    print(f"{CS_P}PRACTABLE THREAD KILLED")

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

    while not armThreadKillEvent.is_set():
        try:
            # check connection exists
            if robot is None:
                # this will trigger code in the except to establish a connection
                raise pn.api.exceptions.ClientNotConnectedException("robot is None")

            # calibrate if needed
            if robot.need_calibration:
                robot.calibrate_auto()

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
            print(f"{CS_A}executing command: {command}")
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
                        else:
                            bufferedCommand = update

                    robot.move(target)

                case "mutable_move_jp":
                    target = args
                    # check for updates to the target
                    while COMMAND_QUEUE.qsize() > 0 and bufferedCommand is None:
                        update = COMMAND_QUEUE.get()
                        if update[0] == "mutable_move_jp":
                            target = update[1]
                        else:
                            bufferedCommand = update

                    robot.move(target)

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
            # in this case, nothing is wrong connection wise
            # we simply need to skip the last command
            command = None

            # and do the appropriate logging/printing/replying
            print(f"{CS_A}NiryoRobotException on command {command}. Details:\n{e}")

        # RAISED WHEN:
        #   bad ip address on connection attempt
        #   cable unplugged on connection attempt
        #   call function on properly disconnected robot variable
        #       can filter for by setting to None
        #       or checking type(robot)
        #   the robot is turned off
        #   QUEUED connection disconnected, sometimes (see above)
        except pn.api.exceptions.ClientNotConnectedException as e:
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
                    print(f"{CS_A}CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # log stuff here

                # we have triggered the event, now we need to reset it
                connectionAttemptEnded.clear()

                # the connection attempt has ended, but it could have encountered an exception
                if exc is None:
                    # successfully connected
                    print(f"{CS_A}SUCCESSFULLY CONNECTED TO ARM")
                    setupRobot()
                    break
                else:
                    # there's an exception, try again in 10 secs
                    print(f"{CS_A}ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    time.sleep(10)

                    # remember to reset exc
                    exc = None
            

        # RAISED WHEN:
        #   call function on previously connected but now disconnected robot
        #       this means the connection is permanently deadlocked. Too bad!
        #   HUNG connection disconnected, sometimes (see above)
        except pn.api.exceptions.HostNotReachableException as e:
            print(f"{CS_A}HostNotReachable RAISED! VERY LIKELY TO HANG! Attempting to reconnect:")
            
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
                    print(f"{CS_A}CONNECTION THREAD TIMED OUT - LIKELY HANGING INDEFINITELY")
                    # log stuff here
                
                # we have triggered the event, now we need to reset it
                connectionAttemptEnded.clear()

                # the connection attempt has ended, but it could have encountered an exception
                if exc is None:
                    # successfully connected
                    print(f"{CS_A}SUCCESSFULLY CONNECTED TO ARM")
                    setupRobot()
                    break
                else:
                    # there's an exception, try again in 10 secs
                    print(f"{CS_A}ARM CONNECTION THREAD ENCOUNTERED EXCEPTION:\n{exc}\n RETRYING IN 10 SECS")
                    time.sleep(10)

                    # remember to reset exc
                    exc = None

            

        # RAISED WHEN:
        #   currently unknown. susptected to be internal only.
        #   added for posterity and in case it triggers
        except pn.api.exceptions.TcpCommandException as e:
            print(f"{CS_A}TcpCommandException {e} encountered. HOW DID THIS HAPPEN?")
            pass

    # code here runs if the thread is killed
    print(f"{CS_A}ARM THREAD KILLED")




# ====================
# | MAIN THREAD CODE |
# ====================

practableThread = threading.Thread(target=practableThreadFunction, args=[])
practableThread.start()

armThread = threading.Thread(target=armThreadFunction, args=[])
armThread.start()


# need to do SOMETHING with the main thread, otherwise we just instantly close
while True:
    masterThreadEvent.wait(timeout=15)
    # main maintainance loop
    # check health of other threads
    if not practableThread.is_alive():
        practableThread = threading.Thread(target=practableThreadFunction, args=[])
        practableThread.start()

    if not armThread.is_alive():
        armThread = threading.Thread(target=armThreadFunction, args=[])
        armThread.start()
