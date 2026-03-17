# Python webserver (sort of)
# written by Nathan Page 
# for use in my university dissertation (Dissertation Title TBA)

# This program should act as a bridge between the Practable.io software and Niryo Ned 2 Arm hardware.
# Data is transmitted to/from the arm via inter/ethernet (TBD) 
# and sent to the Practable.io API via a localhost websocket

# Note that this file internally uses niryo data structures
# only on data in and out are standard arrays/JSON etc actually used

# imports
import pyniryo as pn
from pyniryo import NiryoRobot

import asyncio
import websockets as ws
from websockets.asyncio.client import connect as wsconnect

import math
import json
import datetime
import numpy as np
# import scipy.spatial.transform as sp
from scipy.spatial.transform import RigidTransform as Tf
from scipy.spatial.transform import Rotation as R

# config variables
# IP address for the robot
# ROBOT_IP = "10.10.10.10"         # wifi hotspot
ROBOT_IP = "169.254.200.200"   # ethernet cable

# number of attempts to connect to the arm
NO_CONNECTION_ATTEMPTS = 3

# localhost websocket port to send and recieve data to/from Practable.io
PRACTABLE_WEBSOCKET_ADDRESS = "ws://localhost:8888/ws/data" # TODO: this is a guess. verify.
# PRACTABLE_WEBSOCKET_ADDRESS = "ws://localhost:9999" # TODO: this is a guess. verify.

# TCP limits
# TODO: verify units
TCP_LIMIT_UPPER_X = 490
TCP_LIMIT_LOWER_X = -490

TCP_LIMIT_UPPER_Y = 490
TCP_LIMIT_LOWER_Y = -490

TCP_LIMIT_UPPER_Z = 490
TCP_LIMIT_LOWER_Z = 10      # NOTE: 0 is barely safe on the table, limiting to 10 for safety buffer

# code begins here

# attempt to connect to the arm
robot = None
for i in range(NO_CONNECTION_ATTEMPTS):
    if robot == None:
        try:
            robot = NiryoRobot(ROBOT_IP)

        except pn.api.exceptions.ClientNotConnectedException:
            print(f"WARNING: failed connection attempt to {ROBOT_IP}")

# check if connection successful
if (robot != None):
    print(f"connection to {ROBOT_IP} successful")
else:
    print(f"ERROR: Could not connect to {ROBOT_IP}")
    exit(False)

# from this point, assume connection to arm successful

# calibrate arm
robot.calibrate_auto()

# save home pose now
homePose = robot.get_pose()

# open gripper and save state
# it should be open anyway, but just to make sure
robot.open_gripper()
gripperOpen = True

# need this to freeze the arm
# if unix time is less than this, all commands will be rejected
frozenTime = 0

# arm related variables
# joints = robot.get_joints()

# VERY important function.
# returns whether the given pose is a valid (and SAFE) position.
# get this function right, or be ready to pay 4 grand when someone breaks the arm
def verifyPose(pose):
    # return True # FOR TESTING
    # rough check
    # 1: check the pose x,y,z values against tcp limits
    # return (pose.x <= TCP_LIMIT_UPPER_X and 
    #         pose.x >= TCP_LIMIT_LOWER_X and

    #         pose.y <= TCP_LIMIT_UPPER_Y and
    #         pose.y >= TCP_LIMIT_LOWER_Y and

    #         pose.z <= TCP_LIMIT_UPPER_Z and
    #         pose.z >= TCP_LIMIT_LOWER_Z
    #         )

    # precise check
    # 1: calculate bounds of the physical gripper from the tcp position
    GRIPPER_WIDTH = 80# mm
    GRIPPER_HEIGHT = 28# mm
    gripperBounds = np.array([
        [0,GRIPPER_WIDTH/2,GRIPPER_HEIGHT/2],
        [0,-GRIPPER_WIDTH/2,GRIPPER_HEIGHT/2],
        [0,-GRIPPER_WIDTH/2,-GRIPPER_HEIGHT/2],
        [0,GRIPPER_WIDTH/2,-GRIPPER_HEIGHT/2]
    ])

    # apply roll, pitch, and yaw to bounds
    rot =  R.from_euler("xyz", [pose.roll, pose.pitch, pose.yaw], degrees=False)
    updatedBounds = rot.apply(gripperBounds)

    # add tcp x,y,z as offsets to get real coords
    for i in range(len(updatedBounds)):
        updatedBounds[i][0] += pose.x
        updatedBounds[i][1] += pose.y
        updatedBounds[i][2] += pose.z

    # 2: check the verts of the bounding box against the tcp limits
    for i in range(len(updatedBounds)):
        if (updatedBounds[i][0] < TCP_LIMIT_LOWER_X
            or updatedBounds[i][0] > TCP_LIMIT_UPPER_X

            or updatedBounds[i][1] < TCP_LIMIT_LOWER_Y
            or updatedBounds[i][1] > TCP_LIMIT_UPPER_Y

            or updatedBounds[i][2] < TCP_LIMIT_LOWER_Z
            or updatedBounds[i][2] > TCP_LIMIT_UPPER_Z
            ):
            return False
        else:
            return True

# forward kinematics is fast. just feed it into the above
def verifyJointposition(jointPos):
    return verifyPose(robot.forward_kinematics(jointPos))
    


# rotates the joints to the given jointposition
# NOTE: needs to have a try/catch for hostnotreachable in case it disconnects for some reason
def moveJointposition(jp):
    if (verifyJointposition(jp)):
        robot.move(jp)
        return True
    else:
        return False


# for conveinience. Just calls the above but takes degrees. might not be used outside of testing
# def moveJointAngleDeg(angles):
#     moveJointAngleRad(list(map(lambda x: math.radians(x), angles)))



def movePose(p):
    if (verifyPose(p)):
        robot.move(p)
        return True
    else:
        return False




# purely for testing safely. DO NOT MAKE AVAILABLE TO USERS
def rotateJoint(jointNumber, rotInRad):
    j = robot.get_joints().to_list()
    j[jointNumber] += rotInRad
    robot.move(pn.JointsPosition(j[0],j[1],j[2],j[3],j[4],j[5]))

def setJoint(jointNumber, rotInRad):
    j = robot.get_joints().to_list()
    j[jointNumber] = rotInRad
    robot.move(pn.JointsPosition(j[0],j[1],j[2],j[3],j[4],j[5]))

# setup practable websocket connection
async def dataHandler():
    # establish connection
    async with wsconnect(PRACTABLE_WEBSOCKET_ADDRESS) as websoc:
        # we will stop the loop via a user command or an error
        frozenTime = 0
        while True:
            # when connected, things should always go as follows:
            #   recieve and interperate request from practable
            #   if command is good, send to arm
            #   send success or fail message back to practable

            response = await websoc.recv()
            print (response)# TESTING AND DEBUG

            # convert response from json to python dict
            try:
                responseJSON = json.loads(response)
            except json.decoder.JSONDecodeError:
                # command was bad json. send a reply stating as such
                websoc.send({"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: BAD JSON - FAILED TO DECODE"})
                continue

            # we now have valid json. interperate it.
            try:
                command = responseJSON["command"]
            except KeyError:
                # command not present. reply with error
                websoc.send({"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE NOT SET FOR RECIEVED COMMAND"})
                continue

            # check for frozen status
            if (int(datetime.datetime.now().timestamp()) < frozenTime):
                # arm is frozen, reject command and continue
                websoc.send({"replyComm":command,"result":"fail","displayText":"Command rejected, the arm is currently frozen","message":"REJECTED COMMAND WHILE FROZEN"})
                continue

            # command variable set. interperate it
            match command:
                case "signal":
                    # command could be malformed. check just to be safe
                    try:
                        print(responseJSON["text"])
                        #print("BEFORE AWAIT")
                        await websoc.send('{"replyComm":"signal","result":"success","displayText":"","message":"Signal recieved successfully"}')
                        #print("AFTER AWAIT")
                        pass
                    except KeyError:
                        print("ERROR: RECIEVED MALFORMED SIGNAL: "+str(responseJSON))
                        await websoc.send('{"replyComm":"NOT_SET","result":"fail","displayText":"Error: Invalid command.","message":"ERROR: COMMAND ATTRIBUTE NOT SET FOR RECIEVED COMMAND"}')

                case "moveTCP":
                    # command could be malformed. check just to be safe
                    try:
                        # print to console
                        print("Recieved moveTCP command: moving to:"
                              +"\nx: "+responseJSON["x"]
                              +"\ny: "+responseJSON["y"]
                              +"\nz: "+responseJSON["z"]
                              +"\nroll: "+responseJSON["roll"]
                              +"\npitch: "+responseJSON["pitch"]
                              +"\nyaw: "+responseJSON["yaw"]
                              )
                        # actually move
                        # NOTE: given values are in cm not mm
                        if (movePose(pn.PoseObject(float(responseJSON["x"]), float(responseJSON["y"]), float(responseJSON["z"]), float(responseJSON["roll"]), float(responseJSON["pitch"]), float(responseJSON["yaw"])))):
                            # move completed successfully
                            await websoc.send('{"replyComm":"moveTCP","result":"success","displayText":"Move Complete","message":"TCP MOVE COMPLETE"}')
                        else:
                            # move failed
                            await websoc.send('{"replyComm":"moveTCP","result":"fail","displayText":"Move Failed: Location Invalid","message":"TCP MOVE FAIL - INVALID LOCATION"}')
                    
                    except KeyError:
                        # malformed command
                        print("ERROR: RECIEVED MALFORMED moveTCP: "+str(responseJSON))
                        await websoc.send('{"replyComm":"moveTCP","result":"fail","displayText":"Error: moveTCP command is missing required arguments.","message":"ERROR: moveTCP COMMAND IS MISSING REQUIRED ARGUMENTS"}')
                
                case "moveJoints":
                    # command could be malformed. check just to be safe
                    try:
                        # print to console
                        print("Recieved moveJoints command: moving to:"
                              +"\nj0: "+responseJSON["j0"]
                              +"\nj1: "+responseJSON["j1"]
                              +"\nj2: "+responseJSON["j2"]
                              +"\nj3: "+responseJSON["j3"]
                              +"\nj4: "+responseJSON["j4"]
                              +"\nj5: "+responseJSON["j5"]
                              )
                        # actually move
                        # NOTE: given values are in cm not mm
                        if (moveJointposition(pn.PoseObject(float(responseJSON["j0"]), float(responseJSON["j1"]), float(responseJSON["j2"]), float(responseJSON["j3"]), float(responseJSON["j4"]), float(responseJSON["j5"])))):
                            # move completed successfully
                            await websoc.send('{"replyComm":"moveJoints","result":"success","displayText":"Move Complete","message":"JOINTS MOVE COMPLETE"}')
                        else:
                            # move failed
                            await websoc.send('{"replyComm":"moveJoints","result":"fail","displayText":"Move Failed: Location Invalid","message":"JOINTS MOVE FAIL - INVALID LOCATION"}')
                    
                    except KeyError:
                        print("ERROR: RECIEVED MALFORMED moveJoints: "+str(responseJSON))
                        await websoc.send('{"replyComm":"moveJoints","result":"fail","displayText":"Error: moveJoints command is missing required arguments.","message":"ERROR: moveJoints COMMAND IS MISSING REQUIRED ARGUMENTS"}')

                case "callibrate":
                    # no argument, so cannot be malformed.
                    # just call calibrate
                    robot.calibrate_auto()
                    await websoc.send('{"replyComm":"callibrate","result":"success","displayText":"Callibration Complete","message":"ARM CALLIBRATED"}')
                    

                case "goHome":
                    # home position should always be safe, don't bother checking
                    movePose(homePose)
                    print("Moved Home")
                    await websoc.send('{"replyComm":"goHome","result":"success","displayText":"Home Move Complete","message":"HOME MOVE COMPLETE"}')

                case "setGripper":
                    pass
                    # command could be malformed, check just to be safe
                    try:
                        if (responseJSON["state"] == "open"):
                            robot.open_gripper()
                            gripperOpen = True
                            await websoc.send('{"replyComm":"setGripper","result":"success","displayText":"Gripper Opened","message":"GRIPPER OPENED"}')


                        elif (responseJSON["state"] == "close"):
                            robot.close_gripper()
                            gripperOpen = False
                            await websoc.send('{"replyComm":"setGripper","result":"success","displayText":"Gripper Closed","message":"GRIPPER CLOSED"}')

                        elif (responseJSON["state"] == "toggle"):
                            if (gripperOpen):
                                robot.close_gripper()
                                gripperOpen = False
                                await websoc.send('{"replyComm":"setGripper","result":"success","displayText":"Gripper Closed","message":"GRIPPER CLOSED"}')
                            else:
                                robot.open_gripper()
                                gripperOpen = True
                                await websoc.send('{"replyComm":"setGripper","result":"success","displayText":"Gripper Opened","message":"GRIPPER OPENED"}')
                        else:
                            # invalid state given. send fail message
                            print("ERROR: INVALID GRIPPER STATE RECIEVED: "+responseJSON)
                            await websoc.send('{"replyComm":"setGripper","result":"fail","displayText":"Error: Invalid Gripper state","message":"INVALID GRIPPER STATE RECIEVED"}')
                    
                    except KeyError:
                        print("ERROR: RECIEVED MALFORMED setGripper: "+str(responseJSON))
                        await websoc.send('{"replyComm":"setGripper","result":"fail","displayText":"Error: setGripper command is missing required arguments.","message":"ERROR: setGripper COMMAND IS MISSING REQUIRED ARGUMENTS"}')

                case "freeze":
                    # check for malformed message
                    try:
                        frozenTime =  int(datetime.datetime.now().timestamp()) + int(responseJSON["time"])
                        print("ARM FROZEN FOR "+str(int(responseJSON["time"]))+ "SECONDS")
                        await websoc.send('{"replyComm":"freeze","result":"success","displayText":"Arm has been frozen","message":"ARM FROZEN")}')

                        
                    except KeyError:
                        print("ERROR: RECIEVED MALFORMED freeze: "+str(responseJSON))
                        await websoc.send('{"replyComm":"freeze","result":"fail","displayText":"Error: freeze command is missing required arguments.","message":"ERROR: freeze COMMAND IS MISSING REQUIRED ARGUMENTS"}')

            
            # TESTING
            # if (responseJSON["command"] == "rotateJoint"):
            #     rotateJoint(int(responseJSON["joint"]),float(responseJSON["rad"]))
                
            

        

asyncio.run(dataHandler())
