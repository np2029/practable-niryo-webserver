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

# config variables
# IP address for the robot
# ROBOT_IP = "10.10.10.10"         # wifi hotspot
ROBOT_IP = "169.254.200.200"   # ethernet cable

# number of attempts to connect to the arm
NO_CONNECTION_ATTEMPTS = 3

# localhost websocket port to send and recieve data to/from Practable.io
PRACTABLE_WEBSOCKET_ADDRESS = "ws://localhost:8888/ws/data" # TODO: this is a guess. verify.

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

# arm related variables
joints = robot.get_joints()

# VERY important function.
# returns whether the given pose is a valid (and SAFE) position.
# get this function right, or be ready to pay 4 grand when someone breaks the arm
def verifyPose(pose):
    # return True # FOR TESTING
    # rough check
    # 1: check the pose x,y,z values against tcp limits
    return (pose.x <= TCP_LIMIT_UPPER_X and 
            pose.x >= TCP_LIMIT_LOWER_X and

            pose.y <= TCP_LIMIT_UPPER_Y and
            pose.y >= TCP_LIMIT_LOWER_Y and

            pose.z <= TCP_LIMIT_UPPER_Z and
            pose.z >= TCP_LIMIT_LOWER_Z
            )

    # precise check
    # 1: calculate bounds of the physical gripper from the tcp position
    # 2: check the verts of the bounding box against the tcp limits

# forward kinematics is fast. just feed it into the above
def verifyJointposition(jointPos):
    return verifyPose(robot.forward_kinematics(jointPos))
    


# rotates the joints to the given jointposition
# NOTE: needs to have a try/catch for hostnotreachable in case it disconnects for some reason
def moveJointposition(jp):
    if (verifyJointposition(jp)):
        robot.move(jp)


# for conveinience. Just calls the above but takes degrees. might not be used outside of testing
# def moveJointAngleDeg(angles):
#     moveJointAngleRad(list(map(lambda x: math.radians(x), angles)))



def movePose(p):
    if (verifyPose(p)):
        robot.move(p)




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
                websoc.send({"message":"Error: Command was not valid JSON"})

            # we now have valid json. interperate it.
            # TESTING
            if (responseJSON["command"] == "rotateJoint"):
                rotateJoint(int(responseJSON["joint"]),float(responseJSON["rad"]))
            

        

asyncio.run(dataHandler())