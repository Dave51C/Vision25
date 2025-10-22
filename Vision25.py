# $Source: /home/scrobotics/src/2025/RCS/Vision25.py,v $
# $Revision: 5.9 $
# $Date: 2025/05/04 12:53:25 $
# $Author: scrobotics $
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from math import pi, atan2, asin, sin, cos, sqrt, tan, radians, degrees
import json
import time
import sys
import cv2
import numpy as np
import apriltag
from pprint import pprint
from PiggyVision25 import TAG_SPECS, rotate, Reef, ROBOT_OFFSET, Crosshairs
#from Kevin import Reef

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

from Find_Apriltags_List import *

Red        = (0,0,255)
Orange     = (0,127,255)
Yellow     = (32,255,255)
Green      = (0,255,0)
Blue       = (255,0,0)
White      = (255,255,255)
Black      = (0,0,0)

# Sweetspot axes: Increasing X moves the sweetspot to the tag's left and
# camera's right. Increasing Y move the sweetspot up. Increasing Z moves the
# sweetspot forward from the tag along the normal to the tag face.

# A SWEETSPOT of 0,0,0 means it's at the center of the tag.
# Add to CORNERS for right Reef "trunk" offset. Subtract for left. Omit for
# all other tags
SWEETSPOT     = np.array([6.47, 0.0, 0.0])  
HEIGHT        = np.array([0.0, 40.0, 0.0])
WIDTH         = np.array([3.0, 0.0, 0.0])

# XSECTION defines the cross-section of the box drawn aroung the tag.

XSECTION  = np.array([[ 1.5, 0.0,  0.0],
                      [ 1.5, 0.0, -20.0],
                      [-1.5, 0.0, -20.0],
                      [-1.5, 0.0,  0.0]])

SLOT      = np.array([[-1.5, 10.0,  0.0],
                      [ 1.5, 10.0,  0.0],
                      [ 1.5, -10.0, 0.0],
                      [-1.5, -10.0, 0.0]])
INSET     = np.array([[-1.0, 9.0,  0.0],
                      [ 1.0, 9.0,  0.0],
                      [ 1.0, -9.0, 0.0],
                      [-1.0, -9.0, 0.0]])
#BASE   = OFFSET_1L + XSECTION
BASE   = XSECTION 


def draw_quad (img, combo, color, weight,IDX=0):
    cv2.line(img, combo[0+IDX], combo[4+IDX], Red, weight, 16)
    cv2.line(img, combo[1+IDX], combo[5+IDX], Black, weight, 16)
    cv2.line(img, combo[4+IDX], combo[5+IDX], Yellow, weight, 16)
    cv2.line(img, combo[0+IDX], combo[1+IDX], Green, weight, 16)
    #print ("Red:    :",combo[0+IDX], combo[4+IDX])
    #print ("Black   :",combo[1+IDX], combo[5+IDX])
    #print ("Yellow  :",combo[4+IDX], combo[5+IDX])
    #print ("Greenw  :",combo[0+IDX], combo[1+IDX])
    cv2.line(img, combo[2+IDX], combo[6+IDX], Red, weight, 16)
    cv2.line(img, combo[3+IDX], combo[7+IDX], Black, weight, 16)
    cv2.line(img, combo[6+IDX], combo[7+IDX], Yellow, weight, 16)
    cv2.line(img, combo[2+IDX], combo[3+IDX], Green, weight, 16)


def draw_box(img,base,top,color,weight):
    # bottom
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, base[0], base[1], Blue, weight, 16)
    cv2.line (img, base[1], base[2], Blue, weight, 16)
    cv2.line (img, base[2], base[3], Blue, weight, 16)
    cv2.line (img, base[3], base[0], Blue, weight, 16)
    # top
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, top[0], top[1], color, weight, 16)
    cv2.line (img, top[1], top[2], color, weight, 16)
    cv2.line (img, top[2], top[3], color, weight, 16)
    cv2.line (img, top[3], top[0], color, weight, 16)
    # sides
    # last param (16) is for anti-alias line type LINE_AA
    cv2.line (img, base[0], top[0], color, weight, 16)
    cv2.line (img, base[1], top[1], color, weight, 16)
    cv2.line (img, base[2], top[2], color, weight, 16)
    cv2.line (img, base[3], top[3], color, weight, 16)

def Smiley (output_img,CamMtx):
    center_coordinates = (int(CamMtx['width']/2), int(CamMtx['height']/2) )
    left_eye  = (int(CamMtx['width']/2)-75, int(CamMtx['height']/2)-75 )
    right_eye = (int(CamMtx['width']/2)+75, int(CamMtx['height']/2)-75 )
    thickness          = 15
    axesLength = center_coordinates
    axesLength         = (150, 150)
    angle              = 0
    startAngle         = 20
    endAngle           = 160
    color = (0, 255, 0) 
    output_img = cv2.circle(output_img, left_eye, 35, color, thickness)
    output_img = cv2.circle(output_img, right_eye, 35, color, thickness)
    output_img = cv2.ellipse(output_img, center_coordinates, axesLength, 
        angle, startAngle, endAngle, color, thickness) 

def Publish_Camera_Location (Tag_ID,Tag):
    Source = Tag["Source"]
    ZYX,jac = cv2.Rodrigues(Tag["rvecs"])
    CamPos  = -np.matrix(ZYX).T * np.matrix(Tag["tvecs"])
    CamWorldX  = TAG_SPECS[Tag_ID]["X"]+CamPos[2]
    CamWorldY  = TAG_SPECS[Tag_ID]["Y"]+CamPos[0]
    CamWorldX,CamWorldY = rotate (CamWorldX,CamWorldY,TAG_SPECS[Tag_ID]["X"],
                TAG_SPECS[Tag_ID]["Y"],
                np.radians(TAG_SPECS[Tag_ID]["ZROT"]))
    # determine camera's rotation wrt NORTH
    deltaX = TAG_SPECS[Tag_ID]["X"] - CamWorldX
    deltaY = TAG_SPECS[Tag_ID]["Y"] - CamWorldY
    A2E = atan2(deltaY,deltaX) + Tag["Hdg"]

    # Dont't bother with A2N. Use A2E instead.
    #A2N = A2E - pi/2
    #if A2N < -pi:
    #    A2N = A2N + 2*pi

    RobotWorldX = CamWorldX + ROBOT_OFFSET[Source]["X"]
    RobotWorldY = CamWorldY + ROBOT_OFFSET[Source]["Y"]
    RobotWorldX,RobotWorldY = rotate (RobotWorldX,RobotWorldY,
                CamWorldX,CamWorldY,A2E)

    #RobotWorldX,RobotWorldY = rotate (RobotWorldX,RobotWorldY,
    #            TAG_SPECS[Tag_ID]["X"],
    #            TAG_SPECS[Tag_ID]["Y"],
    #            np.radians(TAG_SPECS[Tag_ID]["ZROT"]))

    pubRobotWorldX.set(RobotWorldX)
    pubRobotWorldY.set(RobotWorldY)
    pubCamWorldX.set(CamWorldX)
    pubCamWorldY.set(CamWorldY)
    #pubCamWorldR.set(A2N)
    pubCamWorldR.set(A2E)
    pubCamRng.set(Tag["Rng"])
    pubCamHdg.set(Tag["Hdg"])
    pubCamPos0.set(CamPos[0])
    pubCamPos1.set(CamPos[1])
    pubCamPos2.set(CamPos[2])

#options  = apriltag.DetectorOptions(
#           families="tag36h11"
#           ,nthreads      = 4
#           ,quad_decimate = 2
#           ,quad_blur     = 0.0
#           ,refine_edges  = True
#           ,refine_decode = True
#           ,refine_pose   = False
#           ,quad_contours = True
#           )
#detector = apriltag.Detector(options)


configFile = "/boot/frc.json"

class CameraConfig: pass     # Is this just to initialize CameraConfig?

team            = None
server          = False
cameraConfigs   = []
switchedCameraConfigs = []
cameras         = []
GameTags        = [ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                  14, 15, 16, 17, 18, 19, 20, 21, 22 ]
CamPos          = np.array([0.0, 0.0, 0.0])
CamPos          = CamPos.reshape(3,1)
CamRange        = 0.0
#DScmd           = "Waiting"
Horizon         = 36.0          # just in case ~/config.json processing fails

try:
    Cfile   = open('config.json','r')
    j       = json.load(Cfile)
    Horizon = j['Horizon']
    IP      = j['IP']
    Logging = j['Logging']
    LogDir  = j['LogDir']
    Cfile.close()
except:
    print ("Failed to process config.json")

if Logging == True:
    try:
        LogName = LogDir+time.strftime('%Y%m%d_%H%M%S')+'.log'
        print ("Redirecting all messages to ",LogName)
        sys.stdout = open(LogName, 'w') 
        sys.stderr = sys.stdout
        print (time.strftime('%H%M%S: starting'))
        sys.stdout.flush()
    except:
        print ("Can't redirect stdout")

rev = "$Revision: 5.9 $"
__version__ = rev.replace("$","",-1).replace("Revision: ","")
print ("Vision25.py version ",__version__)

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")
    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    CamNbr = 0
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False
        else:
            print ("CamNbr {0:d}, Camera: {1:s} path:{2:s}\n".format\
            (CamNbr,cameras[CamNbr]["name"],cameras[CamNbr]["path"]))
            CamNbr += 1

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    print ("TADA!")
    print ("Horizon: ",Horizon)
    print ("IP",IP)
    # IP='10.42.0.1' # Gotten from config.json

    All           = {}
    #Closest_Tag   = {}
    DriverCam     = 0
    RearTagCam    = 1
    FrontTagCam   = 2
    CameraLabel   = ['Driver Cam','Front Tag Cam','Rear Tag Cam']
    imgBuf        = {}
    input_img     = {}
    input_stream  = {}
    output_img    = {}
    output_stream = {}
    detectors     = {}

    Success, CamCnt, CamMtx, CamDetOpts = get_camera_parameters ()

    print (CamCnt,' cameras used')
    if CamCnt != 3:
        print ('#####################################')
        print ("    YOU'RE NOT USING 3 CAMERAS")
        print ('    PROCEED AT YOUR OWN RISK')
        print ('#####################################')
    Camera_Center = (round(CamMtx[0]['mtx'][0][2]),round(CamMtx[0]['mtx'][1][2]))

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("Vision25")
        #ntinst.setServerTeam(team)
        ntinst.setServer(IP)
        #ntinst.startDSClient()
        #table = ntinst.getTable("data")
        #pub1   = table.getDoubleTopic("1").publish()
        #pub2   = table.getDoubleTopic("2").publish()
        CamPos_tbl   = ntinst.getTable("CamPos")  
        pubRobotWorldX = CamPos_tbl.getDoubleTopic("Robot_X").publish()
        pubRobotWorldY = CamPos_tbl.getDoubleTopic("Robot_Y").publish()
        pubCamWorldX = CamPos_tbl.getDoubleTopic("Camera_X").publish()
        pubCamWorldY = CamPos_tbl.getDoubleTopic("Camera_Y").publish()
        pubCamWorldR = CamPos_tbl.getDoubleTopic("Robot_Rot").publish()
        pubCamRng    = CamPos_tbl.getDoubleTopic("Camera_Rng").publish()
        pubCamHdg    = CamPos_tbl.getDoubleTopic("Camera_Hdg").publish()
        pubCamPos0   = CamPos_tbl.getDoubleTopic("CamPos0").publish()
        pubCamPos1   = CamPos_tbl.getDoubleTopic("CamPos1").publish()
        pubCamPos2   = CamPos_tbl.getDoubleTopic("CamPos2").publish()

    # Subscribe to DS table so we can receive DS "commands"
    DS_tbl = ntinst.getTable("DS")
    #subDS  = DS_tbl.getStringTopic("Cmd").subscribe(' ')
    subDS  = DS_tbl.getStringTopic("View").subscribe('9')

    # start cameras
    # work around wpilibsuite/allwpilib#5055
    #CameraServer.setSize(CameraServer.kSize160x120)
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    # Allocating new images is very expensive, always try to preallocate
    for N in range (0,CamCnt):
        imgBuf[N] = np.zeros(shape=(CamMtx[N]['height'], CamMtx[N]['width'], 3), dtype=np.uint8)

    DSview = subDS.get('9')
    #while DSview == '9':
    #    print ("Waiting for Driver Station")
    #    time.sleep(1)
    #    DSview = subDS.get('9')
    #print ("Talking to Driver Station")

    if DSview in ('1','3'):
        OutputSource = RearTagCam
    else:
        OutputSource = DriverCam

    for N in range (0,CamCnt):
        input_stream[N]  = CameraServer.getVideo(CamMtx[N]['name'])
        output_stream[N] = CameraServer.putVideo(CameraLabel[N], CamMtx[N]['width'], CamMtx[N]['height'])
        #output_stream = CameraServer.putVideo(CameraLabel[OutputSource], CamMtx[OutputSource]['width'], CamMtx[OutputSource]['height'])

        options  = apriltag.DetectorOptions(
           families="tag36h11"
           ,nthreads      = 4
           ,quad_decimate = CamDetOpts[N]["quad_decimate"]
           ,quad_blur     = CamDetOpts[N]["quad_blur"]
           ,refine_edges  = CamDetOpts[N]["refine_edges"]
           ,refine_decode = CamDetOpts[N]["refine_decode"]
           ,refine_pose   = CamDetOpts[N]["refine_pose"]
           ,quad_contours = CamDetOpts[N]["quad_contours"]
           )
        detectors[N] = apriltag.Detector(options)

    Tags={"R_Reef":[6,7,8,9,10,11], "B_Reef":[17,18,19,20,21,22],
          "R_S_Processor":3, "B_S_Processor":16,
          "R_Coral_Stat":[1,2], "B_Coral_Stat":[12,13],
          "R_Barge":[5,15], "B_Barge":[4,14]}

    # loop forever
    while True:
        #K = input('Press any key:')
        #print (K)
        #print ()
        All = {}
        TagsFound = []
        DSview = subDS.get('0')
        if DSview in ('1','3'):
            OutputSource = RearTagCam
        else:
            OutputSource = DriverCam
        if DSview in ('0','1'):
            Overlay = True
        else:
            Overlay = False
        for N in (RearTagCam, DriverCam, FrontTagCam):
            frame_time, input_img[N] = input_stream[N].grabFrame(imgBuf[N])
            output_img[N]= np.copy(input_img[N])
            gray = cv2.cvtColor (input_img[N], cv2.COLOR_BGR2GRAY)
            results = detectors[N].detect(gray)
            InFrame = Find_Apriltags (GameTags, output_img[N], results, CamMtx[N]['mtx'],CamMtx[N]['dist'],CamMtx[N]['FudgeOffset'],CamMtx[N]['FudgeFactor'],N)
            All.update(InFrame)              # Merge with data 
            TagsFound = TagsFound + results  # from other cameras

        # Determine the closest tag seen
        Closest = -1
        for T in All.keys():
            if 0 <= All[T]["Rng"] < All[Closest]["Rng"]:
                Closest = T
       # if All[0][] ==All[1][]
        if T > 0:
            Publish_Camera_Location (T,All[T])

        # Don't create overlays for tags beyond Horizon
        if All[T]["Rng"] > Horizon:
            output_img[OutputSource] = cv2.flip(output_img[OutputSource],-1)
            output_stream[DriverCam].putFrame(output_img[OutputSource])
            continue

        #DScmd = subDS.get('Show')
        #if DScmd == 'Hide':
        #    continue
        DSview = subDS.get('0')
        #if DSview in ('0','1'):
        if DSview == '0':
            for T in TagsFound:
                #if T.tag_id == Closest and All[T.tag_id]["Source"] == FrontTagCam:
                if T.tag_id == Closest:
                    #print ("DSview: ",DSview)
                    #print ("OutputSource: ",OutputSource)
                    if T.tag_id in Tags["R_Reef"] or T.tag_id in Tags["B_Reef"]:
                        Reef(output_img[OutputSource],
                              All[T.tag_id]["LB_OffAxis"],
                              All[T.tag_id]["LB_Hdg"],
                              CamMtx[OutputSource]["width"],
                              CamMtx[OutputSource]["height"],
                              inset  = 0.15,
                              branch = 'L',
                              scale  = 0.3)
                        Reef(output_img[OutputSource],
                              All[T.tag_id]["RB_OffAxis"],
                              All[T.tag_id]["RB_Hdg"],
                              CamMtx[OutputSource]["width"],
                              CamMtx[OutputSource]["height"],
                              inset  = 0.15,
                              branch = 'R',
                              scale  = 0.3)
        elif DSview == '1':
            Crosshairs(output_img[OutputSource],CamMtx[DriverCam],
            CamMtx[OutputSource]["width"], CamMtx[OutputSource]["height"])
        elif DSview == '8':
            Smiley(output_img[OutputSource],CamMtx[DriverCam])
        else:
            pass

        #output_img[DriverCam]  = cv2.undistort(output_img[DriverCam], CamMtx[DriverCam]['mtx'], CamMtx[DriverCam]['dist'])
        output_img[OutputSource] = cv2.flip(output_img[OutputSource],-1)
        output_stream[DriverCam].putFrame(output_img[OutputSource])
        #output_stream.putFrame(output_img[DriverCam])

