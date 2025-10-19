# $Source: /home/scrobotics/src/2025/RCS/Find_Apriltags_List.py,v $
# $Revision: 2.11 $
# $Date: 2025/05/04 12:56:16 $
# $Author: scrobotics $

def __version__ ():
    rev="$Revision: 2.11 $"
    return float(rev.replace("$","",-1).replace("Revision:","").replace(" ","",-1))

print ("Find_Apriltags_List version ",__version__())

import json
import apriltag
import numpy as np
#from ntcore import NetworkTableInstance
#IP='localhost'

#ntinst       = NetworkTableInstance.getDefault()
#ntinst.setServer(IP)
#CamPos_tbl   = ntinst.getTable("CamPos")  
#pubCamWorldX = CamPos_tbl.getDoubleTopic("X").publish()
#pubCamWorldY = CamPos_tbl.getDoubleTopic("Y").publish()
#pubCamWorldR = CamPos_tbl.getDoubleTopic("Rot").publish()

BEARINGS = {-1:{"Status":-1, "Hdg":999.0, "Rng":999.0}}

def BuildWorld (WX, WY, HdgRad, RefTag):
    # New method of knowing position with respect to all tags by only reading one
    # Calculate intermediate angles relative to X-axis. Adjust for heading later.
    # A2R: angle between RefTag and X-axis at camera
    # A2K: angle between Tag K and X-axis at camera

    deltaX = TAG_SPECS[RefTag]["X"] - WX
    deltaY = TAG_SPECS[RefTag]["Y"] - WY
    A2R = atan2(deltaY,deltaX)

    for K in range(1,23):
        deltaX = HotSpots[K][0] - WX
        deltaY = HotSpots[K][1] - WY
        A2K = atan2(deltaY,deltaX)
        H2K = -(A2K - A2R - HdgRad)
        if H2K > pi:
           H2K = H2K - 2*pi 
        BEARINGS[K]["Hdg"] = np.degrees(H2K)
        BEARINGS[K]["Rng"] = sqrt(deltaX**2 + deltaY**2)


def get_camera_parameters ():
    """
    Create a dictionary of all the camer parameters for the cameras in 
    /boot/frc.json 
    """
    import json
    import numpy as np
    USB_Cams = open('/boot/frc.json','r')
    j        = json.load(USB_Cams)
    USB_Cams.close()
    
    Success = True
    CamMtx  = {}
    CamNbr  = 0
    CamDetOpts = {}
    for C in j['cameras']:
        try:
            CamParams = open(C['name']+'.json','r')
            j     = json.load(CamParams)
            Cam   = j['name']
            CamMtx[CamNbr] = {'name':j['name'], 'width':j['width'],
               'height':j['height'],
               'mtx':np.array(j['mtx']), 'dist':np.array(j['dist']),
               'FudgeOffset':j['FudgeOffset'],
               'FudgeFactor':j['FudgeFactor']}
            CamDetOpts[CamNbr] = j["DetectorOptions"]
            CamParams.close()
            CamNbr += 1
        except:
            Success = False
            print(C['name'],' processing failed.')
    #print(json.dumps(CamMtx,indent=4))
    return Success, len(CamMtx), CamMtx, CamDetOpts

"""
Find_Apriltags returns dictionary of dictionaries.
"""
def Find_Apriltags (TagIDs,img, TagsSeen, USBcam_mtx, USBcam_dist, FudgeOffset, FudgeFactor, Source):
    """
    THESE COMMENTS ARE HOPELESSLY OUTDATED
    Find_Apriltag takes 4 arguments: a tag number, an image that may, or may
    not, show one or more tags, the camera's intrinsic matrix and distortion
    coefficients. It returns:
        # Depracating Status and returning only tags foubd
        Status: -1 No Tags found
                 0 Tag found in image (Hdg & Rng computed)
                 1 Tag not found in image (Hdg & Rng inferred) # NOT IMPLEMENTED
        Hdg: Heading (in radians) of the Tag relative to camera's line if sight.
             = 0 means Tag is dead-ahead
             < 0 means Tag is to the right
             > 0 means Tag is to the left
        Rng: Distance (in inches) to Tag
        CamPos:
        OffAxis: Angle between Tag's "normal" vector and the camera's line of
            sight. It has no vertical component, just horizontal plane.
        rvecs, tvecs: magical incantations
    """
    from math import pi,atan2,asin,sqrt,tan
    from PiggyVision25 import rotate, CORNERS, TAG_SPECS, REEF_TAGS, SWEETSPOT, CAMERA_OFFSET, ROBOT_OFFSET
    import time
    import cv2
    import numpy as np

    CamPos   = np.array([0.0, 0.0, 0.0])
    CamPos   = CamPos.reshape(3,1)
    CamRange = 0.0
    fudgedCorners = (CORNERS - FudgeOffset) / FudgeFactor

    Tag_Set={-1:{"Status":-1,"Hdg":999.0,"Rng":999.0,"OffAxis":999.0}}
    for r in TagsSeen:
        if (r.tag_id in TagIDs):
            ret, rvecs, tvecs = cv2.solvePnP(fudgedCorners,r.corners,
                         USBcam_mtx,USBcam_dist,
                         cv2.SOLVEPNP_IPPE_SQUARE)
            ZYX,jac  = cv2.Rodrigues(rvecs)
            R        = cv2.Rodrigues(rvecs)[0]
            # We flip Hdg's sign to make it camera-centric?
            Hdg      = -atan2(tvecs[0],tvecs[2])
            OffAxis  = asin(R[2][0])

            CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)

            # Range calculation ingores vertival component
            CamRange = sqrt(CamPos[0]**2+CamPos[2]**2)
            Status   = 0
            Tag_Set[r.tag_id]={"Status":0, "Hdg": Hdg, "Rng": CamRange,
                      "CamPos" : CamPos, "OffAxis" : OffAxis,"rvecs" : rvecs,
                      "tvecs" : tvecs, "RB_OffAxis":None, "LB_OffAxis" : None,
                      "RB_Error" : 0, "LB_Error" : 0, "Source" : Source}

            if r.tag_id in REEF_TAGS:
                #ret, rvecs, tvecs = cv2.solvePnP(CORNERS-SWEETSPOT-CAMERA_OFFSET[Source],r.corners,
                ret, rvecs, tvecs = cv2.solvePnP(fudgedCorners+SWEETSPOT-CAMERA_OFFSET[Source],r.corners,
                         USBcam_mtx,USBcam_dist,
                         cv2.SOLVEPNP_IPPE_SQUARE)
                R = cv2.Rodrigues(rvecs)[0]
                RB_Hdg     = -atan2(tvecs[0],tvecs[2])
                RB_OffAxis = asin(R[2][0])
                RB_Error   = RB_Hdg - RB_OffAxis
                #ret, rvecs, tvecs = cv2.solvePnP(CORNERS+SWEETSPOT-CAMERA_OFFSET[Source],r.corners,
                ret, rvecs, tvecs = cv2.solvePnP(fudgedCorners-SWEETSPOT-CAMERA_OFFSET[Source],r.corners,
                         USBcam_mtx,USBcam_dist,
                         cv2.SOLVEPNP_IPPE_SQUARE)
                R = cv2.Rodrigues(rvecs)[0]
                LB_Hdg     = -atan2(tvecs[0],tvecs[2])
                LB_OffAxis = asin(R[2][0])
                LB_Error   = LB_Hdg - LB_OffAxis
                Tag_Set[r.tag_id]["RB_OffAxis"] = RB_OffAxis
                Tag_Set[r.tag_id]["LB_OffAxis"] = LB_OffAxis
                Tag_Set[r.tag_id]["RB_Hdg"]     = RB_Hdg
                Tag_Set[r.tag_id]["LB_Hdg"]     = LB_Hdg
                Tag_Set[r.tag_id]["RB_Error"]   = RB_Error
                Tag_Set[r.tag_id]["LB_Error"]   = LB_Error
                Tag_Set[r.tag_id]["Source"]     = Source

    return Tag_Set

"""
OmniTags returns a list of dictionaries. Includs non-Reef too.
"""
def OmniTags (TagIDs,img, TagsSeen, USBcam_mtx, USBcam_dist, FudgeOffset, FudgeFactor, Source):
    from math import pi,atan2,asin,sqrt,tan
    from PiggyVision25 import rotate, CORNERS, TAG_SPECS, REEF_TAGS, SWEETSPOT, CAMERA_OFFSET, ROBOT_OFFSET
    import time
    import cv2
    import numpy as np

    CamPos   = np.array([0.0, 0.0, 0.0])
    CamPos   = CamPos.reshape(3,1)
    fudgedCorners = (CORNERS - FudgeOffset) / FudgeFactor

    Tag_Set = []
    for r in TagsSeen:
        #print ("Source:", Source,"  Saw #",r.tag_id)
        if (r.tag_id in TagIDs):
            ret, rvecs, tvecs = cv2.solvePnP(fudgedCorners,r.corners,
                         USBcam_mtx,USBcam_dist, cv2.SOLVEPNP_IPPE_SQUARE)
            ZYX,jac  = cv2.Rodrigues(rvecs)
            CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)
            Wrk_Row  = [{"Tag_Id":r.tag_id, "Source":Source, "CamPos":CamPos, 
                      "rvecs" : rvecs, "tvecs" : tvecs}]
            Tag_Set += Wrk_Row
    #for i in range (0, len(Tag_Set)):
    #    print (Tag_Set[i])

    return Tag_Set

if __name__ == "__main__":
    import argparse, time
    import cv2, numpy as np
    options  = apriltag.DetectorOptions(
           families="tag36h11"
           ,nthreads      = 4
           ,quad_decimate = 2
           ,quad_blur     = 0.0
           ,refine_edges  = True
           ,refine_decode = True
           ,refine_pose   = False
           ,quad_contours = True
           )
    parser = argparse.ArgumentParser()
    parser.add_argument("--tags", nargs='+', type=int, help="List of Tag IDs you're looking for")
    args = parser.parse_args()

    cap    = cv2.VideoCapture(2)         # Initialize webcam
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

    detector = apriltag.Detector(options)

    img    = np.zeros(shape=(height, width, 3), dtype=np.uint8)
    _, frame = cap.read()                    # Capture frame
    gray    = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    cv2.imshow('Webcam', frame)              # Display frame
    cap.release()                # Release the webcam
    cv2.destroyAllWindows()      # Close the window

    THEN=time.time()
    All = Find_Apriltags (args.tags, frame, results)
    print(1/(time.time()-THEN))

    Closest_Tag = -1
    for T in args.tags:
        if 0 <= All[T]["Rng"] < All[Closest_Tag]["Rng"]:
            Closest_Tag = T
