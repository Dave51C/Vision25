# $Source: /home/scrobotics/src/2025/RCS/PiggyVision25.py,v $
# $Revision: 2.8 $
# $Date: 2025/09/26 19:33:38 $
# $Author: scrobotics $

def __version__ ():
    rev="$Revision: 2.8 $"
    return float(rev.replace("$","",-1).replace("Revision:","").replace(" ","",-1))

print ("PiggyVision25 version ",__version__())

DriverCam     = 0
RearTagCam    = 1
FrontTagCam   = 2

import numpy as np
# TAG-centric coordinates. X is right, Y is up, Z points out of screen
CORNERS   = np.array([[-3.25,  3.25, 0.0],   # Start at NW corner and proceed 
                      [ 3.25,  3.25, 0.0],   # clockwise. This is the order
                      [ 3.25, -3.25, 0.0],   # returned by Detector.
                      [-3.25, -3.25, 0.0]])  

# Sweetspot axes: Increasing X moves the sweetspot to the tag's left and
# camera's right. Increasing Y move the sweetspot up. Increasing Z moves the
# sweetspot forward from the tag along the normal to the tag face.

# A SWEETSPOT of 0,0,0 means it's at the center of the tag.
# Add to CORNERS for right Reef "trunk" offset. Subtract for left. Omit for
# all other tags
SWEETSPOT = np.array([6.47, 0.0, 0.0])

# CAMERA_OFFSET measures the distance beteen a camera and the coral mechanism.
CAMERA_OFFSET = np.array([[-1.0, 0.0, 0.0],[0.0, 0.0, 0.0],[20.75, 0.0, 0.0]])

# ROBOT_OFFSET measures the distance between the camera as (0,0) and the 
# center of the robot's 39x39 inch chassis. Uhh, make that 29.5 x 29.5.
ROBOT_OFFSET = {
# EXPRESS X AND Y IN FIELD DIRECTIONS
#  DriverCam:   {"X":-11.0,  "Y":-13.75}
# ,RearTagCam:  {"X":10.0,   "Y":0.0}      # Pure guesswork. Get measurement.
# ,FrontTagCam: {"X":-13.25, "Y":9.75}
# Thes values are for development testing. The above values are the real ones.
  DriverCam:   {"X":-15.5,  "Y":16.5}
 ,RearTagCam:  {"X":15.5,   "Y":16.5}      # Pure guesswork. Get measurement.
 ,FrontTagCam: {"X":15.5, "Y":16.5}
}

TAG_SPECS={
   1:{"X": 657.37,"Y":  25.80,"Z":58.50,"ZROT":126,"XROT":  0,"Alliance":"RED","Site":"CORAL STATION LEFT"}
 , 2:{"X": 657.37,"Y": 291.20,"Z":58.50,"ZROT":234,"XROT":  0,"Alliance":"RED","Site":"CORAL STATION RIGHT"}
 , 3:{"X": 455.15,"Y": 317.15,"Z":51.25,"ZROT":270,"XROT":  0,"Alliance":"RED","Site":"PROCESSOR"}
 , 4:{"X": 365.20,"Y": 241.64,"Z":73.54,"ZROT":  0,"XROT": 30,"Alliance":"RED","Site":"BARGE RIGHT "}
 , 5:{"X": 365.20,"Y":  75.39,"Z":73.54,"ZROT":  0,"XROT": 30,"Alliance":"RED","Site":"BARGE LEFT"}
 , 6:{"X": 530.49,"Y": 130.17,"Z":12.13,"ZROT":300,"XROT":  0,"Alliance":"RED","Site":"REEF NEAR LEFT"}
 , 7:{"X": 546.87,"Y": 158.50,"Z":12.13,"ZROT":  0,"XROT":  0,"Alliance":"RED","Site":"REEF NEAR CENTER"}
 , 8:{"X": 530.49,"Y": 186.83,"Z":12.13,"ZROT": 60,"XROT":  0,"Alliance":"RED","Site":"REEF NEAR RIGHT"}
 , 9:{"X": 497.77,"Y": 186.83,"Z":12.13,"ZROT":120,"XROT":  0,"Alliance":"RED","Site":"REEF FAR RIGHT"}
 ,10:{"X": 481.39,"Y": 158.50,"Z":12.13,"ZROT":180,"XROT":  0,"Alliance":"RED","Site":"REEF FAR CENTER"}
 ,11:{"X": 497.77,"Y": 130.17,"Z":12.13,"ZROT":240,"XROT":  0,"Alliance":"RED","Site":"REEF FAR LEFT"}
 ,12:{"X":  33.51,"Y":  25.80,"Z":58.50,"ZROT": 54,"XROT":  0,"Alliance":"BLUE","Site":"CORAL STATION RIGHT"}
 ,13:{"X":  33.51,"Y": 291.20,"Z":58.50,"ZROT":306,"XROT":  0,"Alliance":"BLUE","Site":"CORAL STATION LEFT"}
 ,14:{"X": 325.68,"Y": 241.64,"Z":73.54,"ZROT":180,"XROT": 30,"Alliance":"BLUE","Site":"BARGE LEFT"}
 ,15:{"X": 325.68,"Y":  75.39,"Z":73.54,"ZROT":180,"XROT": 30,"Alliance":"BLUE","Site":"BARGE RIGHT "}
 ,16:{"X": 235.73,"Y":  -0.15,"Z":51.25,"ZROT": 90,"XROT":  0,"Alliance":"BLUE","Site":"PROCESSOR"}
 ,17:{"X": 160.39,"Y": 130.17,"Z":12.13,"ZROT":240,"XROT":  0,"Alliance":"BLUE","Site":"REEF NEAR RIGHT"}
 ,18:{"X": 144.00,"Y": 158.50,"Z":12.13,"ZROT":180,"XROT":  0,"Alliance":"BLUE","Site":"REEF NEAR CENTER"}
 ,19:{"X": 160.39,"Y": 186.83,"Z":12.13,"ZROT":120,"XROT":  0,"Alliance":"BLUE","Site":"REEF NEAR LEFT"}
 ,20:{"X": 193.10,"Y": 186.83,"Z":12.13,"ZROT": 60,"XROT":  0,"Alliance":"BLUE","Site":"REEF FAR LEFT"}
 ,21:{"X": 209.49,"Y": 158.50,"Z":12.13,"ZROT":  0,"XROT":  0,"Alliance":"BLUE","Site":"REEF FAR CENTER"}
 ,22:{"X": 193.10,"Y": 130.17,"Z":12.13,"ZROT":300,"XROT":  0,"Alliance":"BLUE","Site":"REEF FAR RIGHT"}
}
REEF_TAGS = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]

RED_TAGS  = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
  
BLUE_TAGS = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]

# About CAGE_LOCS. The 3 Red and the 3 Blue cages have known locations on the
# field. We number them starting at 23 so we can merge them with the TAG_SPECS
# dictionary and not clobber anything.

#CAGE_LOCS = {      # NOT READY FOR PRIME-TIME
#  23:{"X" : 345.4, "Y" : 284.61, "Z": 0.0, "Alliance":"BLUE","Site":"LEFT"}
# ,24:{"X" : 345.4, "Y" : 241.61. "Z", 0.0, "Alliance":"BLUE","Site":"CENTER"}
# ,25:{"X" : 345.4, "Y" : 198.61. "Z", 0.0, "Alliance":"BLUE","Site":"RIGHT"}
# ,26:{"X" : 345.4, "Y" :  30.59, "Z": 0.0, "Alliance":"RED","Site":"LEFT"}
# ,27:{"X" : 345.4, "Y" :  73.59. "Z", 0.0, "Alliance":"RED","Site":"CENTER"}
# ,28:{"X" : 345.4, "Y" : 116.50. "Z", 0.0, "Alliance":"RED","Site":"RIGHT"}
#}

def pointer(length=300, rot=0, location=[0,0]):
    """
    Create an arrow as a 7-sided polygon. The shaft length will be "length"
    pixels long. The arrow will be rotated from north by "rot" radians and
    located at "location".
    """
    from PiggyVision25 import rotate
    import numpy as np
    
    shaft_length = length
    shaft_wid    = 50
    head_length  = 100
    head_wid     = 120

    # The reference arrow is a seven-sided polygon defined by these 8 points
    ref_arrow = np.array([[0,0]
                 ,[head_wid/2,   head_length]
                 ,[shaft_wid/2,  head_length -10]
                 ,[shaft_wid/2,  shaft_length + head_length]
                 ,[-shaft_wid/2, shaft_length + head_length]
                 ,[-shaft_wid/2, head_length -10]
                 ,[-head_wid/2,  head_length]
                 ,[0,0]],dtype=np.int32)
    # The reference arrow will be rotated abour the centroid of the shaft
    Cx        = 0
    Cy        = shaft_length/2 + head_length

    arrow     = np.copy(ref_arrow)
    for i in range(8):
        arrow[i,0],arrow[i,1] = rotate(ref_arrow[i,0],ref_arrow[i,1],Cx,Cy,rot)
    location[1] = location[1] - head_length - int(shaft_length/2)
    return [arrow+location]

def rotor(radius=200, rot=0, location=[0,0], scale=1.0):
    """
    Create a graphic that indicates the robot's rotation.
    """
    from PiggyVision25 import rotate
    import numpy as np
    from math import sin, cos, pi
    
    notch_length = 60
    # The reference notch is a six-sided polygon defines by these 7 points
    ref_notch = np.array([[0,0]
                ,[notch_length*sin(pi/8), notch_length*(1-cos(pi/8))]
                ,[notch_length*sin(pi/4), notch_length*(1-cos(pi/4))]
                ,[0,notch_length]
                ,[-notch_length*sin(pi/4), notch_length*(1-cos(pi/4))]
                ,[-notch_length*sin(pi/8), notch_length*(1-cos(pi/8))]
                ,[0,0]],dtype=np.int32)

    notch     = np.copy(ref_notch*scale).astype(np.int32)
    #notch[:1]+=radius
    for i in range(7):
        notch[i,0],notch[i,1] \
             = rotate(notch[i,0],notch[i,1],0,radius*scale,rot,True)
             #= rotate(notch[i,0],notch[i,1],225,125,rot)
    notch[:,1]-=radius
    return [notch+location]

def rotate(px, py, ox, oy, angle, Integer=False):
    """
    Rotate a point at (px, py) about an origin at (ox, oy) by the given angle.
    The angle is in radians.
    If Integer is True the new values are rounded to the nearest integer
    Return the point's new (X,Y).
    """
    from math import sin, cos
    newx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    newy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    if Integer:
        newx = round(newx)
        newy = round(newy)
    return newx, newy

def register(X, Y, Hdg, pathname, delimiter=' '):
    """
    This writes the arguments plus a timestamp to a file (pathname). Its
    intended to log`the robot's position (X,Y) and orientation (Hdg). By
    default, the fields are separated by spaces. 
    """
    import time
    try:
        poslog=open(pathname,'a') 
    except:
        print ("Can't open output. Is file system read-only?")
        return (False)

    poslog.write('{1:5.3f}{0:s}{2:5.3f}{0:s}{3:5.3f}{0:s}{4:5.3f}\n'
       .format(delimiter,float(X),float(Y),float(Hdg),time.time()))
    poslog.close
    return (True)

def PosRngHdg (rvecs, tvecs):
    """
    This magic uses the rotation and translation vectors from solvePNP.
    It returns
     - the Camera X & Y in tag-centric coords. We drop the vertical component
       because we don't use it.
     - the Camera's distance to the tag (range)
     - the tag's angular offset (radians) from "dead ahead"
    """
    from math  import pi,atan2,sqrt 
    from cv2   import Rodrigues
    from numpy import matrix
    ZYX,jac  = Rodrigues(rvecs)

    # Omit vertical component tvecs[1]
    HdgRad   = atan2(tvecs[0],tvecs[2])

    CamPos   = -matrix(ZYX).T * matrix(tvecs)

    # Omit vertical component CamPos[1]
    CamRange = (sqrt(CamPos[0]**2+CamPos[2]**2)) 

    return CamPos[0], CamPos[2], CamRange, HdgRad

def Crosshairs (output_img,CamMtxi,width,height):
    import cv2
    thickness = 3
    cv2.line(output_img,(int(width/2),0),(int(width/2),height),(0,255,0),thickness)
    cv2.line(output_img,(int(width/2),0),(int(width/2),0),(0,255,0),thickness)
    P1 = (int(width*0.9/2),int(height*0.1))
    P2 = (int(width*1.1/2),int(height*0.1))
    cv2.line(output_img,P1,P2,(0,255,0),thickness)

def Reef(output_img,OffAxis,Hdg,width,height,scale=1.0,inset=0.15,branch='L'):
    def Circle_move(output_img,OffAxis,Hdg,width,height,inset,branch,Offset=0,scale=1.0):
        if branch == 'L':
            cx = int(inset*width + (Hdg-OffAxis)*150)    # The 150 is just to exagerate the difference
        else:
            cx = int(width - inset*width + (Hdg-OffAxis)*150)
        cy = int(height / 2)
        radius = width *inset * scale * 2
    
        cv2.circle(output_img, (cx, cy), int(radius), (0, 255, 0), thickness)
        Bar = np.array([[0,radius],[0,-radius]])
        newx,newy = rotate(Bar[0][0], Bar[0][1], 0, 0, OffAxis, True)
        Bar[0][0] = newx + cx
        Bar[0][1] = newy + cy
        newx,newy = rotate(Bar[1][0], Bar[1][1], 0, 0, OffAxis, True)
        Bar[1][0] = newx + cx
        Bar[1][1] = newy + cy
        cv2.line(output_img,tuple(Bar[0].astype(int)),tuple(Bar[1].astype(int)),(0,255,0),thickness)

    import cv2
    thickness = 3
    cv2.line(output_img,(int(width * inset),0),
                        (int(width * inset),height),(0,255,0),thickness)
    cv2.line(output_img,(width-int(width * inset),0),
                        (width-int(width * inset),height),(0,255,0),thickness)
    if Hdg >0:
        Circle_move(output_img,OffAxis, Hdg,width,height,inset,branch, -13,scale,)
    elif Hdg <0:
        Circle_move(output_img,OffAxis, Hdg,width,height,inset,branch,  13,scale)

