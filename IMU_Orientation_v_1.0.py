import vpython as vp
import time
import math
import serial
import numpy as np

def Rx(vec, rotationAngle):
    new_x = vec.x
    new_y = vec.y * math.cos(rotationAngle) - vec.z * math.sin(rotationAngle)
    new_z = vec.y * math.sin(rotationAngle) + vec.z * math.cos(rotationAngle)
    return vp.vector(new_x, new_y, new_z)

def Ry(vec, rotationAngle):
    new_x = vec.x *  math.cos(rotationAngle)  + vec.z *  math.sin(rotationAngle)
    new_y = vec.y
    new_z = vec.x * -math.sin(rotationAngle) + vec.z * math.cos(rotationAngle)
    return vp.vector(new_x, new_y, new_z)

def Rz(vec, rotationAngle):
    new_x = vec.x * math.cos(rotationAngle) + vec.y * -math.sin(rotationAngle)
    new_y = vec.x * math.sin(rotationAngle) + vec.y * math.cos(rotationAngle)
    new_z = vec.z
    return vp.vector(new_x, new_y, new_z)

def Move(vec, newPos):
    new_x = vec.x + newPos.x
    new_y = vec.y + newPos.y
    new_z = vec.z + newPos.z
    return vp.vector(new_x, new_y, new_z)

# intializing the varibles
newPhi = newTheta = newPsi = oldPhi = oldTheta = oldPsi = 0    
new_x_Trans = new_y_Trans = new_z_Trans = old_x_Trans = old_y_Trans = old_z_Trans = 0
gameSpeed = 10
rot_vel = gameSpeed * math.pi/16 
trans_vel = gameSpeed * 5

# initializing IMU Board
IMUBoardLength = 21.2
IMUBoardWidth = 16.4
IMUBoardHeight = 3.3
IMUBoard = vp.box(pos= vp.vector(0, 0, 0), 
                  size= vp.vector(IMUBoardLength, IMUBoardWidth, IMUBoardHeight), 
                  color = vp.color.blue)

# initializing Global Coordinates Axis
globalArrowLength = 25
xGlobalArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(1, 0, 0), 
                        length= globalArrowLength, color= vp.vector(1, 0, 0))
yGlobalArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 1 , 0), 
                        length= globalArrowLength, color= vp.vector(0, 1, 0))
zGlobalArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0 , 1), 
                        length= globalArrowLength, color= vp.vector(0, 0, 1))

# initializing Local Coordinates Axis
IMUArrowLength = 20
xIMUArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(1, 0, 0), 
                    length= IMUArrowLength, color= vp.vector(0.8, 0.5, 0.5))
yIMUArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 1 , 0), 
                    length= IMUArrowLength, color= vp.vector(0.5, 0.8, 0.5))
zIMUArrow = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0 , 1), 
                    length= IMUArrowLength, color= vp.vector(0.5, 0.5 , 0.8))



# initializing serial commnication
while True:
    try:
        arduinoData = serial.Serial('COM8', 19200)
        break
    except Exception as error:
        print("An error occurred:", error)
        print("Trying again after 1 sec ...")
        time.sleep(1)

lastTime = time.time()  # starting timer for delta time

oldPhi = 0
oldTheta = 0
oldPsi = 0

while True:
    # reseting variables
    deltaTime = 0
    # starting delta timer
    lastTime = time.time()
    while deltaTime < 1/60 : # limiting animation speed by waiting
        deltaTime = (time.time() - lastTime)
    
        while True:
            try:
                arduinoString = arduinoData.readline()
                arduinoString = str( arduinoString, encoding="utf-8")
                dataList = []
                dataList = arduinoString.split('/')
                
                dataList = list(np.array(dataList).flat) # ???

                newPhi = float(dataList[0])
                newTheta = float(dataList[1])
                newPsi = float(dataList[2])
                break
            except:
                pass    
        ## -- Rotation Update --
    # rotate about x
    IMUBoard.rotate(angle= newPhi-oldPhi, axis=xGlobalArrow.axis)
    oldPhi  = newPhi

    # rotate about y
    IMUBoard.rotate(angle= newTheta-oldTheta, axis=yGlobalArrow.axis)
    oldTheta = newTheta
    
    # rotate about z
    IMUBoard.rotate(angle= newPsi-oldPsi, axis=zGlobalArrow.axis)
    oldPsi = newPsi
    
    # update IMU local coordinates
    xIMUArrow.axis = IMUBoard.axis * IMUArrowLength / IMUBoard.axis.mag
    yIMUArrow.axis = IMUBoard.up * IMUArrowLength
    x_cross_y = xIMUArrow.axis.cross(yIMUArrow.axis)
    x_cross_y = x_cross_y / x_cross_y.mag   # normalizing
    zIMUArrow.axis = x_cross_y * IMUArrowLength
       
    ## -- Translation Update --
    # relative translation of IMUBoard and IMU local coordinate
    IMUBoard.pos = Move(IMUBoard.pos, vp.vector(new_x_Trans - old_x_Trans, 
                                                new_y_Trans - old_y_Trans, 
                                                new_z_Trans - old_z_Trans))
    old_x_Trans = new_x_Trans
    old_y_Trans = new_y_Trans
    old_z_Trans = new_z_Trans
    xIMUArrow.pos = yIMUArrow.pos = zIMUArrow.pos = IMUBoard.pos
    
