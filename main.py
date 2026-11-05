from os import path
from sys import exit
import math	 
from rplidar import RPLidar
from time import sleep
import laspy
import numpy as np
 
BAUD_RATE: int = 115200
TIMEOUT: int = 1

DEVICE_PATH: str = "/dev/ttyUSB0"
 
global motorAngle
global motorRotation

motorAngle = 0
motorRotation = 1.8
hdr = laspy.LasHeader(version="1.2", point_format=0)
 

def verify_device() -> bool:
    if path.exists(DEVICE_PATH):
        return True
    else:
        return False

def moveMotor():
    #gpio will be done after wiring
    global motorAngle
    motorAngle += motorRotation
	 
def saveStuff(dataList):
    scansInArray = np.array(dataList)
    points = laspy.ScaleAwarePointRecord.zeros(point_count=len(scansInArray), header=hdr)
    points.x = scansInArray[:,1]
    points.y = scansInArray[:,2]
    points.z = scansInArray[:,3]
    #points.intensity = scansInArray[:0]

    with laspy.open("points.las", mode="w", header=hdr) as writer:
        writer.write_points(points)


if __name__ == '__main__':
    
    if not verify_device():
        print(f'No device found: {DEVICE_PATH}')
        exit(1)

    lidar = RPLidar(port=DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)
    lidar.clean_input()

    try:
        validPoints = []
        numOfScans = int(360 / motorRotation)
        for i in range(numOfScans):

            iterator = lidar.iter_measures(max_buf_meas=5000)
            scans = []

            for j in range(500):
                scan = next(iterator)
                scans.append(scan)

            lidar.stop()
            lidar.clean_input()
            moveMotor()
            
            #rechnet in 3D punkte um [quality,x,y,z]
            for scan in scans:
                t = []
                if scan[3]!= 0:
                    t.append(scan[1])                                       #saves scan quality
                    t.append(math.sin(scan[2] /  180 * math.pi) * math.cos(motorAngle /  180 * math.pi) * scan[3])     #t[1] == x in 3D
                    t.append(math.sin(scan[2] /  180 * math.pi) * math.sin(motorAngle /  180 * math.pi) * scan[3])  #t[2] == y von p
                    t.append(math.cos(scan[2] /  180 * math.pi) * scan[3])     #t[3] == z

                    validPoints.append(t)  
    
            print(f"scan {i + 1} of {numOfScans}")
    except KeyboardInterrupt:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

    finally:    
        saveStuff(validPoints)
        print("done")