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
 
D_MAX: int = 6000
I_MIN: int = 0
I_MAX: int = 50
scans = []
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
    motorAngle += motorRotation
	 
if __name__ == '__main__':
	 
    if not verify_device():
        print(f'No device found: {DEVICE_PATH}')
        exit(1)

    lidar = RPLidar(port=DEVICE_PATH, baudrate=BAUD_RATE, timeout=TIMEOUT)
    lidar.clean_input()

	 
    try:

        lidar.clean_input()
        iterator = lidar.iter_measures(max_buf_meas=10000)
        
        while True:

            lidar.clean_input()
            scans = []

            for i in range(500):
                scan = next(iterator)
                scans.append(scan)
           
            #moveMotor()

            #rechnet in 3D punkte um [quality,x,y,z]
            
            for scan in scans:
                t = []
                t.append(scan[1])                                       #saves scan quality
                r = math.sin(scan[2] *  360 / math.pi) * scan[3]        #radius without y offset
                t.append(math.sin(motorAngle * 360 / math.pi) * r)      #t[1] == x in 3D
                t.append(math.cos(scan[2] *  360 / math.pi) * scan[3])  #t[2] == y von p
                t.append(math.cos(motorAngle * 360 / math.pi) * r)      #t[3] == z

                scan = t                                               #überschreibt scan mit t
            
            scansInArray = np.array(scans)
            points = laspy.ScaleAwarePointRecord.zeros(point_count=500, header=hdr)
            for scan in scans:

                points.x = scansInArray[:,1]
                points.y = scansInArray[:,2]
                points.z = scansInArray[:,3]
                #points.intensity = scansInArray[:0]
            with laspy.open("points.las", mode="w", header=hdr) as writer:
                writer.write_points(points)
            print("W")
            sleep(1)
            

    except KeyboardInterrupt:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()