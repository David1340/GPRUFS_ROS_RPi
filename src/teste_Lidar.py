from LidarX2 import LidarX2
import time
import matplotlib.pyplot as plt
import numpy as np

def convert_to_scalar(measures):
    angles = []
    distances = []
    for measure in measures:
        angle,distance = measure.get_pair()
        angles.append(angle*(np.pi/180))
        distances.append(distance)
    return angles,distances

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, polar=True)

lidar = LidarX2("/dev/ttyUSB0")  # Name of the serial port, can be /dev/tty*, COM*, etc.

if not lidar.open():
    print("Cannot open lidar")
    exit(1)

t = time.time()
while time.time() - t < 5*60:  # Run for 20 seconds
    measures = lidar.getMeasures()  # Get latest lidar measures
    
    if(len(measures) > 0):
        print(len(measures))
        #print(measures)   
        angles,distances = convert_to_scalar(measures)
        ax.clear()
        ax.scatter(angles, distances,s = 0.5,alpha = 1)
        ax.set_ylim(0, 6500)  # Define o limite superior do eixo radial como 6500
        plt.draw
        plt.pause(0.01)
        #time.sleep(0.1)
    
lidar.close()