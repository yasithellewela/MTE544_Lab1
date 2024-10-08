# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import math
import numpy  as np

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.legend()
    plt.xlabel('Time (ns)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('LIDAR Output: Spiral')
    plt.grid()
    plt.show()


def plot_laser(filename):
    #Function to plot laser data
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)


    increm = values[0][-2]  
    
    ranges = np.array(values[0][:-2])
    ang = 0
    x = []
    y = []

    for val in ranges:
        x.append(val*math.cos(ang)) 
        y.append(val*math.sin(ang)) 
        ang += increm


    plt.scatter(x, y)

    plt.xlabel('X-distance (m)')
    plt.ylabel('Y-distance (m)')
    plt.title('LIDAR Output: Spiral')

    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        # plot_errors(filename)
        plot_laser(filename)
