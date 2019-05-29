import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import string
from scipy.interpolate import spline
import math
import glob
import os



x = []
y = []

def plot():
    global x,y
    with open('sluttest13.txt', 'r') as f:
        f = f.readlines()
        del f[-1]
        for line in f:
            lat = float(line.split(",")[0])
            lon = float(line.split(",")[1])
            x.append(lat)
            y.append(lon)
            #lat = "%.10f" % lat
            #lon = "%.10f" % lon
            #print(lat + "," + lon)
                
    x_o = x[0]
    y_o = y[0]
    x = np.array(x)
    y = np.array(y)
    x = x - x_o
    y = y - y_o
    
    plt.plot(x,y)
    plt.axis('equal')
    plt.show()
    
if __name__ == '__main__': 
	plot()
