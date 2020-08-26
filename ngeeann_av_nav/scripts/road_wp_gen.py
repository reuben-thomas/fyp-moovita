from __future__ import print_function
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
from utils.cubic_spline_planner import *

class Simulate:

    def __init__(self):
        
        # Track resolution
        self.ds = 0.5

        self.ax = [0,  0,   0,   0,   0,   0,   0, -25, -50, -100,  -150, -125, -150, -175, -200, -100, -75, -125, -150, -90,   0,   0,  0, 0]
        self.ay = [0, 50, 100, 150, 190, 198, 200, 225, 200,  125,   125,  200,  250,  165,  100,   70,  25,    0,  -65, -50, -15, -10, -5, 0]
        print('Initialized Targets:')
        print(self.ax)
        print(self.ay)

        self.cx, self.cy, _, _, _ = calc_spline_course(self.ax, self.ay, self.ds)


    # forms the coordinates for the road to be imported in the world file
    def form_road(self):
        for n in np.arange(0, len(self.cx)):
            print ('<point>', self.cx[n], ' ', self.cy[n], ' ', 0, '</point>', sep = '')

    # forms target waypoint csv for the vehicle to follow
    def form_targets(self):
        X, Y = [], []

        for n in np.arange(0, len(self.cx)):
            if n % 50 == 0:
                X.append(self.cx[n])
                Y.append(self.cy[n])

        print(X)
        print(Y)

        axis = {'X-axis': X, 'Y-axis': Y}
        df = pd.DataFrame(axis, columns= ['X-axis', 'Y-axis'])
        df.to_csv("waypoints.csv", index = False)

    def plot_waypoints(self):
        figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.plot(self.cx, self.cy)
        plt.plot(self.ax, self.ay)
        plt.show()
  

def main():
    world = Simulate()
    world.plot_waypoints()
    world.form_targets()

if __name__ == "__main__":
    main()
