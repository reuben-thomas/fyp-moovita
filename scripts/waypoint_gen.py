from __future__ import print_function
import numpy as np
import pandas as pd

r = input("Radius in metres: ")
angle = input("Angle in radians: ")

print("\nRadius: ", r, "\nAngle: ", angle)

theta = 0

X = []
Y = []
runs = int(2*np.pi/angle)

for n in np.arange(0, 2*np.pi, angle):
	x = r * np.cos(theta)
	y = r * np.sin(theta)
	theta = theta + angle
	X.append(x)
	Y.append(y)

X.append(r)
Y.append(0)
print(X)
print(Y)
print("\n This program has looped ", runs, " times.")

axis = {'X-axis': X, 'Y-axis': Y}
df = pd.DataFrame(axis, columns= ['X-axis', 'Y-axis'])
df.to_csv("waypoints.csv", index = False, header = False)