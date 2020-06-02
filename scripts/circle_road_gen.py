from __future__ import print_function
import numpy as np

r = input("Radius in metres (101.6): ")
angle = input("Angle in radians (0.0174533): ")

print("\nRadius: ", r, "\nAngle: ", angle)

theta = 0
runs = int(2*np.pi/angle)

for n in np.arange(0, 2*np.pi, angle):
	x = r * np.cos(theta)
	y = r * np.sin(theta)
	theta = theta + angle
	print ('<point>', x, ' ', y, ' ', 0, '</point>', sep = '')

print ('<point>', r, ' ', 0, ' ', 0, '</point>', sep = '')
print("\n This program has looped ", runs, " times.")