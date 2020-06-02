'''
This is a waypoint generator for circular roads.
'''

from __future__ import print_function
import numpy as np
import pandas as pd

def main():
	mode = raw_input("Point or Angle mode (p/a): ")

	# Point mode
	if mode == "p":
		point_mode()
	elif mode == "a":
		angle_mode()
	else:
		print("Wrong command")
		main()

def point_mode():
	try:
		r = input("Radius in metres (101.6): ")
		points = input("Number of points: ")
	
	except:
		print("Wrong command.")
		point_mode()

	print("\nRadius: ", r, "\nPoints: ", points)

	theta = 0

	X = []
	Y = []

	angle = 2*np.pi/points
	runs = 0

	for n in np.arange(0, 2*np.pi, angle):
		x = r * np.cos(theta)
		y = r * np.sin(theta)
		theta = theta + angle
		X.append(x)
		Y.append(y)
		runs = runs + 1

	X.append(r)
	Y.append(0)
	print(X)
	print(Y)
	print("\n This program has looped ", runs, " times.")

	axis = {'X-axis': X, 'Y-axis': Y}
	df = pd.DataFrame(axis, columns= ['X-axis', 'Y-axis'])
	df.to_csv("waypoints.csv", index = False)

def angle_mode():
	try:
		r = input("Radius in metres (101.6): ")
		angle = input("Angle in radians: ")

	except:
		print("Wrong command.")
		angle_mode()	

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
	df.to_csv("waypoints.csv", index = False)

if __name__ == "__main__":
	main()