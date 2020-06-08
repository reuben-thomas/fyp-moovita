from __future__ import print_function
import numpy as np

def main():
	try:
		r = float(input("Radius in metres (103.67): "))
		degrees = float(input("Angle in degrees (1): "))
		
	except:
		print("Invalid input.")
		main()
		
	print("\nRadius: ", r, "\nAngle: ", degrees)

	angle = np.radians(degrees)
	theta = 0
	runs = int(2*np.pi/angle)

	for n in np.arange(0, 2*np.pi, angle):
		x = r * np.cos(theta)
		y = r * np.sin(theta)
		theta = theta + angle
		print ('<point>', x, ' ', y, ' ', 0, '</point>', sep = '')

	print ('<point>', r, ' ', 0, ' ', 0, '</point>', sep = '')
	print("\n This program has looped ", runs, " times.")

if __name__ == "__main__":
	main()
