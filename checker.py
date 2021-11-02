import numpy as np
import cv2 as cv
pts = [(0, 0), (25, 30), (51, 60), (81, 90), (82, 120), (97, 150), (127, 180), (157, 210), (187, 240), (199, 270), (199, 300), (201, 330)]
angles = [5.389, 4.236, -0.0, 41.055, 19.231, -0.0, -0.0, -0.0, 22.521, 45.0, 41.055, 45.0]
hops = 30

dimX = 400
dimY = 400

def connect_point():
	error_theta = 0
	img = np.zeros((dimX, dimY,3), np.uint8)
	img = cv.bitwise_not(img)
	thetas = []
	for i in range(len(pts)-1):
		x0, y0 = pts[i]
		x1, y1 = pts[i+1]
		thetas.append(round(np.arctan((y1 - y0)/ (x1 - x0+ 1e-9))*180/np.pi - 45, 3))
		error_theta += abs(thetas[i] - angles[i]) 
	print(thetas)
	print(error_theta)


	"""

	for pt, angle in  zip(pts, angles):
		x0, y0 = pt
		theta = 45 + angle
		m = np.tan(theta*180/np.pi)
		x1 = x0
		y1 = m*(x1-x0) + y0
		x1, y1 = int(x1), int(y1)1
		pt2 = (x1, y1)
		cv.line(img, pt, pt2,(255, 0, 0), 2)
	"""
	for i in range(len(pts)-1):
		cv.line(img, pts[i], pts[i+1],(255, 0, 0), 2)
	cv.line(img, pts[len(pts)-1], (200, 368),(255, 0, 0), 2)

	cv.imshow('Vectorised input', img)
	cv.waitKey(0)

m = 15

def devectorise():
	pass


