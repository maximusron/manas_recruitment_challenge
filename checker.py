import numpy as np
import cv2 as cv
pts = [(0, 0), (25, 30), (51, 60), (81, 90), (82, 120), (97, 150), (127, 180), (157, 210), (187, 240), (199, 270), (199, 300), (201, 330)]
angles = [5.389, 4.236, -0.0, 41.055, 19.231, -0.0, -0.0, -0.0, 22.521, 45.0, 41.055, 45.0]
hops = 30

dimX = 400
dimY = 400

num_steps = 0

end_point = (200, 368)
angle_wrt = 45

car_width = 7
car_height = 12
car_diagonal = (car_width**2 + car_height**2)**0.5

ld = 0

grid = np.zeros((dimX, dimY), np.uint8)
grid = cv.bitwise_not(grid)

def get_test_data():
	obstacles1 = [(120, 240), (163, 309), (250, 300), (338, 99), (120, 120), (201, 182)]#, (), (), (), ()]
	obstacles2 = [(120, 240), (163, 309), (250, 300), (338, 99), (120, 120), (201, 182), (59, 15), (40, 155)]#, (), ()]
	obstacles3 = [(304, 285), (20, 372), (340, 217), (203, 73), (81, 81), (77, 160), (344, 88), (311, 19), (9, 56), (381, 365), (213, 52), (77, 301)]
	obstacles4 = [(196, 83), (266, 233), (118, 361), (58, 28), (288, 136), (388, 276), (25, 198), (57, 373), (55, 173), (247, 277), (343, 244), (23, 333), (350, 171), (87, 263), (175, 117), (154, 123), (194, 228), (358, 362), (79, 81), (106, 21)]
	obstacles5 = [(35, 312), (356, 41), (50, 46), (209, 316), (225, 272), (354, 283)]

	return obstacles2

def generate_map(obstacles):


	for obst in obstacles:
		cv.circle(grid, obst, 25 + int(car_diagonal/2), (0, 0, 0), -1)

	return grid



def connect_point():

	obsts = get_test_data()
	grid = generate_map(obsts)

	error_theta = 0
	img= np.zeros((dimX, dimY,3), np.uint8)
	img = cv.bitwise_not(img)
	thetas = []
	for i in range(len(pts)-1):
		x0, y0 = pts[i]
		x1, y1 = pts[i+1]
		thetas.append(round(np.arctan((y1 - y0)/ (x1 - x0+ 1e-9))*180/np.pi -angle_wrt , 3))
		error_theta += abs(thetas[i] - angles[i]) 
	print(thetas)
	print(error_theta)

	for i in range(len(pts)-1):
		cv.line(img, pts[i], pts[i+1],(255, 0, 0), 1)
	cv.line(img, pts[len(pts)-1], end_point,(255, 0, 0), 1)


	path = []
	obst_hits = []
	for j in range(dimY):
		for i in range(dimX):
			if(img[j, i][0] == (255) and img[j, i][1] == 0 and img[j, i][2] == 0 ):
				path.append((i, j))
				if(grid[j, i]== 0):
					obst_hits.append((i, j))
					cv.circle(grid, (i, j), 2,(255, 0, 0), -1)

	print(path)
	print(len(path))

	print('Obst hits = ', len(obst_hits))
	print(obst_hits)

	# for i in range(len)




	cv.imshow('Vectorised input', img)
	cv.imshow('gr', grid)
	# cv.waitKey(0)


	cv.waitKey(0)

m = 15

def get_lat_dist(pts2):
	for pt1, pt2 in zip(pts1, pts2):
		x2, y2 = pt2
		x, y = pt1
		ld+= ((x2 - x)**2 + (y2 - y))**0.5

def devectorise(pts2, angles2):
	for i in range(len(pts2) - 1):
		pass

def hard_score


connect_point()