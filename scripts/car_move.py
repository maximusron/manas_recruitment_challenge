import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import math
from a_star import *
import random

dimX = 400
dimY = 400
car_width = 7
car_height = 12
car_diagonal = (car_width**2 + car_height**2)**0.5

img = np.zeros((dimX, dimY,3), np.uint8)

img = cv.bitwise_not(img)
#cv.imshow('win', img)
#cv.waitKey(0)
#obstacles = [(150, 240), (250, 300), (290, 150), (120, 120)]
obstacles1 = [(120, 240), (163, 309), (250, 300), (338, 99), (120, 120), (201, 182)]#, (), (), (), ()]
obstacles2 = [(120, 240), (163, 309), (250, 300), (338, 99), (120, 120), (201, 182), (59, 15), (40, 155)]#, (), ()]
obstacles3 = [(304, 285), (20, 372), (340, 217), (203, 73), (81, 81), (77, 160), (344, 88), (311, 19), (9, 56), (381, 365), (213, 52), (77, 301)]
obstacles4 = [(196, 83), (266, 233), (118, 361), (58, 28), (288, 136), (388, 276), (25, 198), (57, 373), (55, 173), (247, 277), (343, 244), (23, 333), (350, 171), (87, 263), (175, 117), (154, 123), (194, 228), (358, 362), (79, 81), (106, 21)]
obstacles5 = [(35, 312), (356, 41), (50, 46), (209, 316), (225, 272), (354, 283)]
obstacles6 =  [(14, 193), (233, 374), (202, 171), (237, 372), (393, 75), (133, 262), (300, 29), (28, 60), (324, 241), (374, 374), (95, 387), (5, 370), (162, 20), (199, 115), (259, 143), (165, 244), (300, 126), (317, 227), (190, 129), (357, 377), (159, 286), (271, 189)]

num = 23
obstacles3 = [(x, y) for x, y in zip(list(np.random.randint(dimX, size=(num))),list(np.random.randint(dimY, size=(num))))]

costs1 = list(np.random.uniform(0, 1, 10))
costs2 = [10, 10, 9, 7, 7, 7, 7, 4, 3, 3, 3, 4, 6 ,6, 3, 6, 6]

obstacles = obstacles6
costs = costs1

for obst in obstacles:
	cv.circle(img, obst, 20, (0, 0, 0), -1)

grid = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
for obst in obstacles:
	cv.circle(grid, obst, 25 + int(car_diagonal), (0, 0, 0), -1)
	#cv.circle(grid, obst, 25 , (0, 0, 0), -1)
map_easy = img.copy()

copy = img.copy()
"""
c = [[150, 240], [230, 300]]
rect = cv.minAreaRect(c)
box = cv.boxPoints(rect)
box = np.int0(box)
cv.drawContours(img,[box],0,(0,191,255),2)

"""

def draw_angled_rec(x0, y0, width, height, angle, img):

    _angle = angle * math.pi / 180.0
    b = math.cos(_angle) * 0.5
    a = math.sin(_angle) * 0.5
    pt0 = (int(x0 - a * height - b * width),
           int(y0 + b * height - a * width))
    pt1 = (int(x0 + a * height - b * width),
           int(y0 - b * height - a * width))
    pt2 = (int(2 * x0 - pt0[0]), int(2 * y0 - pt0[1]))
    pt3 = (int(2 * x0 - pt1[0]), int(2 * y0 - pt1[1]))
    #print(pt0, pt1, pt2, pt3)
    cv.line(img, pt0, pt1, (0, 255, 0), 3)
    cv.line(img, pt1, pt2, (0, 255, 0), 3)
    cv.line(img, pt2, pt3, (0, 255, 0), 3)
    cv.line(img, pt3, pt0, (0, 255, 0), 3)
    #plt.imshow(img)
    #plt.show()

def show_img():
	plt.imshow(img)
	plt.show()

def getgrid():
	mapgrid = grid.copy()

	mapgrid = cv.bitwise_not(mapgrid)
	mapgrid = mapgrid/255
	return mapgrid

def getCostMap():
	costmap = getgrid()
	num = 5
	obstacles3 = [(240, 120), (301, 160), (350, 100), (338, 99), (120, 120), (201, 182), (59, 15), (40, 155)]#, (), ()]
	obstacles3 = [(x, y) for x, y in zip(list(np.random.randint(dimX, size=(num))),list(np.random.randint(dimY, size=(num))))]

	obstacles = obstacles3
	for obst in (obstacles):
		cv.circle(costmap, obst, 25 + max(car_width, car_height), random.choice(costs), -1)
	plt.imshow( costmap)
	plt.show()
	return costmap
		
		
def get_custom_grid(grid):
	mapgrid = grid.copy()

	mapgrid = cv.bitwise_not(mapgrid)
	mapgrid = mapgrid/255
	return mapgrid


#draw_angled_rec(150, 200, 5, 10, 30, img)
#show_img()


#plt.imshow(mapgrid)
#plt.show()
start = (0, 0)
#start = (350, 50)

dest = (200, 368)

mapview = img.copy()

pts = []
angles = []

def vectorise(current_pts):
	x1, y1 = last_pt = current_pts[len(current_pts) -1]
	x0, y0 = first_pt = current_pts[0]
	theta = np.arctan((y1 - y0) / ((x1 - x0) + 1e-9)) * 180/ (np.pi)
	if theta < 0:
		theta+=180
	print('<x_h, y_h> = ', '<', (x0),',',(y0),'>')
	pts.append((x0, y0))
	print(round(theta-45, 3), '°')
	angles.append(round(theta-45, 3))
	cv.line(copy, first_pt, last_pt, (255, 0, 0), 2)


def car_move(map_lvl, start, dest):
	global img, copy
	mapgrid = map_lvl
	mapview = img

	print('Generating path...')
	path = astar(mapgrid, start, dest)
	print('Generated path')
	prev_pt = start


	hops = 0
	current_pts = []

	for pt in path:
		mapview = copy.copy()
		#img = mapgrid.copy()
		y = pt[1]
		x = pt[0]
		img[int(y), int(x)] = 0
		x0, y0 = prev_pt
		#print('prev_pt', )

		delx = x- x0
		dely = y - y0
		#print('delta: ',delx, dely)

		if delx >= 1 and dely >= 1:
			angle = 135
		elif delx == 0 and dely ==1:
			angle = 180


		else:
			angle = 30

		prev_pt = x, y
		draw_angled_rec(x, y, car_width, car_height, angle, mapview)
		

		cv.imshow('a', mapview)
		cv.waitKey(2)
		#cv.imshow('vectors ongrid', grid)
		#cv.waitKey(2)

		#print(pt)

		current_pts.append(pt)
		hops+=1
		if(hops % 15 == 0):
			print('vectorising')
			vectorise(current_pts)
			current_pts = []

def easy_level():
	mapgrid = getgrid()

	start= (0, 0)
	dest = (200, 368)

	car_move(mapgrid, start, dest)


	#mapgrid = costmap
	
		
def hard_level():
	global img, copy
	costmap = getCostMap()

	img = costmap
	copy = img.copy()
	cv.imwrite('map_hard5.jpg', copy)

	start_cost = (0, 0)
	dest_cost = (359, 370)

	car_move(costmap, start_cost, dest_cost)


def do_level():
	img_map = cv.imread('map_hard1.jpg')
	get_custom_grid(img_map)


def euclid_dist(start, dest):
	x1, y1 = dest
	x0, y0 = start
	return ((x1-x0)**2 + (y1-y0)**2)**0.5

def display_tests(map_color, start, dest):
	map_level = cv.cvtColor(map_color, cv.COLOR_BGR2GRAY)

	mapcpy = map_color.copy()

	map_level = cv.bitwise_not(map_level)

	map_level = map_level/255

	print('Generating path...')
	path = astar(map_level, start, dest)
	print('Generated path')
	prev_pt = start


	hops = 0
	current_pts = []

	for pt in path:
		mapview = map_color.copy()
		#img = mapgrid.copy()
		y = pt[1]
		x = pt[0]
		img[int(y), int(x)] = 0
		x0, y0 = prev_pt
		#print('prev_pt', )

		delx = x- x0
		dely = y - y0
		#print('delta: ',delx, dely)

		if delx >= 1 and dely >= 1:
			angle = 135
		elif delx == 0 and dely ==1:
			angle = 180


		else:
			angle = 30

		prev_pt = x, y
		draw_angled_rec(x, y, car_width, car_height, angle, mapview)
		

		cv.imshow('a', mapview)
		cv.waitKey(2)
		#cv.imshow('vectors ongrid', grid)
		#cv.waitKey(2)

		#print(pt)

		current_pts.append(pt)
		hops+=1
		if(hops % 15 == 0):
			print('vectorising')
			x1, y1 = last_pt = current_pts[len(current_pts) -1]
			x0, y0 = first_pt = current_pts[0]
			theta = np.arctan((y1 - y0) / ((x1 - x0) + 1e-9)) * 180/ (np.pi)
			if theta < 0:
				theta+=180
			print('<x_h, y_h> = ', '<', (x0),',',(y0),'>')
			pts.append((x0, y0))
			print(round(theta-45, 3), '°')
			angles.append(round(theta-45, 3))
			cv.line(map_color, first_pt, last_pt, (255, 0, 0), 2)

			current_pts = []



def generate_test_cases():
	test = np.zeros((dimX, dimY, 3), np.uint8)

	test = cv.bitwise_not(test)

	obstacles3 = [(x, y) for x, y in zip(list(np.random.randint(dimX, size=(num))),list(np.random.randint(dimY, size=(num))))]

	# costs1 = list(np.random.uniform(0, 1, 10))
	# costs2 = [10, 10, 9, 7, 7, 7, 7, 4, 3, 3, 3, 4, 6 ,6, 3, 6, 6]

	obstacles = obstacles3
	radii = []

	for obst in obstacles3:
		radius = np.random.randint(4, 42-car_diagonal)
		#cv.circle(test, obst, int(10+car_diagonal), (0, 0, 0), -1)
		cv.circle(test, obst, int(radius+car_diagonal), (0, 0, 0), -1)
		radii.append(radius)

	test_grid = cv.cvtColor(test, cv.COLOR_BGR2GRAY)
	# FINDING START AND END POINT

	start_x = np.random.randint(dimX)
	start_y = np.random.randint(dimX)
	start = start_x, start_y
	while (test_grid[start_y, start_x] != 255 ):
		start_x, start_y = random.sample(range(0, dimY), 2)
		print('iterating start')
		start = start_x, start_y

	dest_x, dest_y = random.sample(range(0, dimY), 2)
	dest = dest_x, dest_y
	while (test_grid[dest_y, dest_x] != (255) or  euclid_dist(start, dest) < 0.7* dimX):
		dest_x, dest_y = random.sample(range(0, dimY), 2)
		print('iterating dest')
		dest = dest_x, dest_y

	print('car_width', car_width)
	print('car_height', car_height)
	print('Obstacles', obstacles3)
	print('Radii', radii)
	print('Start :', start)
	print('End : ', dest)
	plt.imshow(test)
	plt.show()
	#car_move(test, start, dest)
	display_tests(test, start, dest)
	#cv.imwrite('test_easy1.jpg', img)

#easy_level()
#hard_level()

generate_test_cases()
print(pts)
print(angles)

#cv.imwrite('map1_easy.jpg', map_easy)


#TODO: ANSHUMAN
# Smoothen out path
# Ensure minimum metric
# Potential