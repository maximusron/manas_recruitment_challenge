import cv2 as cv
car_diagonal = (car_width**2 + car_height**2)**0.5
def generate_map(obstacles, radii, costs):
	#costs is a list with values from 1-10, 10 being highest cost (unpassable) and 1 being lowest cost that is greater than 0 (no cost)
	grid = np.zeros((400, 400))
	grid = cv.bitwise_not(gridsrc)
	for obst, radius, cost in  zip(obstacles, radii, costs):
		cv.circle(grid, obst, radius + int(car_diagonal/2), costs/10, -1)
		i+=1

	return grid