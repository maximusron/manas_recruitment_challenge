import numpy as np
import random
import cv2 as cv
dimX = 400
dimY = 400
num = 22
img = np.zeros((dimX, dimY,3), np.uint8)

img = cv.bitwise_not(img)

obstacles = [(x, y) for x, y in zip(list(np.random.randint(dimX, size=(num))),list(np.random.randint(dimY, size=(num))))]
for ob in obstacles:
	cv.circle(img, ob, 37, (0, 0, 0), -1)
print(obstacles)

dest = np.random.randint(dimX, size=(2))
x,y = dest
while (img[y, x][0] == 0 ):
	dest = np.random.randint(dimX, size=(2))
	x,y = dest

print(dest)


cv.imshow('map', img)
cv.waitKey(0)

