Points =[(291, 376), (276, 361), (261, 349), (246, 344), (231, 333), (216, 334), (201, 325), (186, 310), (171, 295), (156, 280), (141, 265), (126, 250), (111, 235), (96, 220), (90, 205), (90, 190), (84, 175), (78, 160), (79, 145), (88, 130), (88, 115)]
Angles= [0.0, -2.121, -29.055, -6.843, 130.914, -15.255, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 21.801, 45.0, 25.346, 21.801, 49.086, 77.735, 45.0, 45.0]


ls = []
for i, j in zip(Points, Angles):
	ls.append((i, j))
print('x, y, theta =', ls)
