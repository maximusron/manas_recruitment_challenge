
import cv2 as cv
import json
import string
import sys
import numpy as np
import matplotlib.pyplot as plt


# def generate_map(obstacles, radii, costs):
#     grid = np.zeros((400, 400))
#     grid = cv.bitwise_not(gridsrc)
#     for obst, radius, cost in  zip(obstacles, radii, costs):
#         cv.circle(grid, obst, radius + int(car_diagonal/2), costs/10, -1)
#         i+=1
#     return grid
obstacles = []
car_diagonal = 0
def generate_map_easy(obstacles, car_diagonal):
    grid = np.zeros((400, 400))
    # grid = cv.bitwise_not(gridsrc)
    for obst in obstacles:
        cv.circle(grid, (obst[0],obst[1]), obst[2] + int(car_diagonal/2), 1, -1)
    return grid

def run_custom_checker():
    # Don't print anything to STDOUT in this function
    # Enter your custom checker scoring logic here

    # obstacles = []
    vectorised_path=[]
    vectorised_path_expected = []
    
    pts = []
    pts_expected = []
    thetas = []
    thetas_expected = []

    input_path = 'C:/Users/MaximusX/Desktop/manas_recruitment_challenge/input00.txt'
    count = len(open(input_path).readlines())
    # print(count)
    with open(input_path,'r') as fil:
        lines=fil.readlines()
        start_points = lines[0].strip().split(' ')
        start_x, start_y = start_points[0], start_points[1]
        end_points= lines[1].strip().split(' ')
        end_x, end_y = end_points[0], end_points[1]
        car_dims = lines[2].strip().split(' ')
        car_width, car_height = float(car_dims[0]), float(car_dims[1])
        vector_mag = float(lines[3])
        num_obstacles = int(lines[4])
        # print(num_obstacles)
        car_diagonal = (car_width**2 + car_height**2)**0.5
        for i in range(5, 5+num_obstacles):
            nums = lines[i].strip().split(' ')
            vector = []
            for x in nums:
                if(x !=''):
                    vector.append(int(x))
                if len(vector) == 3:
                    x,y,r = vector[0], vector[1], vector[2]
                    obstacles.append((x,y,r))
             
    with open('output00.txt','r') as fil:
        lines=fil.readlines()
        for line in lines:
            # x,y,theta=map(float,line.strip().split(' '))
            nums = line.strip().split(' ')
            vector = []
            for x in nums:
                if(x !=''):
                    vector.append(float(x))

            if len(vector) == 3:
                x,y,theta = vector[0], vector[1], vector[2]
                vectorised_path.append((x,y,theta))
                pts.append((x, y))
                thetas.append(theta)
    num_points = 0
    with open('expected_output00.txt','r') as fil:
        lines=fil.readlines()
        for line in lines:
            # x,y,theta=map(float,line.strip().split(' '))
            nums = line.strip().split(' ')
            vector = []
            for x in nums:
                if(x !=''):
                    vector.append(float(x))

            if len(vector) == 3:
                x,y,theta=vector[0], vector[1], vector[2]
                vectorised_path_expected.append((x,y,theta))
                pts_expected.append((x, y))
                thetas_expected.append(theta)
                num_points = num_points + 1

    with open('expected_output00.txt','r') as fil:
        lines=fil.readlines()
        for i in range(num_points, num_points+400):
            row = lines[i].strip().split(' ')
            # print(len(row))
            i = 0
            for x in row:
                if(x != ''):
                    row[i] = float(x)
            grid_parsed.append(row)
    print(np.array(grid_parsed).shape)  
    
    close_waypoints = 0
    close_thetas = 0
    
    if len(pts_expected) == len(pts):
        for pt1, pt2 in zip(pts, pts_expected):
            eucl = 0

            x1, y1 = pt1
            x2, y2 = pt2
            eucl += ((x1- x2)**2 + (y1-y2)**2)**0.5
            if(eucl < 25):
                close_waypoints+=1
        
        for theta1, theta2 in zip(thetas, thetas_expected):
            delta_theta = 0
            
            delta_theta += abs(theta1 - theta2)
            
            if(delta_theta < 3.5):
                close_thetas +=1

        # print("start points")
        # print(start_points)
        # print("End points")
        # print(end_points)
        # print("car dims")
        # print(car_dims)
        # print("Vector mag")
        # print(vector_mag)
        # print("Obstacles")
        # for k in obstacles:
        #     print(k)
    #     r_obj.score = 0.8 * float(close_waypoints/len(pts_expected)) + 0.2 * float(close_thetas/len(thetas_expected))
    #     score=close_waypoints/len(pts_expected)

    #     if(r_obj.score<0.5):
    #         r_obj.result=False
    #         r_obj.message="Failed"
    #     else:
    #         r_obj.result=True
    #         r_obj.message="Accepted"
    #         # r_obj.score=close_waypoints/len(pts_expected)

    #         # r_obj.message = str(score)
    # else:
    #     r_obj.result=False
    #     r_obj.message="Failed"
    #     r_obj.score = float(max(0, 0.5 - abs(len(pts) - len(pts_expected))*0.24))     

    

#     r_obj.result = True;
#     r_obj.score = 1.0;
#     r_obj.message = "Success";

# End of BODY
run_custom_checker()
# print(obstacles)
grid = generate_map_easy(obstacles, car_diagonal)
plt.imshow(grid)
plt.show()
mat = np.matrix(grid)
with open('outfile.txt','wb') as f:
    for line in mat:
        np.savetxt(f, line, fmt='%.2f')


# print(grid)
# k = input()
        