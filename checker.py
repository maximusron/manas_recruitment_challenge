from car_move import *

costmap = getCostMap()

img = costmap
copy = img.copy()
cv.imwrite('map_hard5.jpg', copy)
start_cost = (0, 0)
dest_cost = (359, 370)
car_move(costmap, start_cost, dest_cost)

def run_custom_checker(t_obj, r_obj):
    # Don't print anything to STDOUT in this function
    # Enter your custom checker scoring logic here
    vectorised_path=[]
    with open(t_obj.testcase_output_path,'r') as fil:
        path=fil.readlines()
        for i in path:
            x = path[i][0]
            y = path[i][1]
            theta = path[i][2]
            vectorised_path.append([x,y,theta])

    r_obj.result = True
    r_obj.score = 1.0
    r_obj.message = "Success"





    points = []
    output = []
    x_start = output[0][0]
    y_start = output[0][1]
    theta = output[0][2]
    output.pop(0)
    points.append([x_start,y_start])
    def devectorise(output):
        for vector in output:
            theta = vector[2]
            x = points[-1][0] + m*math.cos(theta)
            y = points[-1][1] + m*math.sin(theta)
            points.append([x,y])

    


         