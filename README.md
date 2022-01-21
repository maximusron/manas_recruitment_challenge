It's time for Project MANAS to upgrade their PC parts but unfortunately they cannot be shipped to us. AI members Ishika and Aryan decide that they would drive down to the seller and collect it themselves. Since both of them do not have a driving licence it's up to EVE, our self driving car to take them to their destined location. 


Your mission, should you choose to accept it would be to write a vector based navigation solution that avoids all obstacles on their way to the store.
Due to the extremely short battery life of EVE and the fact that Ishika was too lazy to fully charge it, ensure that nav output is the shortest possible path.
EVE uses a controller to navigate which relies on a heading angle and position to correct the path.


The objective would be to output the position and heading angle of EVE at points along your generated path at intervals of a fixed magnitude.
Note that you will be penalised for vectors that enter obstacle zones as well as for discontinuous paths.


![image](https://s3.amazonaws.com/hr-assets/0/1636193192-e7e01e1b8b-Screenshotfrom2021-11-0615-28-08.png)

### Input Format

x<sub>1</sub>,  y<sub>1</sub> - Start coordinates

x<sub>2</sub>,  y<sub>2</sub> - End coordinates

l, w- Car dimensions

m - Vector magnitude

n - Number of obstacles

x<sub>i</sub>,  y<sub>i</sub>,  r<sub>i</sub> - Obstacle locations and radius

### Constraints

(0, 0) < x<sub>1</sub>,  y<sub>1</sub> < (400, 400)

(0, 0) < x<sub>2</sub>,  y<sub>2</sub> < (400, 400)

(5, 10) < w,l < (20, 30)

10 < m < 50 

7 < n < 50 

1 < r<sub>i</sub> < 42

(0, 0) < x<sub>i</sub>, y<sub>i</sub> < (400, 400)

### Output Format

A list of vectors representing the traversed path in terms of tail coordinates and heading angle. Each vector has the format:

x<sub>j</sub>, y<sub>j</sub>, &theta;<sub>j</sub>

Where x<sub>j</sub>, y<sub>j</sub> is the position and  &theta;<sub>j</sub> is the angle (with respect to the x-axis) of the j<sup>th</sup> vector.

## HARD

In this question, each obstacle has a predefined traversability value (or 'cost') between 1 and 10. For example, shallow puddles can quite easily be passed (they have low cost = 1), sand is more difficult but still traversable, while rocks are impossible to pass through (with the highest cost = 10). Since EVE has sensitive electronic components, driving over difficult obstacles can cause a lot of damage to our club's hard work (and yearly budget).

Avoiding all obstacles which are impossible to pass through (with cost = 10), find an optimum path that causes least damage to EVE. Don't forget that EVE also has low charge, so a very long path is not a great idea!

### Input Format

x1, y1 - Start coordinates

x2, y2 - End coordinates

l, w- Car dimensions

m - Vector magnitude

n - Number of obstacles

xi, yi, ri, costi - Obstacle locations (x,y), radius, and cost

### Constraints

(0, 0) < x1, y1 < (400, 400)

(0, 0) < x2, y2 < (400, 400)

(5, 10) < w,l < (20, 30)

10 < m < 50

7 < n < 50

(0, 0) < xi, yi < (400, 400)

1 < ri < 42

0 < costi < 10

### Output Format

A list of vectors representing the traversed path in terms of tail coordinates and heading angle. Each vector has the format:

xj, yj, θj

Where xj, yj is the position and θj is the angle (with respect to the x-axis) of the jth vector.
