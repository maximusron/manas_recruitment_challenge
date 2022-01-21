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
