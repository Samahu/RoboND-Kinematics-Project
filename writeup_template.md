## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[forward-kinematics-rviz]: ./misc_images/forward-kinematics-rviz.png
[forward-kinematics-urdf]: ./misc_images/forward-kinematics-urdf.png
[forward-kinematics]: ./misc_images/forward-kinematics.jpg
[inverse-kinematics-theta1]: ./misc_images/inverse-kinematics-theta1.jpg
[inverse-kinematics-theta2-theta3]: ./misc_images/inverse-kinematics-theta2-theta3.jpg
[success]: ./misc_images/success.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

In progress

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


The following image shows the model composition and links relationships taken using the rviz software
![alt text][forward-kinematics-rviz]

The following image matches the link lengths with their values taken from the urdf file.
![alt text][forward-kinematics-urdf]

Based on these values and the joints relationship and their types we can define the origin and the axes for each joint in a way that minimizes the number of DH parameters and values as much as possible, as shown in the following image:
![alt text][forward-kinematics]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The following table capture the DH parameters based on the previous images:

_Note_: I changed the table header from d(i-1) to d(i)

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | +pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The following figures shows the how to theta1, theta2, theta3 were derived:

TOP VIEW
![alt text][inverse-kinematics-theta1]

SIDE VIEW
![alt text][inverse-kinematics-theta2-theta3]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


First I start by defining some of the symbols needed to define the DH tables, then I construct the transforamtion matrices for the first three joints, here is the matrix for the first  joint, the other two are similar (this of course can be refactored into a method):

```python

T0_1 = Matrix([
[            cos(q1),             -sin(q1),        0,          a0],
[sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0),  -sin(alpha0),  -sin(alpha0)*d1],
[sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),   cos(alpha0),   cos(alpha0)*d1],
[                  0,         0,               0,           1]])
T0_1.subs(s)

```

Then I concat these three matrices to create a single transformation matrix that represents the total transformation:

```python
T0_3 = simplify(T0_1 * T0_2 * T0_3)
```

Next I contruct the Rypr matrix as follows:

```python
y, p, r = symbols("y p r")
R_ext_z = Matrix([
[  cos(y),    -sin(y),   0],
[  sin(y),     cos(y),  0],
[       0,         0,  1]])

R_ext_y = Matrix([
[     cos(p),    0,   sin(p)],
[     0,    1,       0],
[    -sin(p),    0,   cos(p)]])

R_ext_x = Matrix([
[       1,     0,         0],
[  0,     cos(r),  -sin(r)],
[  0,     sin(r),  cos(r)]])

R_ext = R_ext_z * R_ext_y * R_ext_x

```

I also create the correction matrix that is required to map arm transformation from gazebo simulation to our model:

```python
R_int_z = Matrix([
[         cos(pi),  -sin(pi),    0],
[    sin(pi),   cos(pi),    0],
[    0,         0,    1]])

R_int_y = Matrix([
[    cos(-0.5*pi),    0,  sin(-0.5*pi)],
[      0,    1,       0],
[   -sin(-0.5*pi),    0,   cos(-0.5*pi)]])


R_corr = R_int_z * R_int_y
```

Using these two matrices I construct the R0_6 matrix, using R_ext with R_corr, I then substitute the yaw, pitch, roll values passed down by the simulation:

```python
R0_6_sym = simplify(R_ext * R_corr)
R0_6 = R0_6_sym.evalf(subs={y: yaw, p: pitch, r: roll})
```

This allows me to compute Wrist Center (wc) location, which is crucial to computing the first 3 theta values:

```python
d_7 = s[d7]
wc_x = px - d_7 * R0_6[0, 2]
wc_y = py - d_7 * R0_6[1, 2]
wc_z = pz - d_7 * R0_6[2, 2]
```

The first 3 theta values are computed according to the analysis done in the previous section:

```python
theta1 = (atan2(wc_y, wc_x)).evalf()

a_1 = s[a1]
a_2 = s[a2]
a_3 = s[a3]
d_1 = s[d1]
d_4 = s[d4]

s1 = sqrt(a_3**2 + d_4**2)   #sqrt( (-0.054)^2 + (1.5)^2 )
s2 = sqrt(wc_x**2 + wc_y**2) - a_1
s3 = wc_z - d_1    #wc_z - 0.75
ss = sqrt(s2**2 + s3**2)

D1 = (ss**2 + a_2**2 - s1**2) / (2*ss*a_2)
beta1 = atan2(s3, s2)
beta2 = atan2(sqrt(1 - D1**2), D1)
theta2 = (0.5 * pi - beta1 - beta2).evalf()

D2 = (s1**2+a_2**2-ss**2) / (2*s1*a_2)
gamma1 = atan2(a_3, d_4)
gamma2 = atan2(sqrt(1 - D2**2), D2)
theta3 = (0.5 * pi - gamma1 - gamma2).evalf()
```

Then I sub these computed values in the map defining the DH table (dict s) and I use it compute R0_3:
```python
s.update({q1: theta1, q2: theta2 - 0.5 * pi.evalf(), q3: theta3})  # append the new values
R0_3 = T0_3.evalf(subs=s)
R0_3 = R0_3[:3, :3]
```

I then compute inv(R0_3) and multiply with R0_6 to get R3_6 matrix:
```python
R0_3_inv = R0_3.inv("LU")
R3_6 = simplify(R0_3_inv * R0_6)
```

Finally I use R3_6 matrix to get the remaining theta values 4, 5, 6 as follows:
```python
r11 = R3_6[0, 0]
r12 = R3_6[0, 1]
r13 = R3_6[0, 2]

r21 = R3_6[1, 0]
r22 = R3_6[1, 1]
r23 = R3_6[1, 2]

r31 = R3_6[2, 0]
r32 = R3_6[2, 1]
r33 = R3_6[2, 2]

theta4 = (atan2(r33, -r13)).evalf()
theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
theta6 = (atan2(-r22, r21)).evalf()

```

Using this code, the arm is able to locate and pick the object and drop it in the right place as shown in the following image:

![alt_text][success]

However, the main problem I face is that the execution is somewhat slow and this can be improved probably by moving more stuff out of the loop code or switch to using numpy.


