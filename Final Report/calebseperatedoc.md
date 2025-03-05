# ME 467 Project 1

Caleb Hottes

Charles Friley





OUTLINE
1. Introduction
	Overview of the Furuta pendulum
	Importance of kinematics in robotics
	Goals of the project
	Tools used (MuJoCo, mathematical methods)
2. Position-Level Kinematics (Question 1)
	Introduction
	Methodology
	Results
	Discussion
3. Velocity-Level Kinematics (Question 2)
	Introduction
	Methodology
	Results
	Discussion
4. Verification in MuJoCo (Question 3)
	Introduction
	Implementation details
	Validation of results
	Discussion
5. Simulation and Analysis (Question 4)
	Introduction
	Simulation setup (Runge-Kutta integration, applied torque)
	Results (height of end-effector, linear & angular velocity plots)
	System behavior analysis
6. Conclusion
	Summary of key findings
	Alignment with theoretical expectations
	Lessons learned and future improvements
 
## Position-Level Kinematics (Question 1)
### Introduction
In this section we will resolve the position-level forward and backwards kinematics of the Furuta pendulum with the help of the coordinate systems as shown in the figure. This will include representing rotation matrxes in 
### Methodology and Results
To find the position-level forward kinematics, we need 

The current goal is to represent frame 2 and frame c in respect to the world frame ${}^0T_c$ and  ${}^0T_2$. To do this we will construct the position and rotation matrixes by inspection. 
Fine pose of 01, 12, 2c, and then multiply them to get 0c
Pose is found by defining position and rotation matrixes by inspection, then constructing pose
Next, find the pose of c with given angles. Just plug angles into T0c – part a 
Extract rotation matrix, just the 3x3 part of the pose
Convert to quaternian representatation
Use the same process for pose of 2 - part b
Next, represent the rotation with axis angle – part c
Then recover the rotation matrix with rodreguez formula
Do the same for frame 2 
Next, find the angles that result in a given orientation of the z axis. Inverse kinematics of orientation of the z axis - part d
This is done by solving the inverse kinematics of the z part of the rotation. 
Z=k
3 equations, 3 unks, solve for theta1 and 2
4 solutions were found, so plug it back in to check and get the correct solutions
Next, find the angles that result in a given position, inverse kinematics of position - part e
Set up three equations, position = function of theta1 and theta2
4 solutions found
Plug them back in to check. 
Results
Discussion



## Position-Level Kinematics (Question 1)



## Velocity-Level Kinematics (Question 2)


### Introduction
Question two moves from the position level kinematics domain and asks us questions about the velocity of the pendulum. 


### Methodology

We started as recommended by resolving the forward and inverse kinematics.  

#### Forward Velocity Kinematics
The first step was to compute the spatial twist. In order to find the spatial twist we need to know the angular velocity $\mathsf{\omega}$ and the velocity *v* of the end-effector with respect to the world frame. We will fist consider the spatial case. 

##### Spatial
The spatial twist of frame c with respect to the world frame is given by ${}^0T_c^0=\dot{T}T^{-1}=$ From question one we have ${}^0T_c^0$ and can invert it, so all we need to find is $\dot{T}$.

We know that $\dot{T} = \begin{bmatrix}\dot{R} & \dot{t} \\\bold{0} & 0\end{bmatrix}$ so we need to find $\dot{R}$ and $\dot{t}$.

From the definition we know that ${}^0\dot{R}_c^0={}^0\hat{\omega}_c^0*{}^0R_c^0$. ${}^0R_c^0$ has already been found in question one, which 