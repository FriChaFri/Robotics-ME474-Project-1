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

In this section we will resolve the position-level forward and inverse kinematics of the Furuta pendulum with the help of the coordinate systems as shown in the figure. 

Parts a, b, and c of question 1 will be shown simultaneously and parts d and e will be shown seperatly after. 

### Methodology and Results

#### Parts A, B, and C

To start, we will first find the the position level kinematics of frame c and frame 2 in respect to frame 0 (${}^0T_c$ and  ${}^0T_2$). To do this we first construct all the rotation and position matrixes from frame 0 to frame c by inspection. 

The positions can be constructed by inspection. 

${}^0t_1 = \left[\begin{matrix}0\\0\\d_{1}\end{matrix}\right]$, ${}^1t_2 = \left[\begin{matrix}0\\0\\d_{2}\end{matrix}\right]$, ${}^2t_c = \left[\begin{matrix}0\\0\\d_{3}\end{matrix}\right]$

The rotations are simple to construct.

From 0 to 1, we first rotate about $x$ by $\pi/2$, then we rotate about $y$ by $\theta_{1}$.

$R_{x, \pi/2} R_{y, \theta_{1}} = {}^0R_1 = \left[\begin{matrix}\cos{\left(\theta_{1} \right)} & 0 & \sin{\left(\theta_{1} \right)}\\\sin{\left(\theta_{1} \right)} & 0 & - \cos{\left(\theta_{1} \right)}\\0 & 1 & 0\end{matrix}\right]$

From 1 to 2, we first rotate about $x$ by $\pi/2$, then we rotate about $y$ by $\theta_{2}$.

$R_{x, \pi/2} R_{y, \theta_{2}} = {}^1R_2 = \left[\begin{matrix}\cos{\left(\theta_{2} \right)} & 0 & \sin{\left(\theta_{2} \right)}\\\sin{\left(\theta_{2} \right)} & 0 & - \cos{\left(\theta_{2} \right)}\\0 & 1 & 0\end{matrix}\right]$

From 2 to c, we just rotate about $z$ by $-\pi/2$.

$R_{z, -\pi/2} = {}^2R_c = \left[\begin{matrix}0 & 1 & 0\\-1 & 0 & 0\\0 & 0 & 1\end{matrix}\right]$

From these, the homogenous representation of the pose can be constructed.

$ {}^0T_1 = \left[\begin{matrix}\cos{\left(\theta_{1} \right)} & 0 & \sin{\left(\theta_{1} \right)} & 0\\\sin{\left(\theta_{1} \right)} & 0 & - \cos{\left(\theta_{1} \right)} & 0\\0 & 1 & 0 & d_{1}\\0 & 0 & 0 & 1\end{matrix}\right]$

$ {}^1T_2 = \left[\begin{matrix}\cos{\left(\theta_{2} \right)} & 0 & \sin{\left(\theta_{2} \right)} & 0\\\sin{\left(\theta_{2} \right)} & 0 & - \cos{\left(\theta_{2} \right)} & 0\\0 & 1 & 0 & d_{2}\\0 & 0 & 0 & 1\end{matrix}\right]$

$ {}^2T_c = \left[\begin{matrix}0 & 1 & 0 & 0\\-1 & 0 & 0 & 0\\0 & 0 & 1 & d_{3}\\0 & 0 & 0 & 1\end{matrix}\right]$

With these poses the desired pose can be constructed.

$^{0}T_{c} = {}^{0}T_{1} \, {}^{1}T_{2} \, {}^{2}T_{c}$

$^{0}T_{2} = {}^{0}T_{1} \, {}^{1}T_{2} $


Which results in:

$$^{0}T_{c} = \left[\begin{matrix}- \sin{\left(\theta_{1} \right)} & \cos{\left(\theta_{1} \right)} \cos{\left(\theta_{2} \right)} & \sin{\left(\theta_{2} \right)} \cos{\left(\theta_{1} \right)} & d_{2} \sin{\left(\theta_{1} \right)} + d_{3} \sin{\left(\theta_{2} \right)} \cos{\left(\theta_{1} \right)}\\\cos{\left(\theta_{1} \right)} & \sin{\left(\theta_{1} \right)} \cos{\left(\theta_{2} \right)} & \sin{\left(\theta_{1} \right)} \sin{\left(\theta_{2} \right)} & - d_{2} \cos{\left(\theta_{1} \right)} + d_{3} \sin{\left(\theta_{1} \right)} \sin{\left(\theta_{2} \right)}\\0 & \sin{\left(\theta_{2} \right)} & - \cos{\left(\theta_{2} \right)} & d_{1} - d_{3} \cos{\left(\theta_{2} \right)}\\0 & 0 & 0 & 1\end{matrix}\right]$$

$$ ^{0}T_{2} = \left[\begin{matrix}\cos{\left(\theta_{1} \right)} \cos{\left(\theta_{2} \right)} & \sin{\left(\theta_{1} \right)} & \sin{\left(\theta_{2} \right)} \cos{\left(\theta_{1} \right)} & d_{2} \sin{\left(\theta_{1} \right)}\\\sin{\left(\theta_{1} \right)} \cos{\left(\theta_{2} \right)} & - \cos{\left(\theta_{1} \right)} & \sin{\left(\theta_{1} \right)} \sin{\left(\theta_{2} \right)} & - d_{2} \cos{\left(\theta_{1} \right)}\\\sin{\left(\theta_{2} \right)} & 0 & - \cos{\left(\theta_{2} \right)} & d_{1}\\0 & 0 & 0 & 1\end{matrix}\right] $$

Plugging in the values of $\theta$ from part a of question 1,  $\theta_1 = \pi/3$ and $\theta_2=-3\pi/7$  we find that:

$$ ^{0}T_{c} = \left[\begin{matrix}-0.5 & -0.193 & 0.844 & 1.075\\-0.866 & 0.111 & -0.487 & 0.303\\0.0 & -0.975 & -0.223 & 1.022\\0.0 & 0.0 & 0.0 & 1.0\end{matrix}\right] $$

$$ ^{0}T_{2} = \left[\begin{matrix}-0.193 & 0.5 & 0.844 & 0.4\\0.111 & 0.866 & -0.487 & 0.693\\-0.975 & 0.0 & -0.223 & 1.2\\0.0 & 0.0 & 0.0 & 1.0\end{matrix}\right] $$

The rotation matrix can be extracted:

$$ ^{0}R_{c} = \left[\begin{matrix}-0.5 & -0.193 & 0.844\\-0.866 & 0.111 & -0.487\\0.0 & -0.975 & -0.223\end{matrix}\right] $$

$$ ^{0}R_{2} = \left[\begin{matrix}-0.193 & 0.5 & 0.844\\0.111 & 0.866 & -0.487\\-0.975 & 0.0 & -0.223\end{matrix}\right] $$

Which can then be represented both as a unit quaternion and as a axis-angle. We used the `r2q()` function for the quaternion representation and `tr2angvec()` function for the axis angle representation, both from Spatial Math.

The quaternion representing $^{0}R_{c}$ is: 

$$\ 0.3117 < -0.3909,  0.6771, -0.5400 > $$

The quaternion representing $^{0}R_{2}$ is: 
$$  0.6022 <  0.2024,  0.7552, -0.1614 > $$

The axis angle representing $^{0}R_{c}$ is:

$$ Angle: 2.508 \ rad \\ Axis: \left[\begin{matrix}-0.411 & 0.713 &  -0.568\end{matrix}\right]$$

The axis angle representing $^{0}R_{2}$ is:
$$ Angle: 1.849 \ rad \\ Axis: \left[\begin{matrix}0.253 & 0.946 & -0.202\end{matrix}\right]
$$


The rotation matrix can be recovered from the axis-angle representation by using rodrigues formula.

$ R = e^{\widehat{\omega} \theta} = \mathbf{I} + \sin(\theta) \widehat{\omega} + (1 - \cos(\theta)) \widehat{\omega}^2 $

Where:
- $\omega$ is the **axis of rotation**, a unit vector that defines the direction of rotation.
- $\theta$ is the **scalar part of the axis-angle rotation**, representing the rotation magnitude in radians.
- $\widehat{\omega}$ is the **skew-symmetric matrix** of $\omega$, used to compute the rotation matrix.
- $\mathbf{I}$ is the **identity matrix**.

Implementing this formula allowed us to recover the rotation matrix. 

$$ ^{0}R_{c} = \left[\begin{matrix}-0.5 & -0.193 & 0.844\\-0.866 & 0.111 & -0.487\\0.0 & -0.975 & -0.223\end{matrix}\right] $$

$$ ^{0}R_{2} = \left[\begin{matrix}-0.193 & 0.5 & 0.844\\0.111 & 0.866 & -0.487\\-0.975 & 0.0 & -0.223\end{matrix}\right] $$

#### Part D
If camera is installed at the end-effector of the Furuta pendulum, what is the value of $ \theta_1 $ and $ \theta_2 $ if the $ z $-axis of the camera points in the direction $ k $, where

$$
k = -\frac{1}{2} x_0 + \frac{1}{2} y_0 - \frac{\sqrt{2}}{2} z_0.
$$

In order to solve this question, we need to find the solutions for $\theta_1$ and $\theta_2$ in this equation:

$$
\begin{bmatrix} 
\ {}^0z_{c,1} \\ 
\ {}^0z_{c,2} \\ 
\ {}^0z_{c,3} 
\end{bmatrix} 
=
\begin{bmatrix} 
k_1 \\ 
k_2 \\ 
k_3 
\end{bmatrix}
$$
Where $^0z_c$ is the $z$ part of  $^{0}R_{c}$. Plugging in values:
$$
\begin{bmatrix} 
\sin{\left(\theta_{2} \right)} \cos{\left(\theta_{1} \right)} \\ 
\sin{\left(\theta_{1} \right)} \sin{\left(\theta_{2} \right)} \\ 
- \cos{\left(\theta_{2} \right)} 
\end{bmatrix} 
=
\begin{bmatrix} 
-0.5 \\ 
0.5 \\ 
- \frac{\sqrt{2}}{2} 
\end{bmatrix}
$$

We plugged these formulas into the `solve()` function in SymPy. This produced four potential solutions, which we then checked by plugging those $\theta$ values back into the forward kinematic equations to see if the resulting orientation matches that of the desired orientation. Only two of the four solution pairs were correct:

$$
\begin{align*}
(\theta_1, \theta_2) &= (-0.785, 5.498),\\
                     &\quad \ (2.356, 0.785)
\end{align*}
$$

Two solutions should be expected, since the end effector can reach the same z-axis orientation from two separate positions. 

#### Part E

In this question we are asked to find the value of $ \theta_1 $ and $ \theta_2 $ if the position of the camera is located at $p = 1.075x_0 + 0.303y_0 + 1.022z_0$ in respect to the world frame. Solving this is similar to the solution path layed out in part d. First, we will find the inverse kinematic equations for position, then plug in our position to solve.

The desired position is given by:

$ p={}^0t_c = [{}^0x_c, {}^0y_c, {}^0z_c] $

Which can be extracted from the pose ${}^0T_c$. Each part is a function of $\theta_1$ and $\theta_2$:

$ {}^0x_c = f(\theta_1, \theta_2) $

$ {}^0y_c = f(\theta_1, \theta_2) $

$ {}^0z_c = f(\theta_1, \theta_2) $



From these three equations, $ \theta_1 $ and $ \theta_2 $ can be solved to achieve the desired position.

Plugging in these equations into the SymPy `solve()` function yielded four solutions, which are lengthy. **Only two of these solutions will be valid, but we will not find out which until we plug in the numbers after.** 

---
Potential Solution 1:

$ \theta_1 = - 2 \operatorname{atan}{\left(\frac{d_{3} \sqrt{\frac{\left(- d_{1} + d_{3} + {}^0z_c\right) \left(d_{1} + d_{3} - {}^0z_c\right)}{d_{3}^{2}}} - \sqrt{- d_{1}^{2} + 2 d_{1} {}^0z_c + d_{2}^{2} + d_{3}^{2} - \left({}^0y_c\right)^{2} - \left({}^0z_c\right)^{2}}}{d_{2} - {}^0y_c} \right)} $

$ \theta_2 = \operatorname{acos}{\left(\frac{d_{1} - {}^0z_c}{d_{3}} \right)} $
___

Potential Solution 2:

$ \theta_1 = 2 \operatorname{atan}{\left(\frac{d_{3} \sqrt{\frac{\left(- d_{1} + d_{3} + {}^0z_c\right) \left(d_{1} + d_{3} - {}^0z_c\right)}{d_{3}^{2}}} - \sqrt{- d_{1}^{2} + 2 d_{1} {}^0z_c + d_{2}^{2} + d_{3}^{2} - \left({}^0y_c\right)^{2} - \left({}^0z_c\right)^{2}}}{d_{2} - {}^0y_c} \right)} $

$ \theta_2 = - \operatorname{acos}{\left(\frac{d_{1} - {}^0z_c}{d_{3}} \right)} + 2 \pi $
___

Potential Solution 3:

$ \theta_1 = - 2 \operatorname{atan}{\left(\frac{d_{3} \sqrt{\frac{\left(- d_{1} + d_{3} + {}^0z_c\right) \left(d_{1} + d_{3} - {}^0z_c\right)}{d_{3}^{2}}} + \sqrt{- d_{1}^{2} + 2 d_{1} {}^0z_c + d_{2}^{2} + d_{3}^{2} - \left({}^0y_c\right)^{2} - \left({}^0z_c\right)^{2}}}{d_{2} - {}^0y_c} \right)} $

$ \theta_2 = \operatorname{acos}{\left(\frac{d_{1} - {}^0z_c}{d_{3}} \right)} $
___

Potential Solution 4:

$ \theta_1 = 2 \operatorname{atan}{\left(\frac{d_{3} \sqrt{\frac{\left(- d_{1} + d_{3} + {}^0z_c\right) \left(d_{1} + d_{3} - {}^0z_c\right)}{d_{3}^{2}}} + \sqrt{- d_{1}^{2} + 2 d_{1} {}^0z_c + d_{2}^{2} + d_{3}^{2} - \left({}^0y_c\right)^{2} - \left({}^0z_c\right)^{2}}}{d_{2} - {}^0y_c} \right)} $

$ \theta_2 = - \operatorname{acos}{\left(\frac{d_{1} - {}^0z_c}{d_{3}} \right)} + 2 \pi $

---

Plugging in our values from $p$ results in 4 soulution pairs, which were then checked by plugging in the resulting theta values back into the forward kinematic equations and checking to see if the positions matched up. Only two of the solutions brought correct solutions for theta, which are:

$$
\begin{align*}
(\theta_1, \theta_2) &= (1.073, 1.346),\\
                     &\quad \ (2.618, -1.346) 
\end{align*}
$$

Two solutions should be expected, since the end effector can reach the same position from two separate orientations. 



### Position-Level Kinematics (Question 1)



### Velocity-Level Kinematics (Question 2)


#### Introduction
Question two moves from the position level kinematics domain and asks us questions about the velocity of the pendulum. 


#### Methodology

We started as recommended by resolving the forewrd and inverse kinematics.  