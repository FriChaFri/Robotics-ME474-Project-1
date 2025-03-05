# RoboticsClassProject1
Caleb Hottes

Charles Friley

Python version 3.12.8  

Packages:

    pip install numpy, matplotlib, jupyter, mujoco, spatialmath-python, control, sympy, scipy, time, os

To run the MuJoCo file, run `python -m mujoco.viewer` and then drag and drop the "FurturaPendulum.xml" file inside the viewer. The position that is loaded automatically is the zero position. Keyframe 0 loads the Home position at $\theta_1 = \pi/2$ and $\theta_2=0$. Keyframe 1 loads the position outlined in part a of question 1, $\theta_1 = \pi/3$ and $\theta_2=-3\pi/7$.

## Overview of important files

- Final Report
    - **Friley_Hottes_Project1.md** $\rightarrow$ This file is the report of the whole project. It contains rigorous details of all calculations and includes results of mujoco verifications and the simulations. Its a great place to start. 
    - assets $\rightarrow$ This directory contains the images used in the markdown report. 
    - Friley_Hottes_Project1.pdf $\rightarrow$ A pdf export of the final report. 
- notebooks
  - FurturaPendulum.xml $\rightarrow$ This is the model of the pendulum for use with mujoco
  - Part4.ipynb $\rightarrow$ This file contains the code powering the simulation for question 4
  - Question1+2_final.ipynb $\rightarrow$ This file contains the python code used to perform the position and velocity level analysis of the problem. 
  - Question3.ipynb $\rightarrow$ This file contains the mujoco code that verifies the solutions to the position and velocity level kinematics. 
  - scuffedMujoCO.SLDPRT $\rightarrow$ This file is a Solidworks part which contains a model of the problem as a 3D sketch. It was very useful early on for gaining a intuitive understanding of the problem and gut-checking various solutions. 
  - util.py $\rightarrow$ This file contains a function that is used to print latex for ease of creating the final report. 
- Instructions $\rightarrow$ This directory contains the given instructions for the assignment for easy reference. 
- HTML Export of Questions 1-4 $\rightarrow$ This directory contains HTML versions of the jupyter notebook files for easy viewing. 