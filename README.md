# EBSHQSim
Event-Based Sensing in Humphreys' Quadrotor Simulation (EBSHQSim). An forked/editted version of a MATLAB simulation made in the UT-Austin Aerial Robotics course in S2021 instructed by [Todd Humphreys](https://www.ae.utexas.edu/people/faculty/faculty-directory/humphreys) (Radionavigation Laboratory). Forked to support research in Event-Based Control/Sensing at The University of Texas at Austin under the appointment of [Takashi Tanaka](https://sites.utexas.edu/tanaka/about/)

# Acknowlwdgement of Credits
It is important to note that, although I am the one uploading this repository for research, credits are due to many other supporters during its original creation in Spring 2021 and during its update in Summer 2021 when I was under the assignment of the Tanaka Lab. Throughout this repository, there is authorship from the following sources as noted in each file:
 
- Todd Humphreys, members of the [Radionavigation Laboratory](https://radionavlab.ae.utexas.edu/), and TAs in the Aerial Robotics Course 
- Alireza Pedram, [Tanaka Lab](https://sites.utexas.edu/tanaka/people/)

# Installation and Running
In terms of installation, [MATLAB](https://www.mathworks.com/products/matlab.html) is the only external requirement for utilization of this repository. Any additional requirements which may have been installed during my original MATLAB installation will be notified when attempting to run - in the MATLAB command line.


To run the simulation, navigate to the [topSimulateControl.m](./topSimulateControl.m) file. This simulation requires a *.mat file be loaded prior to running. This *.mat file is loaded on line 35, and requires 3 variables of equal time-indexed length:
- t_ext: 1xn time vector
- x_ext: Either 3xn or 2xn column-stacked position matrix
- P_ext: Either 3x3xn or 2x2xn Cov Matrix Stack, respective to dimensions of x_ext

run the entire [topSimulateControl.m](./topSimulateControl.m) file from the MATLAB editor tab (or by keybinding or MATLAB command line) to collect and visualize data. 
