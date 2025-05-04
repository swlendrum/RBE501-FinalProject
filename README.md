# RBE501-FinalProject

This is the final project for RBE501-S25 Team 10.

## Abstract

This project seeks to validate and verify the findings in [FABRIK: A Fast, Iterative Solver for Inverse Kinematics](https://doi.org/10.1016/j.gmod.2011.05.003), by Andres Aristidou and Joan Lasenby. Their work asserts a novel approach to inverse kinematics, calculating a set of joint angles in a serial robotic arm given a goal pose. This solution utilizes a two-stage iterative solving method that constrains the possible joint angles to conform to a line between the base frame and end effector. We replicated the FABRIK method on a custom manipulator, which allowed us to compare this solver's performance to the previously developed methods.

## Instructions

To run this project, you must install [Robotics Toolbox by Peter Corke](https://petercorke.com/toolboxes/robotics-toolbox/), and have MATLAB installed on your local machine. 

- **runme.m** performs a demo of FABRIK using our custom manipulator, either for a spiral trajectory or from the provided waypointsNN.mat files.

- **runme3D.m** performs a demo of [FABRIK-R](http://dx.doi.org/10.1109/ACCESS.2021.3070693) using a 6-DOF manipulator on a spiral trajectory.

- **runme_originalMethods.m** performs a demo of the Newton-Raphson, Damped Least-Squares, and Gradient Descent inverse kinematic methods against our custom manipulator.

## Acknowledgements
We would like to acknowledge the authors of the original FABRIK algorithm for their work. We would also like to thank Professor Fichera for his guidance throughout RBE 501 - Robot Dynamics.