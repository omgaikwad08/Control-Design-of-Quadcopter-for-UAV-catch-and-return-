# Assessment-of-Linear-Quadratic-Regulator-for-Quadrotor

Basic implementation of Linear Quadratic Regulator for a Quadrotor

## Abstract

The proposed project is an implementation of the Linear Quadrotor Regulator (LQR) theory for the control of a quadrotor that spans the given restricted airspace. A control model is proposed such that the quadrotor can withstand any disturbances that it may experience during trajectory traversal. Moroever, the idea focuses on the capture and retrieval of unknown aerial intruders such as a UAV. In control design approaches, control systems that provide swiftness in responses to disturbances are preferred. Various factors such as response time, launch sequence, trajectory control, capture aptness etc. determine the type of controller to be used. As for the context of this project, we have chosen a Linear Quadratic Regulator to execute the assigned goal of "capture and return".The non-linear dynamic model of the quadrotor has been derived and further linearised using a first-order Taylor series expansion. The success of the proposed controller has been defined through a series of experiments and results.

## Simulation

For demonstartion purpose, run the following command in MATLAB Command Window:
```bash
Quadrotor_LQR_Sim.m
