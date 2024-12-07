# Dynamical simulation of tethered net capture of space debris

This repository contains source codes for my Bachelors' graduation thesis. I aimed to simulate the dynamic behaviors of a tethered net capturing a space debris.

The main project is the C++ simulation environment (./CPP), which includes:
  1. the simulation of net-deployment simulation (./CPP/Tether-net Deployment)
  2. simple capturing simulation of cylinder capturing (./CPP/Tether-net to Cylinder)
  3. complex capturing simulation of any triangle-meshed object (./CPP/Capture Dynamics)

Besides, I provided some simple examples of multi-body systems' simulation, such as Damped-Mass-Spring-System and Falling-Ball-to-Ground (./CPP).
This project contains easy Matlab examples for creating a Mass-Spring system, and visualizing the simulation result (./MATLAB).

## Requirements

The main simulations requires eigen-3.3.8 package to run.

## Simulation of Net Deployment

-- Net Deployment
![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/tree/master/Demonstrations-GIF/NetDeploy.gif)

-- Capture a cuboid
![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/blob/master/Demonstrations-GIF/CuboidCapture-1.gif)
![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/blob/master/Demonstrations-GIF/CuboidCapture-2.gif)

