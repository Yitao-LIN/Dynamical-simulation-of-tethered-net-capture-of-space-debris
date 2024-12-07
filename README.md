# Dynamical Simulation of Tethered Net Capture of Space Debris

This repository contains the source code for my Bachelor's graduation thesis. The project focuses on simulating the dynamic behavior of a tethered net capturing space debris.
I apologize for the messy undergraduate coding and am willing to address any issues via email or GitHub.

## Project Structure

The main project is a C++ simulation environment located in the `./CPP` directory. It includes:

1. **Net Deployment Simulation** (`./CPP/Tether-net Deployment`): Simulates the deployment of the tethered net.
2. **Simple Capture Simulation** (`./CPP/Tether-net to Cylinder`): Simulates the capture of a cylindrical object.
3. **Complex Capture Simulation** (`./CPP/Capture Dynamics`): Simulates the capture of arbitrarily shaped objects represented as triangle meshes.

Additionally, the repository provides simple examples of multi-body system simulations, such as:
- **Damped Mass-Spring System**
- **Falling Ball to Ground**

These examples can also be found in the `./CPP` directory.

The project includes MATLAB scripts in the `./MATLAB` directory for creating mass-spring systems and visualizing simulation results.

## Requirements

The main simulations require the [Eigen 3.3.8](https://eigen.tuxfamily.org/) library and are designed to run on Windows 10/11.

## Simulation of Net Deployment

-- Net Deployment

![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/blob/master/Demonstrations-GIF/NetDeploy.gif)

-- Capture a cuboid

![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/blob/master/Demonstrations-GIF/CuboidCapture-1.gif)
![](https://github.com/Yitao-LIN/Dynamical-simulation-of-tethered-net-capture-of-space-debris/blob/master/Demonstrations-GIF/CuboidCapture-2.gif)

