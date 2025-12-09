# UAHAdaptiveControlResearch
## Overview
This repository contains the MATLAB/Simulink models, scripts, and supporting files for implementing and testing a Nonlinear Adaptive Sliding Mode Control (NASMC) algorithm on the Quanser Aero 2-DOF helicopter platform.<br><br>
The purpose of this project is to transition a previously theoretical NASMC approach into a hardware-realizable framework, validating its performance and robustness to parameter uncertainty.
The work is conducted under the upervision of Dr. Fahimi in the Robotics and Controls Laboratory at the University of Alabama in Huntsville as part of the Undergraduate Honors Thesis Requirement.
## Research Motivation
Conventional control strategies are often limited by model uncertainty and parameter variation. Adaptive control offers robustness to disturbances while continuously adjusting control parameters in real time.<br><br>
This work investigates whether NASMC, originally proposed for quadcopter applications, can be successfully adapted for the Quanser Aero test platform, which exhibits similar coupled pitch–yaw dynamics.
## Running The Simulation Model
1. Clone this repo:
```
git clone https://github.com/<your-username>/NASMC_QuanserAero.git
cd NASMC_QuanserAero
```
2. Open MATLAB and open the following scripts and models:
```
run_QuanserAero_AdaptiveV3_SN.m
model_QuanserAero_AdaptiveV3_SN.slx
```
3. Run the simulation from the MATLAB script:
```
run_QuanserAero_AdaptiveV3_SN.m
```
4. Outputs are displayed
## Running The Experiment on Quanser Aero
Not available...
## Aknowledgements
- Dr. Farbod Fahimi, Robotics and Controls Laboratory, University of Alabama in Huntsville
- Ryan Mathewson, foundational NASMC theoretical formulation
