# UWB-KF-Auto
Modeling localization using UWB technology with a Kalman Filter for vehicle applications

## Introduction

This repository contains a MATLAB file used to simulate UWB localization for automotive applications.  The file included `vary_anchors_45m.m` runs the model.

### Model Description

The model creates a circular path around a vehicle for the tag to follow. The vehicle is shown as a rectangle. Anchors are placed on the outside or inside of the vehicle. The number of anchors may be changed. Because this is a simulation, measurement noise is added to the distance from each point. This noise is a random Gaussian variable with a variance of 0.5 m^2. The measurements from the tag at time step _k_ to each anchor are used in a Least Squared algorithm. This model utilizes one of MATLAB's non-linear least squares functions to create the estimated position.

This position is then the measurement vector used in the Kalman Filter. After the Kalman filter, the LS estimates and Kalman estimates are plotted with the original path. The RMSE is calculated for both LS and the Kalman filter. If the script is being run for multiple iterations, the RMSE is averaged over those iterations.

The squared error of each time step _k_ is calculated and plotted against the angle of the location of the tag from the center of the vehicle. The front of the vehicle is toward the right (x increasing). 

## Variables

Many things can be changed in this model: 
 - The number of iterations. This is the variable _iter_. The higher the number of iterations, the longer the code will take to run, but the more accurate the RMSE values are.
 - The number of anchors. The anchors are stored in a matrix A. These are preset values at mostly equidistant locations throughout the vehicle. By changing the variable _m_ in line 10 to the number of anchors you'd like to use, the script will take the first _m_ values from the matrix. You can also change _m_ to be a vector of integers and the script will run consecutive times and store all of the RMSE values in the matrix _sweep_. 
 - The radius of the path. The radius of the path can be changed by editing the variable _r_. The radius is in meters.
 - The position of the anchors. If you'd like to change the position of the anchors, simply comment out the original A matrix and create a new one. Or edit the values in the current A matrix. Don't forget to make sure the number of anchors _m_ aligns with your matrix A. 

## Results

The model will output the followin:
- Figure 1 - This figure will show the vechicle, the anchors on the vehicle, the path of the tag, the LS estimates, and the Kalman estimates
- Figure 2 - This figure will show the squared error of the angle (in degrees) of each time step, averaged over all iterations ran.
- avgLSerr - This value is the average Least Squares Algorithm RMSE over all the iterations ran.
- avgKerr - This value is the average Kalman Filter RMSE over all the iterations ran.
