# Unscented Kalman Filter Project
The goal of this project is to implement a UKF algorithm to estimate the state of a moving object of interest with noisy lidar and radar measurements. The algorithm reads sensor telemetry from Laser and Radar and has to fuse the data to estimate the position and velocity of the moving abject. The RMSE values of the state estimation has to be less than [.09, .10, .40, .30]

## Build & Run
Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

    * `mkdir build`
    * `cd build`
    * `cmake ..`
    * `make`
    * `./UnscentedKF`



Also, This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Implementation
To do the sensor fusion, my algorithm follows the general processing flow as taught in the UKF course. It handles the first measurements appropriately based on the sensor type. Then, at each iteration, it predict the next state and then update it. The algorithm can handle radar and/or lidar measurements:

### State representation:
I used CTRV model for state representation
  * x
  * y
  * v
  * psi
  * psi_dot

### Initialization:
   * Initializing the state `x_` with the first measurement.It can be lidar or radar.
   * Createing the covariance matrix. I used the following `P_` matrix:
             `0.1,   0.0,    0.0,   0.0,   0.0`
             `0.0,   0.1,    0.0,   0.0,   0.0`
             `0.0,   0.0,    0.5,   0.0,   0.0`
             `0.0,   0.0,    0.0,   0.5,   0.0`
             `0.0,   0.0,    0.0,   0.0,   0.5`

   * Convert radar or lidar data to CTRV (x, y, v, psi, psi_dot)
     
### Prediction:
   * Augment state vector and covarience matrix
   * Predict sigma points, the state, and the state covariance matrix based on `delta_t`
   * lower the process noise to `std_a_ = 6 m/s^2` and 'std_yawdd_ = 6 rad/s^2'

### Update:
   * Useing the sensor type to perform the update step.
   * Updating the state and covariance matrices in `UpdateRadar` or `UpdateLidar` function.
   * Using appropriate measurement noise for each sensor

       
### Performance:
Based on this implementation, I got the following RMSE for data set provided for this project: [0.087, 0.096, 0.40, 0.39]

       
## Discussion
1. My algorithm follows the general processing flow as taught in the lessons and I was able to get the required performance. I encountered 2 issues during the implementation.
  * The first issue was regarding the the time stamp for the first measrement. Since I didn't set the `previous_timestamp_` during the initialization, I had a huge `P_` and RMSE at the end of the second iteration. After correcting the bug, the algorithm worked as expected.
  * I did not converted us to s at the first place and it casused a bug in the predicted values.

2. Also, I tried to run the algorithm by only using radar or laser. Here are the RMSE values in each setting:
  * only Laser: [0.12, 0.11, 0.72, 0.52]
  * only Radar: [0.21, 0.32, 0.52, 0.61]
  * conclusion: Certainly using the both sesnsors will result in a better performance.

3. Also, I compared my the performance of UKF and EKF algorithm here:
  * UKF: [0.08, 0.09, 0.40, 0.39]
  * EKF: [0.11, 0.11, 0.52, 0.52]
  * Conclusion: UKF performs slightly better.

I was able to meet the required performance by using both laser and radar and also lower the process noise to `std_a_ = 6 m/s^2` and 'std_yawdd_ = 6 rad/s^2'




