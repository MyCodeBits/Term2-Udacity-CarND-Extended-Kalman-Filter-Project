[image1]: ./outputs/1.png "1"
[image2]: ./outputs/2.png "2"
[image3]: ./outputs/3.png "3"
[image4]: ./outputs/4.png "4"

# Extended Kalman Filter Project.
Self-Driving Car Engineer Nanodegree Program

The README for project starter code int order to do the code setup is at this [Udacity link.](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

##Brief on Project.
In this project, we will utilize a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. It is expected to keep the RMSE (error) values, **[px, py, vx, and vy]** less than or equal to the values **[.11, .11, 0.52, 0.52]**.

This project uses the Term 2 Simulator which can be downloaded [here.](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

Refer [Udacity link's](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) **Other Important Dependencies** section.

## Basic Build Instructions

After having the dependencies above met :

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) has been used.

## Project Implementation details.

Starting with Udacity's ["Extended kalman filter" starter code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project), following code implementations were added which can be seen as diff in [this commit.](https://github.com/MyCodeBits/Term2-Udacity-CarND-Extended-Kalman-Filter-Project/commit/8bfd1c58762e7d8823c475ccc7feb80668643840?diff=split)

- **src/main.cpp :** Aleady coded as part of started code. Reads in the sensor data line by line from the client and stores the data into a measurement object that it passes to the Kalman filter for processing. Also a ground truth list and an estimation list are used for tracking RMSE.  
- **src/FusionEKF.cpp :** Added code for following:
  1. initialized variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
  2. initialized the Kalman filter position vector with the first sensor measurements
  3. modified the F and Q matrices prior to the prediction step based on the elapsed time between measurements
  4. called update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.
- **kalman_filter.cpp :** added the code to to implement the **prediction** and **update** equations.
- **tools.cpp :** Implemented functions to calculate **root mean squared error** and the **Jacobian matrix**.
- **outputs :** Output images and output text file captured from the run using simulator.


**Outputs**

*Images*

![alt text][image1]

![alt text][image2]

![alt text][image3]

![alt text][image4]

The outputs for estimation is in the [output.txt](https://github.com/MyCodeBits/Term2-Udacity-CarND-Extended-Kalman-Filter-Project/blob/master/outputs/output.txt) file.
