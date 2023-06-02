# *Volatiles Investigating Polar Exploration Rover* (VIPER): Path, Trajectory, Motion planning and Localization  

## Project Description
In this repository are addressed the Path planning and localization tasks for the VIPER moon rover.  
This is the final exercise for the course of Space Robotics System of held in La Sapienza University of Rome, by Professor Antonio Genova.  
The tasks that were requested to complete the exercises are:  
    1. Trajectory control of the Rover, passing through a set of via points, avoiding the Permanently Shadowed Regions (PSRs);
    2. Path Planning with the A* algorithm from a start to a goal pose, also having access to an elevation map of the moon's surface;
    3. According to the trajectory computed in Task 1, the states must be reconstructed comparing Dead Reckoning localization with Extended Kalman Filter (EKF) approach.  
The project was carried out through MATLAB implementation. A more technical description of the project's objectives is available at 'project_objectives.pdf' and the discussion on the result is available at 'report.pdf'.  

![image](http://drive.google.com/uc?export=view&id=1F7fqyJTW6OzYF4QJuWCNxlJpSBloW9he)  
![image](http://drive.google.com/uc?export=view&id=1FMlh_H7Vx6WzMw9WDfs8HZRYpRPBZlCH)
## How to use this repository 
Before initializing the 'main.m' file, be sure to have the 'exercise.mat' file installed. It can be installed at this [link](https://drive.google.com/file/d/1pc2K26E_A2ZkY_A6XljhpC_EZEfGdfmr/view?usp=drive_link). This file, when loaded make possible to access at the occupancy grid of the moon's surface.  
To run the experiments, access to the main.m script, and run it. But first select in 'src/init.m' the variables of interest. To select whether doing the first, second or the third task, you need to change the variable 'task: int' in main.m.  
## Repository Description
* images/  
    * task1_images/ : Plots of the results obtained in the first task  
        * heading.jpg: Heading angle evolution for each via point;
        * rate_heading.jpg: Heading angle rate evolution for each via point;
        * rover_confs.jpg: Rover configurations evolution for each via point;
        * velocity.jpg : Rover's longitudinal velocity;
        * trajectory.jpg : Rover's path on the moon's surface;
        * via_points_n/via_points.jpg : Visualization of the World frame, with respect to the image frame;
        * via_points.jpg : Via points positions.
    
    * task2_images/ : Plots of the results obtained in the second task  
        * a_star_no_steep_expansion.jpg : Expansion of the A* algorithm, when the DEM is not considered in the cost function;
        * a_star_no_steep1/2.jpg : Path generated with A* when DEM is not considered in the cost function  
        * expanded_steep.jpg : visited nodes with A* algorithm, when considering the DEM, no path was found.     

    * task3_images/ : Plots of the results obtained in the third task  
        * kalman_det.jpg : determinant of the covariance matrix, when the EKF engine is enabled.  
        * landmarks.jpg : Landmarks visualization  
        * new_kalman_zoom.jpg : Visualization of the small error commited in the state reconstruction with the kalman filter.  
        * new_kalman.jpg : Kalman filter localization, trajectory reconstruction.  
        * new_odometry.jpg : State reconstruction with dead reckoning.  
        * odom_det.jpg : determinant of the covariance matrix, with dead reckoning reconstruction.

    
    * elevation_map.jpg  
    * map.jpg  
    * obstacle_map.jpg  
    * rover_model.png  
* src/ : Source code of MATLAB.
* task1_vector/ : Stored data for the first task. These values are stored for the localizaiton experiment of the third task.  
    * 10hz/ : Contains the data for the 10hz trajectory tracking  
        * P0_P1_confs.dat : Configurations from P0 to P1 viapoint
        * P0_P1_numbers.dat : Number of states traversed from P0 to P1 viapoint
        * P0_P1_theta.dat : Heading angles from P0 to P1 viapoint
        * P0_P1_thetad.dat : Heading angles rate from P0 to P1 viapoint
        * P0_P1_time.dat : Timesteps from P0 to P1 viapoint
        * P0_P1_vels.dat : Longitudinal Velocities from P0 to P1 viapoint 
    * P0_P1_confs.dat : Configurations from P0 to P1 viapoint
    * P0_P1_numbers.dat : Number of states traversed from P0 to P1 viapoint
    * P0_P1_theta.dat : Heading angles from P0 to P1 viapoint
    * P0_P1_thetad.dat : Heading angles rate from P0 to P1 viapoint
    * P0_P1_time.dat : Timesteps from P0 to P1 viapoint
    * P0_P1_vels.dat : Longitudinal Velocities from P0 to P1 viapoint  
* exercise.mat : File that contains the information about the maps. (Download it as described before)   
* main.m : Main file to run the project  
* project_objectives.pdf : Detailed descrpition of the project's requirements.  
* report.pdf : Detailed description of the results and the achieved performances.


