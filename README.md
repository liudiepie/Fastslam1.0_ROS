# Fastslam1.0_ROS
This repository is based on [EKFSLAM_FastSLAM](https://github.com/ogzkhrmn/EKFSLAM_FastSLAM) and Turtlebot3  
It is required to install these two packages to obtain the full process  
The purpose of making this project is to understand the basic knowledge of SLAM  
This is the performance video [LINK](https://youtu.be/M4Wkvzu3IXc)
### Installation
1. Install Turtlebot3  
   There are lots of resource teaching people how to install Turtlebot3.    
   Here is one of the example, [LINK](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/)  
   Based on the Ubuntu version to choose the right version.
2. Install EKFSLAM_FastSLAM  
   Go to the website above,   
   ```bash
   git clone <HTTPS or SSH>
   ```
   It is suggested to clone this package under the turtlebot3_slam/src file  
   Make sure both packages install properly
3. Install this repository  
   Clone the package also under the turtlebot3_slam/src file  
   It would be easier to modify
### Implementation
1. Replace the file  
   Copy Main.py and sensor_data.dat in this repocitory and replace the same name file under EKFSLAM_FastSLAM
2. Launch Turtlebot3  
   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```
3. Move the Turtlebot  
   Open a terminal under this repository and run
   ```bash 
   python move.py
   ```
   The turtlebot should wait for a few second and move to the certain position  
   Certain position can be easier change through goal in move.py  
4. Checking the collected data  
   After Running the move.py, odom_data.dat which includes sensor data would occur.
   Plugging the numbers into the sensor_data.dat  
   The data would be ready to perform fastslam  
5. Performance  
   Open a terminal under EKFSLAM_FastSLAM file and run  
   ```bash
   python Main.py
   ```
   The result would occur in few seconds.
### Environment  
   Ubuntu 20.04 noetic  
   Python 3.6
   
