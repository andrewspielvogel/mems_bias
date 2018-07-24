Rosnode for MEMS Bias Estimation. Tested on Ubuntu 16.04

## Installation:

- Install ROS. Instructions [here](http://wiki.ros.org/kinetic/Installation).
- Setup ROS workspace. Instructions [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
- Install eigen3
```
sudo apt-get install libeigen3-dev
```
- Clone node into src folder of ROS workspace.
```
git clone https://github.com/andrewspielvogel/mems_bias.git
```
- Make you workspace. `cd` into the top level of your ROS workspace and run:
```
catkin_make
```
- Create a location for the log file storage or to use default make a KVH log directory in `/log` and give proper permissions
```
sudo mkdir /log   
sudo chown USER.USER /log
chmod +rwx /log           
mkdir /log/kvh
mkdir /log/microstrain
```

## Launch File Params

ang_bias : Initial estimate of the angular rate bias (rad/s). Default: [0,0,0]"

acc_bias : Initial estimate of the acceleration bias (m/s^2). Default: [0,0,0]"

mag_bias : Initial estimate of the magnetometer bias. Default: [0,0,0]"

k_acc: Diagonal of the acceleration gain matrix of the bias estimator.

k_mag: Diagonal of the magnetometer gain matrix of the bias estimator.

k_acc_bias: Diagonal of the acceleration bias gain matrix of the bias estimator.

k_ang_bias: Diagonal of the angular rate bias gain matrix of the bias estimator.

k_ang_mag: Diagonal of the magnetometer bias gain matrix of the bias estimator.

frame_id: Frame ID of the IMU Node subscribing to.


## Using the Node

- Start Node via launch files:
```
roslaunch mems_bias mems_bias.launch
```
launches the bias estimator.

## Topics
Node subscribed to:
- IMU Data topic (dscl_msgs::Imu9DOF)
```
/frame_id/imu topic
```

Topics published:
- IMU bias topic (dscl_msgs::ImuBias)
```
/frame_id_bias/imu_bias
```
- Bias corrected IMU packet (dscl_msgs::Imu9DOF)
```
/frame_id_bias/imu_corrected
```


## Generate Documentation

To generate documentation, use doxygen.

- Install doxygen from [here](http://www.stack.nl/~dimitri/doxygen/download.html). Note: you need to install the packages flex and bison before making doxygen with:
```
sudo apt install flex
sudo apt install bison
```

- Then `cd` into the truenorth directory and run:
```
doxygen Doxyfile
```

- To view the documentation, open `index.html` located in the `html/` directory.
