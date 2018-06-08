# Multi_sensor_localization

Make sure you have updated to the master and catkin_make
# Imu
Please Follow Sparfun IMU tutorial.
Start from section "Getting Started With the Example Firmware" to section "Libraries and Example Firmware"
https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide?_ga=2.240203594.855447152.1519633782-59846934.1519633782

# Imu calibration


# Realsense Camera
1. Please follow the realsense camera repo to install library
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
2. Install Realsense ROS package
  sudo apt-get install ros-kinetic-realsense-camera

# Run Tango apriltag
Step 1. Open tango and setup IP
Step 2. rosrun image_transport republi compressed in:=/tango_sensors/tango_camera_color raw out:=/tango_sensors/tango_camera_color
Step 3. Enter command "rostopic list", then you can see the topic Aprilatg_ros need: /tango_sensors/tango_camera_color
/tango_sensors/tango_camera_color/camera_info

# Run Tango position visualization
Step 1. Step 1. Open tango and setup IP
Step 2. rosrun apriltags_ros tango_position_vis 

 
