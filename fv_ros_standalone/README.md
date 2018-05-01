# fv_ros_standalone Folder

This folder contains a modified version of the programs in /standalone as well as a few extra scripts. The additions to the standalone program were applying fisheye lens correction to incoming frames, publishing ROS measurements of blob displacement data, and creating a ros service for storing images for generating calibration test sets. Also, calibration scripts were added

# Summary of Folders and files

*calib/* - contains json files of the calibration test sets

*msg/* - definition of ros messages

*scatter_plots/* - file where calibration scripts store scatter plots

*src/* - source files

*srv/* - definition of ros services

*CMakeLists.txt* - Cmake file for catkin_make

*package.xml* - ros package definitions file

