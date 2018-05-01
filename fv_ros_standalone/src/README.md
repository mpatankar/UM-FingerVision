# fv_ros_standalone/src Folder

# Summary files

*gentests.py* - python script that reads the test_config.json file, generates a json test set for calibration, and generates ros service requests to the blob tracker to save calibration images. 

*optimize.py* - python script that reads in previously generated test sets and produces coefficients relating displacements to forces

*vs_test_force.py* - python script that reads blob movement ros messages and prints out the expected force on the sensor

*simple_blob_tracker4.cpp* - program that performs the lens rectification of incoming frames, dot tracking, saving of calibration images (during the calibration procedure)
