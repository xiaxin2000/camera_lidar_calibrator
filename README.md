# camera_lidar_calibrator

## Modification
* Accept more points from images and lidar frames for calibrating the camera and lidar parameters. 
	-To mitigate the influence from the errors caused by manully selecting points, we need to add more points to the pnp solver, which is used to solve the optimization problem to get the extrinsic parameters between camera and lidar. The original package from default autoware version only accepts 9 points, which are not enough to get accurate extrinsic parameters.
	
* We also added the code to save the used coordinates of the points from images and lidar frames as well as the calibration results into csv files for further analysis.

* We also added a function to call another solver (Ransac based pnp solver) to calculate the extrinsic parameters, which can be used for comparing purpose.

## Notes 
* The baseline is from Autoware repo [https://autoware.readthedocs.io/en/feature-documentation_rtd/DevelopersGuide/PackagesAPI/sensing/autoware_camera_lidar_calibrator.html] and [https://github.com/Autoware-AI/utilities/tree/master/autoware_camera_lidar_calibrator].
* Use the same command to launch the rosnode: roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/home/carma/autoware.ai/int_calibration_tesla_20211111.yaml image_src:=/image