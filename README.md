#FD-SLAM
This is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR.

It is based on hdl_graph_slam and the steps to run our system are same with hdl-graph-slam.

We also release [UGICP](https://github.com/SLAMWang/UGICP).

Our paper is under review and will be released soon.
FD-SLAM: Feature&Distribution-based 3D LiDAR SLAM method based on Surface Representation Refinement


## hdl_graph_slam

[hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection. It also supports several graph constraints, such as GPS, IMU acceleration (gravity vector), IMU orientation (magnetic sensor), and floor plane (detected in a point cloud). We have tested this package with Velodyne (HDL32e, VLP16) and RoboSense (16 channels) sensors in indoor and outdoor environments.

## The names and functions do not change compared with hdl-graph-slam.

We use a novel feature-based Lidar odometry for fast scan-matching, and use a proposed UGICP for keyframe matching. The backend in hdl-graph-slam is reused.  We have tested this package with Velodyne (HDL32e, HDL64,VLP16) and Ouster64 sensors in indoor and outdoor environments. The corresponding configure launch files are provided.

## key modifications
src/hdl_graph_slam/imageProjection.cpp and src/hdl_graph_slam/featureAssociation.cpp are added for fast scan matching

As for the fast-gicp, key modifications are taken place in fast_gicp_impl.cpp----calculate_covariances()


## Nodelets
***FD-slam*** consists of four nodelets.

- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *hdl_graph_slam_nodelet*

The input point cloud is first downsampled by *prefiltering_nodelet*, and then passed to the next nodelets. While *scan_matching_odometry_nodelet* estimates the sensor pose by iteratively applying a scan matching between consecutive frames (i.e., odometry estimation), *floor_detection_nodelet* detects floor planes by RANSAC. The estimated odometry and the detected floor planes are sent to *hdl_graph_slam*. To compensate the accumulated error of the scan matching, it performs loop detection and optimizes a pose graph which takes various constraints into account.



## Parameters
All the configurable parameters are listed in *launch/****.launch* as ros params.

## Services
- */hdl_graph_slam/dump*  (hdl_graph_slam/DumpGraph)
  - save all the internal data (point clouds, floor coeffs, odoms, and pose graph) to a directory.
- */hdl_graph_slam/save_map*  (hdl_graph_slam/SaveMap)
  - save the generated map as a PCD file.

## Requirements
***hdl_graph_slam*** requires the following libraries:

- OpenMP
- PCL
- g2o
- suitesparse

The following ROS packages are required:

- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [U_gicp](https://github.com/SLAMWang/UGICP) This is modified based on [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) by us. We use UGICP for keyframe matching.


