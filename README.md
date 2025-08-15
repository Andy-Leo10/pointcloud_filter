# PointCloud Filter - ROS2 Package

## Overview
`pointcloud_filter` is a ROS2 package designed to process and filter 3D point clouds from a LiDAR sensor. It removes unwanted rings (caused by secondary LiDAR reflections) and filters out points belonging to the robot itself. The package also projects the 3D point cloud into a 2D `LaserScan` message for use in SLAM and mapping.

## Features
- **Removes unwanted LiDAR rings** (caused by secondary pulse returns).
- **Filters out points belonging to the robot's own structure**.
- **Converts a 3D point cloud into a 2D `LaserScan`** for navigation and SLAM.
- **Optimized using C++ & PCL** for real-time performance.
- **Easily configure via rqt** or command line parameters.
- **Output frame transform** if needed.
## Limitations
- **Output frame transform** requires that axis are aligned (no rotations between frames)

---

## Installation & Build

### **1️⃣ Clone the Repository**
Navigate to your ROS2 workspace and clone the package.

### **2️⃣ Build the Package**
Build the package using `colcon` and source the workspace.

---

## Usage

### **Launch the PointCloud Filter Node**
Use run or launch to start the node.
```
ros2 run pointcloud_filter pointcloud_filter 
ros2 run pointcloud_filter pointcloud_filter --ros-args -p laser_output_frame:="base_footprint"
```
```
ros2 launch pointcloud_filter filterPCL_and_convertLS.launch.py
```
```
ros2 launch pointcloud_filter yaml_cfg_filter.launch.py
```

### **Subscribed Topics**
- `/ouster/points` (`sensor_msgs/PointCloud2`) - Raw LiDAR point cloud input.

### **Published Topics**
- `/ouster/points_filtered` (`sensor_msgs/PointCloud2`) - The filtered 3D point cloud.
- `/scan` (`sensor_msgs/LaserScan`) - The projected 2D laser scan.

---

## Dependencies
This package requires the following dependencies:

- **ROS2 (Humble or later)**
- **PCL (Point Cloud Library)**
- **pcl_conversions**
- **sensor_msgs**
- **rclcpp**

Ensure they are installed before building the package.

---

## Contribution
Contributions are welcome! Follow these steps:

1. **Fork the repository** on GitHub.
2. Clone your fork.
3. Create a new branch.
4. Make your changes and commit.
5. Push and create a **Pull Request**.

---

## License
This project is licensed under the **Apache-2.0 License**.

---

## Contact
For issues, feature requests, or questions, open a GitHub **Issue** or contact:

📩 Email: [mahdichalaki@ualberta.ca](mailto:mahdichalaki@ualberta.ca)  
📌 GitHub: [mahdichalaki](https://github.com/mahdichalaki)
