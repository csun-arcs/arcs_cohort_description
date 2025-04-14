# ARCS Cohort Description Package

This package provides the description of the ARCS cohort robot, including its URDF model and sensor configurations, for use in ROS2 Jazzy. It enables users to simulate and visualize the robot in RViz.
## Folder Structure

The package is organized into the following folders, each serving a specific purpose:

- **`description/`**: Contains the main URDF (Unified Robot Description Format) file of the robot, along with separate URDF files for various sensors, including:
  - ZED Mini camera
  - IMU (Inertial Measurement Unit)
  - LiDAR
  These files define the robot's physical structure and sensor integration for use in ROS2.

- **`launch/`**: Includes the launch file (e.g., `robot_description.launch.py`) to bring up the robot description and related nodes. The launch file also supports several launch arguments for customization (see details below).

- **`meshes/`**: Holds the STL (Stereolithography) mesh files used in the URDF. These files define the visual and collision geometry of the robot's components, referenced by the URDF for simulation and visualization.

- **`rviz_config/`**: Contains the RViz configuration file (e.g., `robot.rviz`) for visualizing the robot and its sensors in RViz, a 3D visualization tool for ROS.

## Launch File and Arguments

The launch file, located in the `launch/` folder (assumed to be `robot_description.launch.py`), is used to load the robot description and start necessary ROS2 nodes. It includes several launch arguments to configure the launch process. Below are examples of possible arguments (adjust based on your specific implementation):

- **`use_sim_time`**: A boolean to determine whether to use simulation time (e.g., from a simulator like Gazebo) or real-time.
  - **Default**: `false`
  - **Example**: `use_sim_time:=true`

- **`robot_name`**: Specifies the name of the robot, useful for multi-robot scenarios or namespaces.
  - **Default**: `'arcs_robot'`
  - **Example**: `robot_name:=my_robot`

- **`sensors`**: A list of sensors to include in the robot description, allowing users to enable or disable specific sensors.
  - **Default**: `['zed_mini', 'imu', 'lidar']`
  - **Example**: `sensors:='["zed_mini"]'`

To set these arguments, use the following syntax when running the launch file:

```bash
ros2 launch arcs_cohort_description view_model.launch.py arg:=value
