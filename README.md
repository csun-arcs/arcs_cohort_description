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

The launch file, located in the `launch/` folder (assumed to be `view_model.launch.py`), is used to load the robot description and start necessary ROS2 nodes. It includes several launch arguments to configure the launch process. Below are examples of possible arguments (adjust based on your specific implementation):

- **`model_file`**: A path to manually specify the path of the robot model.
  - **Default**: `arcs_cohort_description`
  - **Example**: `model_file:=/home/your_workspace/src/arcs_cohort_description/robot.urdf.xacro`

- **`robot_name`**: Specifies the name of the robot, useful for multi-robot scenarios or namespaces.
  - **Default**: `''`
  - **Example**: `robot_name:=my_robot`

- **`namespace`**: Namespace under which to bring up nodes, topics, etc.
  - **Default**: `''`
  - **Example**: `namespace:=my_namespace`
 
- **`use_sim_time`**: A boolean to determine whether to use simulation time (e.g., from a simulator like Gazebo) or real-time.
  - **Default**: `true`
  - **Example**: `use_sim_time:=false`

- **`use_jsp`**: Specifies if the joint state publisher should be launched or not. It will be set to true when only using rviz.
  - **Default**: `true`
  - **Example**: `use_jsp:=false`

- **`use_jsp_gui`**: Specifies if the joint state publisher should be launched with the GUI or not. 
  - **Default**: `false`
  - **Example**: `use_jsp_gui:=true`
 
- - **`use_rviz`**: Launch rviz when set to true.
  - **Default**: `true`
  - **Example**: `use_rviz:=false`

- **`use_rviz_config_template`**: If true, generate the RViz config from the specified RViz config template. 
  - **Default**: `true`
  - **Example**: `use_rviz_config_template:=false`
 
- - **`rviz_config_template`**: Specifies path to the RViz config template file.
  - **Default**: `default_rviz_config_template_file`
  - **Example**: `rviz_config_template:=/home/your_workspace/src/arcs_cohort_description/rviz_config/your_config_template`

- **`rviz_config`**: Specifies path to the RViz config file. 
  - **Default**: `default_rviz_config_file`
  - **Example**: `rviz_config:=/home/your_workspace/src/arcs_cohort_description/rviz_config/your_config_file`
 
- - **`use_lidar`**: Specifies if we launch the model with the LiDAR or not.
  - **Default**: `false`
  - **Example**: `use_lidar:=false`

- **`lidar_update_rate`**: Specifies the update rate of the LiDAR scan. 
  - **Default**: `30`
  - **Example**: `lidar_update_rate:=10`


To set these arguments, use the following syntax when running the launch file:

```bash
ros2 launch arcs_cohort_description view_model.launch.py arg:=value
