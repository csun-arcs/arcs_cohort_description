# arcs_cohort_description

![License](https://img.shields.io/github/license/csun-arcs/arcs_cohort_description)
![Last Commit](https://img.shields.io/github/last-commit/csun-arcs/arcs_cohort_description)

This is the `arcs_cohort_description` ROS 2 package, part of the [CoHORT](https://github.com/csun-arcs/arcs_cohort) multi-rover autonomy software stack.


## 📝 Description

This package provides the description of the CSUN ARCS CoHORT rover, including its URDF model and sensor configurations, for use in ROS 2. It enables users to simulate and visualize the robot in RViz and Gazebo.


## Folder Structure

The package is organized into the following folders, each serving a specific purpose:

- **`description/`**: Contains the main [URDF (Unified Robot Description Format)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) [Xacro](https://docs.ros.org/en/ros2_packages/rolling/api/xacro/) file for the robot, along with separate URDF Xacro files for various sensors, including:
  - ZED Mini camera
  - IMU (Inertial Measurement Unit)
  - LiDAR
  These files define the robot's physical structure and sensor integration for use in ROS2.

- **`launch/`**: Includes the launch file (e.g., `view_model.launch.py`) to bring up the robot description and related nodes. The launch file also supports several launch arguments for customization (see details below).

- **`meshes/`**: Holds the STL mesh files used in the URDF. These files define the visual and collision geometry of the robot's components, referenced by the URDF for simulation and visualization.

## 📦 Build Status

| Branch | Docs | Tests |
|--------|------|-------|
| `main` | ![Docs](https://github.com/csun-arcs/arcs_cohort_description/actions/workflows/generate-docs.yml.svg?branch=main) | ![Tests](https://github.com/csun-arcs/arcs_cohort_description/actions/workflows/run-tests.yml.svg?branch=main) |




| `humble` | ![Docs](https://github.com/csun-arcs/arcs_cohort_description/actions/workflows/generate-docs.yml.svg?branch=humble) | ![Tests](https://github.com/csun-arcs/arcs_cohort_description/actions/workflows/run-tests.yml.svg?branch=humble) |


## 📚 Documentation

For full documentation and launch file reference, visit the [Wiki](https://github.com/csun-arcs/arcs_cohort_description/wiki)


## 🚀 Launch Files

The following launch files are provided by this package:


- `View Model`: [view_model.launch](https://github.com/csun-arcs/arcs_cohort_description/wiki/view_model.launch)




## 👥 Maintainers


- Barry Ridge (barry [dot] ridge [at] csun [dot] edu)

- Subhobrata Chakraborty (subhobrata [dot] chakraborty [at] csun [dot] edu)



## 🗃️ Repository

- 📁 GitHub: [csun-arcs/arcs_cohort_description](https://github.com/csun-arcs/arcs_cohort_description)
- 📚 Wiki: [Documentation](https://github.com/csun-arcs/arcs_cohort_description/wiki)
- 👥 Contributors: [See contributors](https://github.com/csun-arcs/arcs_cohort_description/graphs/contributors)


## 📄 License

This package is licensed under the [**Apache-2.0** license](https://github.com/csun-arcs/arcs_cohort_description/blob/main/LICENSE).
