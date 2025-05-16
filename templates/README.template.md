# {{ repo_name }}

![License](https://img.shields.io/github/license/{{ github_user }}/{{ repo_name }})
![Last Commit](https://img.shields.io/github/last-commit/{{ github_user }}/{{ repo_name }})

This is the `{{ repo_name }}` ROS 2 package, part of the [CoHORT](https://github.com/{{ github_user }}/arcs_cohort) multi-rover autonomy software stack.

{% if description %}
## 📝 Description

{{ description }}
{% endif %}

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
{% if 'main' in branches -%}
| `main` | ![Docs](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ docs_workflow_filename }}.svg?branch=main) | ![Tests](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ tests_workflow_filename }}.svg?branch=main) |
{% else %}
{% endif %}
{% if 'jazzy' in branches %}
| `jazzy` | ![Docs](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ docs_workflow_filename }}.svg?branch=jazzy) | ![Tests](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ tests_workflow_filename }}.svg?branch=jazzy) |
{% else %}
{% endif %}
{% if 'humble' in branches %}
| `humble` | ![Docs](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ docs_workflow_filename }}.svg?branch=humble) | ![Tests](https://github.com/{{ github_user }}/{{ repo_name }}/actions/workflows/{{ tests_workflow_filename }}.svg?branch=humble) |
{% else %}
{% endif %}

## 📚 Documentation

For full documentation and launch file reference, visit the [Wiki](https://github.com/{{ github_user }}/{{ repo_name }}/wiki)

{% if launch_docs %}
## 🚀 Launch Files

The following launch files are provided by this package:

{% for file in launch_docs %}
- `{{ file.title }}`: [{{ file.name[:-3] }}](https://github.com/{{ github_user }}/{{ repo_name }}/wiki/{{ file.name[:-3] }})
{% endfor %}
{% else %}
_This package does not contain any launch files._
{% endif %}

{% if maintainers %}
## 👥 Maintainers

{% for maint in maintainers %}
- {{ maint.name }}{% if maint.obfuscated_email %} ({{ maint.obfuscated_email }}){% endif %}
{% endfor %}
{% endif %}

## 🗃️ Repository

- 📁 GitHub: [{{ github_user }}/{{ repo_name }}](https://github.com/{{ github_user }}/{{ repo_name }})
- 📚 Wiki: [Documentation](https://github.com/{{ github_user }}/{{ repo_name }}/wiki)
- 👥 Contributors: [See contributors](https://github.com/{{ github_user }}/{{ repo_name }}/graphs/contributors)

{% if license %}
## 📄 License

This package is licensed under the [**{{ license }}** license](https://github.com/{{ github_user }}/{{ repo_name }}/blob/main/LICENSE).
{% endif %}
