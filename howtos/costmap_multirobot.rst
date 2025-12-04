.. _costmap_multirobot:

===================================
Multi-Robot Navigation with EasyNav
===================================

This HowTo demonstrates how to run multiple EasyNav robots simultaneously in a shared simulation.  
Each robot operates independently using its own namespaced set of nodes, topics, and frames.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/BDOEr_L4mq8" frameborder="0" allowfullscreen></iframe>
    </div>

Multi-robot navigation with EasyNav is straightforward, but it **requires discipline** when naming topics and managing TF frames.  
This tutorial explains how to set up multiple robots safely without topic or TF conflicts.

---

Setup
-----

Before starting, make sure you have followed the installation instructions in :doc:`../build_install/index`.  
Then ensure that your workspace includes at least:

- ``EasyNavigation``  
- ``easynav_plugins``  
- ``easynav_playground_kobuki``  
- ``easynav_indoor_testcase``  

and that everything builds correctly:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   colcon build --symlink-install
   source install/setup.bash

---

1. Topic Naming and TF Management (Plugin developers)
-----------------------------------------------------

### Fully-qualified topic names

Each plugin should publish topics under its **fully qualified name**, which includes the node and plugin names.  
This prevents collisions when several robots run the same plugins.

Example (in C++):

.. code-block:: cpp

   path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
     node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path",
     10);

This ensures topics look like:

``/r1/controller_node/simple/path``  
``/r2/controller_node/simple/path``

instead of clashing on ``/path``.

---

### TF topic remapping

Each robot typically has its **own TF tree**.  
To isolate TF data, remap the global TF topics (``/tf`` and ``/tf_static``) to **relative ones**,  
so they are automatically namespaced:

.. code-block:: bash

   -r /tf:=tf -r /tf_static:=tf_static

This yields separate TF topics for each robot:

``/r1/tf`` and ``/r1/tf_static``  
``/r2/tf`` and ``/r2/tf_static``

.. note::

   Do **not** apply this remap if both robots are designed to share the same TF tree (which is rare).

---

### Namespaced frames

If you configure a TF prefix for each system node (e.g., ``tf_prefix: r1``),  
then all frames will include this prefix:

``r1/map``, ``r1/odom``, ``r1/base_link``, etc.

Example configuration snippet:

.. code-block:: yaml

   r1/system_node:
     ros__parameters:
       use_sim_time: true
       use_real_time: true
       position_tolerance: 0.1
       angle_tolerance: 0.05
       tf_prefix: r1

---

2. Launching in Simulation
--------------------------

### Step 1 — Start the simulator with two robots

The multi-robot launch file spawns two Kobuki robots within the same Gazebo world:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_multirobot_kobuki.launch.py do_tf_remapping:=true gui:=false

.. note::

   If your launch file uses a different name, adjust the command accordingly.

---

### Step 2 — Start EasyNav for each robot

Run the EasyNav system for each robot in **separate terminals**.  
Each instance uses the same parameter file but with its own namespace and TF remapping.

**Terminal 1 (robot r1):**

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args \
     --params-file ~/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap_multirobot.params.yaml \
     -r __ns:=/r1 \
     -r /tf:=tf -r /tf_static:=tf_static

**Terminal 2 (robot r2):**

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args \
     --params-file ~/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap_multirobot.params.yaml \
     -r __ns:=/r2 \
     -r /tf:=tf -r /tf_static:=tf_static

---

### Step 3 — Launch RViz for each robot

Open one RViz window per robot namespace to visualize each navigation stack independently:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki rviz_namespaced.launch.py \
     namespace:=r1 use_sim_time:=true \
     rviz_config_file:=~/ros/ros2/easynav_ws/src/easynav_playground_kobuki/rviz/nav2_namespaced_view.rviz

   ros2 launch easynav_playground_kobuki rviz_namespaced.launch.py \
     namespace:=r2 use_sim_time:=true \
     rviz_config_file:=~/ros/ros2/easynav_ws/src/easynav_playground_kobuki/rviz/nav2_namespaced_view.rviz

You can now send **2D Goal Poses** independently in each RViz instance.

---

3. Example Parameters
---------------------

Below is a complete example parameter file for two robots.  
Each section is namespaced (``r1/...``, ``r2/...``), so the same plugins can operate without topic or TF collisions.

.. code-block:: yaml

   r1/controller_node:
     ros__parameters:
       use_sim_time: true
       controller_types: [simple]
       simple:
         rt_freq: 30.0
         plugin: easynav_simple_controller/SimpleController
         max_linear_speed: 1.0
         max_angular_speed: 1.5
         look_ahead_dist: 0.2
         k_rot: 0.7

   r1/localizer_node:
     ros__parameters:
       use_sim_time: true
       localizer_types: [costmap]
       costmap:
         rt_freq: 50.0
         freq: 5.0
         reseed_freq: 1.0
         plugin: easynav_costmap_localizer/AMCLLocalizer
         num_particles: 100
         noise_translation: 0.05
         noise_rotation: 0.1
         noise_translation_to_rotation: 0.1
         initial_pose:
           x: 0.0
           y: 0.0
           yaw: 0.0
           std_dev_xy: 0.1
           std_dev_yaw: 0.01

   r1/maps_manager_node:
     ros__parameters:
       use_sim_time: true
       map_types: [costmap]
       costmap:
         freq: 10.0
         plugin: easynav_costmap_maps_manager/CostmapMapsManager
         package: easynav_indoor_testcase
         map_path_file: maps/home2.yaml
         filters: [obstacles, inflation]
         obstacles:
           plugin: easynav_costmap_maps_manager/ObstacleFilter
         inflation:
           plugin: easynav_costmap_maps_manager/InflationFilter
           inflation_radius: 1.3
           cost_scaling_factor: 3.0

   r1/planner_node:
     ros__parameters:
       use_sim_time: true
       planner_types: [simple]
       simple:
         freq: 0.5
         plugin: easynav_costmap_planner/CostmapPlanner
         cost_factor: 10.0

   r1/sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser1]
       perception_default_frame: odom
       laser1:
         topic: scan_raw
         type: sensor_msgs/msg/LaserScan
         group: points
       camera1:
         topic: rgbd_camera/points
         type: sensor_msgs/msg/PointCloud2
         group: points

   r1/system_node:
     ros__parameters:
       use_sim_time: true
       use_real_time: true
       position_tolerance: 0.1
       angle_tolerance: 0.05
       tf_prefix: r1

   # Robot 2 (r2) - identical configuration with different initial pose and TF prefix

   r2/controller_node:
     ros__parameters:
       use_sim_time: true
       controller_types: [simple]
       simple:
         rt_freq: 30.0
         plugin: easynav_simple_controller/SimpleController
         max_linear_speed: 1.0
         max_angular_speed: 1.5
         look_ahead_dist: 0.2
         k_rot: 0.7

   r2/localizer_node:
     ros__parameters:
       use_sim_time: true
       localizer_types: [costmap]
       costmap:
         rt_freq: 50.0
         freq: 5.0
         reseed_freq: 1.0
         plugin: easynav_costmap_localizer/AMCLLocalizer
         num_particles: 100
         noise_translation: 0.05
         noise_rotation: 0.1
         noise_translation_to_rotation: 0.1
         initial_pose:
           x: 2.0
           y: 1.0
           yaw: 0.0
           std_dev_xy: 0.1
           std_dev_yaw: 0.01

   r2/maps_manager_node:
     ros__parameters:
       use_sim_time: true
       map_types: [costmap]
       costmap:
         freq: 10.0
         plugin: easynav_costmap_maps_manager/CostmapMapsManager
         package: easynav_indoor_testcase
         map_path_file: maps/home2.yaml
         filters: [obstacles, inflation]
         obstacles:
           plugin: easynav_costmap_maps_manager/ObstacleFilter
         inflation:
           plugin: easynav_costmap_maps_manager/InflationFilter
           inflation_radius: 1.3
           cost_scaling_factor: 3.0

   r2/planner_node:
     ros__parameters:
       use_sim_time: true
       planner_types: [simple]
       simple:
         freq: 0.5
         plugin: easynav_costmap_planner/CostmapPlanner
         cost_factor: 10.0

   r2/sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser1]
       perception_default_frame: odom
       laser1:
         topic: scan_raw
         type: sensor_msgs/msg/LaserScan
         group: points
       camera1:
         topic: rgbd_camera/points
         type: sensor_msgs/msg/PointCloud2
         group: points

   r2/system_node:
     ros__parameters:
       use_sim_time: true
       use_real_time: true
       position_tolerance: 0.1
       angle_tolerance: 0.05
       tf_prefix: r2

---

Tips & Gotchas
--------------

- **Namespaces everywhere:**  
  Verify all relative topics (e.g., ``scan_raw``) are correctly resolved under each robot namespace (``/r1/scan_raw``, ``/r2/scan_raw``).

- **Avoid over-remapping:**  
  Only remap ``/tf`` and ``/tf_static`` to relative topics when each robot manages its own TF tree.

- **Frame references:**  
  With ``tf_prefix`` set, refer to frames as ``r1/base_link``, ``r1/odom``, etc.

- **ROS Domain IDs:**  
  If you want isolation or multiple networks, assign different ``ROS_DOMAIN_ID`` per fleet.  
  Otherwise, keep the same domain for shared visualization.

---

With this setup, each robot runs a full EasyNav navigation stack under its own namespace,  
enabling **coordinated multi-robot simulation** in Gazebo and RViz.
