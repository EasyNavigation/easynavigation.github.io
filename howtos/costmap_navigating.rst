.. _costmap_navigating:

=====================================
Mapping with SLAM Toolbox and EasyNav
=====================================

This HowTo shows how to perform **mapping and navigation** with **SLAM Toolbox** and **EasyNavigation (EasyNav)**.  
It assumes that the environment has already been mapped and that you will now configure and run EasyNav to navigate
using the generated map.

.. contents:: On this page
   :local:
   :depth: 2

Setup
------

Before starting, ensure that:

1. You have completed the installation steps in :doc:`../build_install/index`.  
2. You have cloned and built the following repositories in your workspace:

   - ``EasyNavigation``
   - ``easynav_plugins``
   - ``easynav_playground_kobuki`` *(for simulation)*
   - ``easynav_indoor_testcase`` *(for maps and parameter examples)*

3. Your workspace is sourced:

   .. code-block:: bash

      cd ~/ros/ros2/easynav_ws
      source install/setup.bash

---

Mapping and Preparation
-----------------------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/n1vDA4ZeG6M" frameborder="0" allowfullscreen></iframe>
    </div>

If you have not yet created a map, follow the steps in :doc:`costmap_mapping`.  
Once the environment is mapped, save the resulting map files (YAML + image) in the directory of any package
in your workspace — for example, inside ``easynav_indoor_testcase/maps``.

You can later reference this map using the parameters ``package`` and ``map_path_file`` in your configuration file.
(Alternatively, you may use an absolute path with ``map_path_file`` alone.)

---

Creating a Parameter File
-------------------------

In this example, we will use:

- The **SeReST controller** for motion control.  
- The **AMCL localizer** for probabilistic localization.  
- The **Simple Maps Manager** to load the map representation.

Below is a minimal configuration that allows a simulated robot to navigate in a mapped environment.

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [serest]
        serest:
          rt_freq: 30.0
          plugin: easynav_serest_controller/SerestController
          allow_reverse: true
          max_linear_speed: 0.8
          max_angular_speed: 1.2
          v_progress_min: 0.08
          k_s_share_max: 0.5
          k_theta: 2.5
          k_y: 1.5
          goal_pos_tol: 0.1
          goal_yaw_tol_deg: 6.0
          slow_radius: 0.80
          slow_min_speed: 0.02
          final_align_k: 2.5
          final_align_wmax: 0.8
          corner_guard_enable: true
          corner_gain_ey: 1.8
          corner_gain_eth: 0.7
          corner_gain_kappa: 0.4
          corner_min_alpha: 0.35
          corner_boost_omega: 1.0
          a_lat_soft: 0.9
          apex_ey_des: 0.05

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [simple]
        simple:
          rt_freq: 50.0
          freq: 5.0
          reseed_freq: 1.0
          plugin: easynav_simple_localizer/AMCLLocalizer
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

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [simple]
        simple:
          freq: 10.0
          plugin: easynav_simple_maps_manager/SimpleMapsManager
          package: easynav_indoor_testcase
          map_path_file: maps/home.yaml

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_simple_planner/SimplePlanner
          robot_radius: 0.25

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.3
        angle_tolerance: 0.15

---

Running the Simulation
----------------------

1. **Launch the simulator.**  
   You can disable the Gazebo GUI to save resources:

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

2. **Launch RViz2** in a new terminal:

   .. code-block:: bash

      ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

3. **Start EasyNav** using your parameter file:  

   .. code-block:: bash

      ros2 run easynav_system system_main \
         --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/simple.serest_params.yaml

   *(You can also create a dedicated launcher file for convenience.)*

4. In **RViz2**, use the **“2D Goal Pose”** tool to send navigation goals.  
   The robot should begin navigating autonomously along collision-free paths.

---

Notes
-----

- Ensure that the ``map_path_file`` path and package name correspond to your actual map.  
- The *Simple Stack* is suitable for 2D navigation with binary occupancy grids;  
  for graded cost-based navigation, consider switching to the *Costmap Stack* (:doc:`costmap_mapping`).  
- If navigation oscillates or stalls, verify that the controller gains (``k_theta``, ``k_y``) and speed limits
  are consistent with your robot’s maximum velocities.
