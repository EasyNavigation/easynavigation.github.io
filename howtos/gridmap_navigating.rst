.. _gridmap_outdoor_navigation:

=========================================
Outdoor Navigation with GridMaps (Summit)
=========================================

This HowTo demonstrates how to run **EasyNav** outdoors using **GridMap** representation and the **Summit robot**.  
It uses the *GridMap Maps Manager* for environment representation and the *LidarSLAM Localizer plugin* for real-time localization.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/mxivTYNY1yY" frameborder="0" allowfullscreen></iframe>
    </div>

In this tutorial, you will:

1. Launch the **Summit** outdoor simulator.
2. Start the **EasyNav** system configured for GridMap-based navigation.
3. Use **RViz2** to send navigation goals interactively.

This example combines the **GridMap** framework (for multi-layer terrain representation) and **LidarSLAM** (for localization from point clouds).  
It showcases how EasyNav seamlessly integrates mapping, localization, planning, and control in outdoor environments.

---

Setup
-----

1. You have completed the installation described in :doc:`../build_install/index`.  
2. You have a working workspace containing the following repositories:

   - ``EasyNavigation``
   - ``easynav_plugins``
   - ``easynav_gridmap_stack`` *(for GridMap representation and planner)*
   - ``easynav_lidarslam_ros2`` *(for SLAM)*
   - ``easynav_playground_summit`` *(for the Summit simulation world)*

If something is missing, clone the required repositories:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_plugins.git
   git clone https://github.com/EasyNavigation/easynav_playground_summit.git
   git clone https://github.com/EasyNavigation/easynav_outdoor_testcase.git
   git clone -b rolling https://github.com/EasyNavigation/easynav_lidarslam_ros2.git
   git clone -b rolling_ament_fixed https://github.com/fmrico/grid_map.git

.. warning::

   The official `grid_map` repository still uses ``ament_target_dependencies()``,  
   which is deprecated in recent ROS 2 distributions.  
   Use the patched fork above (branch ``rolling_ament_fixed``) to ensure successful builds.

Then build and source your workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

---

1. Launch the Simulator
-----------------------

Start the **Summit** world simulation.  
This environment includes outdoor terrain suitable for GridMap-based navigation.

.. code-block:: bash

   ros2 launch easynav_playground_summit playground_summit.launch.py

Keep the RViz window open to visualize sensor topics and the simulated environment.

---

2. Launch EasyNav with GridMap and LidarSLAM
--------------------------------------------

Next, launch **EasyNav** with the following parameter file configuration.

Save the following YAML file as  
``~/ros/ros2/easynav_ws/src/easynav_outdoor_testcase/robots_params/gridmap.lidarslam.params.yaml``

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [simple]
        simple:
          rt_freq: 30.0 
          plugin: easynav_simple_controller/SimpleController
          max_linear_speed: 1.0
          max_angular_speed: 1.0
          look_ahead_dist: 0.2
          k_rot: 0.5

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [lidarslam]
        lidarslam:
          plugin: easynav_lidarslam_localizer/LidarSlamLocalizer
          input_cloud: /front_laser/points
          imu: /imu/data

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [gridmap]
        gridmap:
          freq: 10.0 
          plugin: easynav_gridmap_maps_manager/GridmapMapsManager
          package: easynav_outdoor_testcase
          map_path_file: maps/pool.yaml

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [astar]
        astar:
          plugin: easynav_gridmap_astar_planner/GridMapAStarPlanner
          max_allowed_slope_deg: 20.0

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: /front_laser_sensor/points
          type: sensor_msgs/msg/PointCloud2
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

    # LidarSLAM parameters (used internally by the localizer plugin)
    scan_matcher:
      ros__parameters:
        use_sim_time: True
        global_frame_id: "map"
        robot_frame_id: "base_link"
        odom_frame_id: "odom"
        registration_method: "NDT"
        ndt_resolution: 2.0
        ndt_num_threads: 2
        gicp_corr_dist_threshold: 5.0
        trans_for_mapupdate: 1.5
        vg_size_for_input: 0.5
        vg_size_for_map: 0.2
        use_min_max_filter: true
        scan_min_range: 1.0
        scan_max_range: 200.0
        scan_period: 0.2
        map_publish_period: 15.0
        num_targeted_cloud: 20
        set_initial_pose: true
        initial_pose_x: 0.0
        initial_pose_y: 0.0
        initial_pose_z: 0.0
        initial_pose_qx: 0.0
        initial_pose_qy: 0.0
        initial_pose_qz: 0.0
        initial_pose_qw: 1.0
        use_imu: false
        use_odom: false
        debug_flag: false

    graph_based_slam:
      ros__parameters:
        use_sim_time: True
        registration_method: "NDT"
        ndt_resolution: 1.0
        ndt_num_threads: 2
        voxel_leaf_size: 0.2
        loop_detection_period: 3000
        threshold_loop_closure_score: 0.7
        distance_loop_closure: 100.0
        range_of_searching_loop_closure: 20.0
        search_submap_num: 2
        num_adjacent_pose_cnstraints: 5
        use_save_map_in_loop: true
        debug_flag: true

.. note::

   The **LidarSLAM parameters** (``scan_matcher`` and ``graph_based_slam``) are defined **after** the main EasyNav nodes.  
   These settings are used internally by the ``easynav_lidarslam_localizer/LidarSlamLocalizer`` plugin, which encapsulates
   `lidarslam_ros2` for seamless integration into the EasyNav localization framework.

Then, launch EasyNav with:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_outdoor_testcase/robots_params/gridmap.lidarslam.params.yaml

You should now see console logs from the GridMap Maps Manager, the planner, and the LidarSLAM localizer starting up.

---

3. Commanding Navigation Goals
------------------------------

Once EasyNav is running, open **RViz2** (if not already open) and add the **2D Goal Pose** tool.

.. code-block:: bash

   ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

Click anywhere in the map to send navigation goals to the system.  
The robot will compute paths using the **GridMap A\* Planner** and execute them via the **Simple Controller**.

You can observe in RViz:

- The **GridMap layers** published by the Maps Manager.  
- The **path** generated by the A\* planner.  
- The **robot trajectory** updated in real time as the LidarSLAM localizer refines the pose.

---

Notes
-----

- This tutorial uses the **LidarSLAM Localizer plugin**, which wraps `lidarslam_ros2` internally for tighter integration with EasyNav.
- GridMap enables **multi-layer outdoor representation**, supporting elevation, traversability, and slope data.
- The **A\*** planner respects elevation limits using the parameter ``max_allowed_slope_deg``.
- You can edit the YAML file under ``maps/pool.yaml`` to try different outdoor maps or terrains.

---

With this setup, your Summit robot navigates outdoor environments using real-time LidarSLAM localization and GridMap-based path planning.  
It provides a complete example of **EasyNav’s 3D-aware navigation stack** in simulation.
