.. _gridmap_outdoor_navigation:

===========================================================
3D/Uneven-Terrain Navigation with EasyNav: the NavMap Stack
===========================================================

.. warning::

   **This page originally described navigating with a "GridMap" stack** on the Summit robot,
   using an ``easynav_gridmap_stack`` maps manager/planner (``GridmapMapsManager``,
   ``GridMapAStarPlanner``) together with an ``easynav_lidarslam_ros2``/``LidarSlamLocalizer``
   integration and an ``easynav_outdoor_testcase`` configuration package.

   None of these exist in the current codebase:

   - There is no "gridmap"/``grid_map`` concept anywhere in ``EasyNavigation``,
     ``easynav_plugins``, or the playground packages (verified by grepping the whole workspace).
   - There is no ``easynav_lidarslam_ros2`` or ``easynav_outdoor_testcase`` package in this
     workspace.
   - ``easynav_playground_summit`` only launches the Summit Gazebo simulation
     (``playground_summit.launch.py`` includes ``summit_simulator``'s ``robot_gazebo.launch.py``);
     it ships no EasyNav parameter file or localizer/planner configuration for outdoor navigation.

   The closest currently-supported stack for navigating over a 3D/non-flat surface representation
   is the **NavMap** stack: a triangulated mesh maps manager (``easynav_navmap_maps_manager``) with
   a mesh-aware A\* planner (``easynav_navmap_planner``) and a matching localizer
   (``easynav_navmap_localizer``). There is currently **no shipped, tested NavMap configuration for
   the Summit outdoor robot** in this workspace — the example below uses the real, working NavMap
   configuration that *is* shipped, for the indoor Kobuki playground
   (``easynav_indoor_testcase/robots_params/navmap.kobuki.params.yaml``), so every plugin id,
   parameter and topic below can be verified against real source. Adapt it to your own
   robot/environment if you want to use NavMap outdoors.

   If you just need standard 2D navigation today, see :doc:`costmap_navigating` (indoor,
   Kobuki-based, fully verified) instead.

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

1. Launch a simulator.
2. Start **EasyNav** configured for **NavMap**-based navigation (mesh maps manager +
   mesh-aware A\* planner + AMCL-style localizer over the mesh).
3. Use **RViz2** to send navigation goals interactively.

Unlike a plain 2D costmap, NavMap represents the environment as a triangulated 3D surface with
per-cell cost layers (``obstacles``, ``inflated_obstacles``), which is the current mechanism for
representing non-flat terrain in EasyNav.

---

Setup
-----

1. You have completed the installation described in :doc:`../build_install/index`.
2. You have a working workspace containing the following repositories:

   - ``EasyNavigation``
   - ``easynav_plugins`` (provides ``easynav_navmap_maps_manager``, ``easynav_navmap_planner``,
     ``easynav_navmap_localizer``, ``easynav_serest_controller``)
   - ``easynav_indoor_testcase`` *(for the example configuration and maps used below)*
   - ``easynav_playground_kobuki`` *(for the simulated robot used below)*

If something is missing, clone the required repositories:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_plugins.git
   git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git

Then build and source your workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

---

1. Launch the Simulator
-----------------------

Start the **Kobuki** playground simulation.

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

---

2. Launch EasyNav with the NavMap Stack
-----------------------------------------

This is the real, shipped configuration
(``easynav_indoor_testcase/robots_params/navmap.kobuki.params.yaml``): the **SeReST controller**
for trajectory tracking, an **AMCL-style localizer running over the NavMap mesh**
(``easynav_navmap_localizer/AMCLLocalizer``), the **NavMap Maps Manager** (built here from a ROS
YAML occupancy map via ``occmap_path_file``, with obstacle/inflation filters), and the **NavMap
A\* planner** (``easynav_navmap_planner/AStarPlanner``):

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
          slow_radius: 0.60
          slow_min_speed: 0.03
          final_align_k: 2.0
          final_align_wmax: 0.6
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
        localizer_types: [navmap]
        navmap:
          rt_freq: 50.0
          freq: 5.0
          reseed_freq: 1.0
          plugin: easynav_navmap_localizer/AMCLLocalizer
          num_particles: 100
          noise_translation: 0.05
          noise_rotation: 0.1
          noise_translation_to_rotation: 0.1
          initial_pose:
            x: 0.0
            y: 0.1
            yaw: 0.0
            std_dev_xy: 0.1
            std_dev_yaw: 0.01

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [navmap]
        navmap:
          freq: 10.0
          plugin: easynav_navmap_maps_manager/NavMapMapsManager
          package: easynav_indoor_testcase
          occmap_path_file: maps/home2.yaml
          filters: [obstacles, inflation]
          obstacles:
            plugin: easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter
          inflation:
            plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
            inflation_radius: 1.3
            cost_scaling_factor: 3.0

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_navmap_planner/AStarPlanner
          cost_factor: 10.0
          continuous_replan: true

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan

    system_node:
      ros__parameters:
        use_sim_time: true
        use_real_time: false
        position_tolerance: 0.3
        angle_tolerance: 0.15

.. note::

   The NavMap A\* planner (``easynav_navmap_planner/AStarPlanner``) only exposes ``cost_factor``
   and ``continuous_replan`` as runtime parameters; it evaluates edge cost using distance combined
   with the ``inflated_obstacles`` layer (falling back to ``obstacles``) of the NavMap — there is no
   ``max_allowed_slope_deg``-style parameter on this planner. Slope/height gating instead happens
   when a NavMap mesh is *built* from a point cloud, via the internal (currently hard-coded)
   ``max_slope_deg``/``max_dz`` fields of ``navmap_ros::BuildParams`` — see
   :doc:`bonxai_navmap_from_rosbag`.

Launch EasyNav with:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/navmap.kobuki.params.yaml

You should see console logs from the NavMap Maps Manager, the planner, and the AMCL-style
localizer starting up.

---

3. Commanding Navigation Goals
------------------------------

Open **RViz2** and add the **2D Goal Pose** tool.

.. code-block:: bash

   ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

Click anywhere in the map to send navigation goals to the system. The robot will compute paths
using the **NavMap A\* Planner** and execute them via the **SeReST Controller**.

You can observe in RViz:

- The **NavMap** mesh and its cost layers published by the Maps Manager
  (``/maps_manager_node/navmap/map``, type ``navmap_ros_interfaces/msg/NavMap``).
- The **path** generated by the A\* planner.
- The **robot trajectory** updated in real time as the localizer refines the pose.

---

Notes
-----

- This tutorial uses the real, shipped NavMap + Kobuki configuration to demonstrate the mesh-based
  stack end-to-end. There is currently no equivalent turnkey configuration for the Summit outdoor
  robot in this workspace — if you want to reproduce this outdoors, you will need to build your own
  NavMap (see :doc:`gridmap_mapping` / :doc:`bonxai_navmap_from_rosbag`) and adapt the parameter
  file above (sensors, frames, package/map paths) to your robot.
- NavMap enables per-layer cost data (``obstacles``, ``inflated_obstacles``) on top of a
  triangulated 3D surface, which is the current mechanism for representing non-flat/outdoor
  terrain — there is no multi-layer ``grid_map``-style elevation/traversability representation in
  the current codebase.
- For standard flat-ground 2D navigation, prefer the **Costmap Stack** (:doc:`costmap_navigating`),
  which is simpler and fully supported.

---

With this setup, your robot navigates using NavMap mesh-based path planning — the current
EasyNav mechanism for representing and planning over non-flat surfaces.
