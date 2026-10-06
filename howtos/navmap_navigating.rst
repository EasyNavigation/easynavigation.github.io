.. _navmap_navigating:

=========================================
Navigating with the NavMap + Bonxai Stack
=========================================

This HowTo shows how to navigate with **NavMap** and **Bonxai** maps in EasyNavigation (EasyNav),
using the :doc:`Summit PlayGround <../playgrounds/summit>`:

- `NavMap <https://github.com/EasyNavigation/NavMap>`_ represents the navigable surfaces as a
  triangle mesh, with layers (occupancy, inflation...) on its cells. It works on uneven 3D terrain
  and, built from a 2D occupancy grid, on flat floors. The planner plans over it.
- `Bonxai <https://github.com/facontidavide/Bonxai>`_ is a probabilistic 3D voxel map. The NavMap
  AMCL localizer scores the 3D point clouds of the robot against it.

.. contents:: On this page
   :local:
   :depth: 2

Setup
-----

Build EasyNav from source (see :ref:`build_from_source`, which also clones NavMap) and the
Summit PlayGround (see :ref:`playgrounds`).

The two scenarios
-----------------

**Outdoors**, in the URJC excavation, an uneven 3D terrain:

.. code-block:: bash

   ros2 launch easynav_playground_summit easynav_bonxai_amcl.launch.yaml

**Indoors**, in a small logistics warehouse, with a flat NavMap built from a 2D occupancy grid:

.. code-block:: bash

   ros2 launch easynav_playground_summit easynav_warehouse_amcl.launch.yaml

Both start Gazebo, the Summit XL, EasyNav and RViz2. Send goals with the **2D Goal Pose** tool.
Both use the same stack:

- **Maps managers**: Bonxai (the 3D map for localization) and NavMap (the surface to plan on),
  with an obstacle filter (what the sensors see) and an inflation filter.
- **Localizer**: AMCL for NavMap stacks (``easynav_navmap_localizer``), with the 3D lidar and the
  depth camera against the Bonxai map.
- **Planner**: A* over the NavMap (``easynav_navmap_planner``).
- **Controller**: Regulated Pure Pursuit. The PlayGround also has MPPI and MPC versions of the
  outdoor configuration (``easynav_mppi.launch.yaml``, ``easynav_mpc.launch.yaml``).

The parameter file
------------------

This is the warehouse configuration (``params/warehouse.amcl.params.yaml``), without the controller
(Regulated Pure Pursuit, as in :doc:`costmap_navigating`, scaled for the Summit XL) and the
recovery system.

Maps
^^^^

.. code-block:: yaml

   maps_manager_node:
     ros__parameters:
       use_sim_time: true
       map_types: [bonxai, navmap]
       bonxai:
         freq: 10.0
         plugin: easynav_bonxai_maps_manager/BonxaiMapsManager
         package: easynav_playground_summit
         bonxai_path_file: maps/warehouse.pcd
       navmap:
         freq: 10.0
         plugin: easynav_navmap_maps_manager/NavMapMapsManager
         package: easynav_playground_summit
         navmap_path_file: maps/warehouse_20cm.navmap
         filters: [obstacles, inflation]
         obstacles:
           plugin: easynav_navmap_maps_manager/NavMapMapsManager/ObstaclesFilter
           max_range: 3.0
           max_height: 1.2
         inflation:
           plugin: easynav_navmap_maps_manager/NavMapMapsManager/InflationFilter
           inflation_radius: 2.0
           cost_scaling_factor: 1.5

- The Bonxai map is loaded from a point cloud (``.pcd``).
- The NavMap is loaded from a ``.navmap`` file (``navmap_path_file``). It can also be built at
  startup from a 2D occupancy grid in the Nav2 YAML + image format (``occmap_path_file``), which
  gives a flat NavMap.
- The ``obstacles`` filter keeps the static map and adds the points the sensors see, within
  ``max_range`` and below ``max_height`` (robot frame).
- The ``inflation`` filter adds a cost around obstacles up to ``inflation_radius``, which keeps
  paths away from them. Cells closer than ``system_node.robot_geometry.inscribed_radius`` are
  blocked.

The outdoor configuration (``params/bonxai.amcl.params.yaml``) is the same with
``maps/excavation_urjc.pcd`` and ``maps/excavation_urjc.navmap``, and no range or height limits on
the obstacles.

Localization
^^^^^^^^^^^^

.. code-block:: yaml

   localizer_node:
     ros__parameters:
       use_sim_time: true
       localizer_types: [amcl]
       amcl:
         rt_freq: 50.0
         freq: 5.0
         reseed_freq: 0.1
         plugin: easynav_navmap_localizer/AMCLLocalizer
         downsampled_cloud_size: 10.0
         num_particles: 100
         noise_translation: 0.1
         noise_rotation: 0.03
         noise_translation_to_rotation: 0.02
         min_noise_xy: 0.2
         min_noise_yaw: 0.1
         compute_odom_from_tf: true
         initial_pose:
           x: 0.0
           y: 0.0
           yaw: 0.0
           std_dev_xy: 0.05
           std_dev_yaw: 0.01

   sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser, camera, imu, gps]
       perception_default_frame: odom
       laser:
         topic: front_laser/points
         type: sensor_msgs/msg/PointCloud2
       camera:
         topic: front_camera/depth/color/points
         type: sensor_msgs/msg/PointCloud2
       imu:
         topic: imu/data
         type: sensor_msgs/msg/Imu
       gps:
         topic: gps/fix
         type: sensor_msgs/msg/NavSatFix

In the warehouse, the localization error is about 0.15 m. In the excavation, the terrain is smooth
and the Bonxai cloud is sparse, so AMCL has little to correct against and its position can drift
0.5–1 m. Outdoors, ``easynav_gps.launch.yaml`` localizes with the GPS instead
(``easynav_fusion_localizer``, a UKF fusing the GPS position, wheel odometry and IMU heading), with
an error of about 0.15 m.

Planning and the robot
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

   planner_node:
     ros__parameters:
       use_sim_time: true
       planner_types: [astar]
       astar:
         freq: 1.0
         plugin: easynav_navmap_planner/AStarPlanner

   system_node:
     ros__parameters:
       robot_geometry:
         radius: 0.7
         inscribed_radius: 0.7
         height: 1.0
       use_sim_time: true
       use_real_time: true
       rt_freq: 50.0
       position_tolerance: 0.3
       angle_tolerance: 0.15

The A* planner weighs the path length against the cost of the cells it crosses (``cost_weight``,
default 5.0). The real-time cycle runs at 50 Hz (``rt_freq``; the default is 200 Hz), because the
collision reflex filters the 3D lidar and depth camera clouds in every cycle.

Building the maps
-----------------

- **Bonxai and NavMap from a point cloud map**: see :doc:`bonxai_navmap_from_rosbag`.
- **A flat NavMap from a 2D occupancy grid**: load the grid with ``occmap_path_file``, or convert
  it to a ``.navmap`` with the ``navmap_resample`` tool of ``navmap_tools``, which can
  also make the cells larger. The warehouse map was made with:

  .. code-block:: bash

     ros2 run navmap_tools navmap_resample maps/warehouse.yaml maps/warehouse_20cm.navmap 0.2

  20 cm cells instead of the grid's 5 cm: 16 times fewer cells, so the filters and the localizer
  keep up with their cycles.
- **Maps of a simulated world**: the Summit PlayGround's map building scripts (see
  :doc:`../playgrounds/summit`).
