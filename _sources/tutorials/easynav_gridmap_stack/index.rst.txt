
.. _easynav_gridmap_stack:

EasyNav Gridmap Stack Tutorials
*******************************

The ``easynav_gridmap_stack`` provides a working setup for EasyNav based on
`ANYbotics Grid Map <https://github.com/ANYbotics/grid_map>`_ (``grid_map``),
a multi-layer 2.5D grid representation well suited to outdoor or uneven terrain.
The key difference versus occupancy/cost maps is the **layered structure** (e.g., *elevation*),
with floating‑point values per cell and consistent geometry metadata.

It includes the following packages:

- ``easynav_gridmap_maps_manager``: Loads/saves ``grid_map`` maps (YAML + per‑layer images)
  and publishes/subscribes ``grid_map_msgs/msg/GridMap``.
- ``easynav_gridmap_astar_planner``: An A* planner that operates directly on ``grid_map``
  layers, constraining traversal by **maximum slope** and clearance.
- *(Localizer)* ``easynav_lidarslam_localizer`` (from the **easynav_lidarslam_ros2** repo, branch *rolling*):
  provides a Lidar SLAM‑based pose estimate used by this stack. It composes two nodes:
  **scan_matcher** and **graph_based_slam**, each with its own parameters.


The GridMap Representation
==========================

A ``grid_map::GridMap`` stores multiple **layers** over the same grid geometry.
Common layers include ``elevation`` (meters), traversability, surface normals, etc.
The geometry is defined by:

- **resolution** (meters/cell)
- **length** = ``[length_x, length_y]`` in meters (map extent)
- **position** = ``[x, y]`` of the map center in the world frame
- **frame_id** (e.g., ``map``)

Core operations include cell/world transforms (``getIndex``, ``getPosition``),
per‑cell access (``map.at("layer", index)``), validity checks (``isValid``),
copying layers by name, and conversion to ROS messages (``grid_map_msgs/msg/GridMap``)
or images via ``grid_map_cv``.

**YAML + image format used by this stack** (read/written by the maps manager):

.. code-block:: yaml

    resolution: 0.20              # meters/cell
    length: [40.0, 30.0]          # map extent in meters [x, y]
    position: [0.0, 0.0]          # map center in world coords [x, y]
    layers:
      - name: elevation
        min: -1.0
        max:  2.0
      - name: traversability
        min: 0.0
        max: 1.0

For each layer listed in ``layers``, there is a sibling image file alongside the YAML,
named ``<yaml_basename>_<layer>.pgm`` (e.g., ``pool_elevation.pgm``).
Values are scaled from the image into the specified ``[min, max]`` using
``grid_map_cv::addLayerFromImage``. When saving, the manager writes the same schema
and emits one PGM per layer using ``grid_map_cv::toImage``.

**Planner and slope constraint.**
The A* planner computes local slope from the **elevation** layer by sampling adjacent
cells, then rejects edges whose slope exceeds ``max_allowed_slope_deg``.
It also considers ``robot_radius`` and a ``clearance_distance`` margin.

.. note::
   If only the ``elevation`` layer is present, the manager keeps it. If multiple layers exist,
   the manager ensures the ``elevation`` data is refreshed, preserving the rest.


HowTo
=====

.. toctree::
   :maxdepth: 1

   ./mapping.rst
   ./navigating.rst


Stack Reference
===============

easynav_gridmap_maps_manager
----------------------------

Loads a static ``grid_map`` from YAML + images and republishes it as ``grid_map_msgs/msg/GridMap``.
Optionally updates the map if a new GridMap is received. Also exposes a ``savemap`` service.

**ROS Parameters (plugin section):**

+----------------+----------------------------------------------------+
| Parameter      | Description                                        |
+================+====================================================+
| package        | Package that contains the YAML + images            |
+----------------+----------------------------------------------------+
| map_path_file  | Relative path (inside ``package``) to the YAML     |
+----------------+----------------------------------------------------+

**Topics:**

Publishes:

- ``maps_manager_node/<plugin_name>/map``: ``grid_map_msgs/msg/GridMap``. QoS: **transient local, reliable**

Subscribes:

- ``maps_manager_node/<plugin_name>/incoming_map``: ``grid_map_msgs/msg/GridMap``.
  On reception the internal map and the published message are updated.

**Services:**

- ``maps_manager_node/<plugin_name>/savemap``: ``std_srvs/srv/Trigger`` —
  writes the current map to ``map_path_file`` (YAML + per‑layer PGM).

**NavState Access:**

+------------------+--------------------------+------------+
| Key              | Type                     | Direction  |
+==================+==========================+============+
| map              | grid_map::GridMap        | Read/Write |
+------------------+--------------------------+------------+


easynav_gridmap_astar_planner
-----------------------------

A grid‑map A* planner that reads the **elevation** layer to enforce a slope limit.

**ROS Parameters (plugin section):**

+---------------------------+------------------------------------------+
| Parameter                 | Description                              |
+===========================+==========================================+
| robot_radius (double)     | Robot radius [m]                         |
+---------------------------+------------------------------------------+
| clearance_distance(double)| Extra clearance margin [m]               |
+---------------------------+------------------------------------------+
| max_allowed_slope_deg     | Max slope allowed (degrees)              |
+---------------------------+------------------------------------------+

**Topics:**

Publishes:

- ``planner/path``: ``nav_msgs/msg/Path``

**NavState Access:**

+------------------+------------------------------+------------+
| Key              | Type                         | Direction  |
+==================+==============================+============+
| map              | grid_map::GridMap            | Read       |
+------------------+------------------------------+------------+
| robot_pose       | nav_msgs/msg/Odometry        | Read       |
+------------------+------------------------------+------------+
| goals            | nav_msgs/msg/Goals           | Read       |
+------------------+------------------------------+------------+
| path             | nav_msgs/msg/Path            | Write      |
+------------------+------------------------------+------------+


Localization (LidarSLAM for EasyNav)
------------------------------------

This stack uses the **easynav_lidarslam_ros2** (branch *rolling*) localizer.
It provides pose estimation via two cooperating nodes you launch/configure separately:

- **Scan Matcher** (node name: ``scan_matcher``)
- **Graph-Based SLAM** (node name: ``graph_based_slam``)

Typical ROS interfaces (from the implementation):

**Scan Matcher**

- Subscribes:
  - ``input_cloud``: ``sensor_msgs/msg/PointCloud2`` (LIDAR points)
  - ``imu``: ``sensor_msgs/msg/Imu`` *(optional; enable with ``use_imu``)*
  - ``initial_pose``: ``geometry_msgs/msg/PoseStamped`` (optional)

- Publishes:
  - ``current_pose``: ``geometry_msgs/msg/PoseStamped``
  - ``map``: ``sensor_msgs/msg/PointCloud2``
  - ``map_array``: ``lidarslam_msgs/msg/MapArray``
  - ``path``: ``nav_msgs/msg/Path``

- Selected parameters (see example below):
  ``registration_method`` (``NDT`` or ``GICP``), ``ndt_resolution``, ``ndt_num_threads``,
  ``gicp_corr_dist_threshold``, ``vg_size_for_input``, ``vg_size_for_map``,
  Min/Max range filters, map publish period, initial pose, and ``use_imu``/``use_odom`` flags.

**Graph-Based SLAM**

- Subscribes:
  - ``map_array``: ``lidarslam_msgs/msg/MapArray``

- Publishes:
  - ``modified_map``: ``sensor_msgs/msg/PointCloud2``
  - ``modified_map_array``: ``lidarslam_msgs/msg/MapArray``
  - ``modified_path``: ``nav_msgs/msg/Path``

- Selected parameters:
  ``registration_method`` (e.g., ``NDT``), ``ndt_resolution``, ``voxel_leaf_size``, loop detection
  period, loop closure thresholds/distances, number of adjacent constraints, and ``use_save_map_in_loop``.

The EasyNav **lidarslam localizer plugin** (``easynav_lidarslam_localizer/LidarSlamLocalizer``)
consumes the scan matcher’s pose/path outputs and updates the EasyNav NavState (``robot_pose``),
broadcasting the TF tree as configured.

.. tip::
   Configure **``scan_matcher``** and **``graph_based_slam``** *independently* in your launch
   or parameter files. The **localizer plugin** in ``localizer_node`` is separate from those
   two ROS nodes, but depends on their outputs.


Example Configuration
---------------------

Below is a minimal configuration tailored for the grid‑map stack plus the LidarSLAM localizer.
Replace package and map paths as needed.

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
          input_cloud: /front_laser_sensor/points
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
          robot_radius: 0.30
          clearance_distance: 0.20

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: /scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

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


YAML Map Schema (GridMap)
-------------------------

A grid‑map YAML (``map_path_file``) in this stack follows:

- ``resolution`` *(double)* — meters per cell
- ``length`` *(list[double, double])* — map size in meters: ``[len_x, len_y]``
- ``position`` *(list[double, double])* — map center in world: ``[x, y]``
- ``layers`` *(sequence of maps)*:

  - ``name`` *(string)* — layer name (e.g., ``elevation``)
  - ``min`` *(float)* — value corresponding to pixel 0
  - ``max`` *(float)* — value corresponding to pixel 255

Layer rasters live next to the YAML as ``<basename>_<layer>.pgm`` and are
loaded/saved using ``grid_map_cv`` converters.

