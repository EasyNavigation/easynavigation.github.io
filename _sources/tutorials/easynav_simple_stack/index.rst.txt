
.. _easynav_simple_stack:

EasyNav Simple Stack Tutorials
******************************

The ``easynav_simple_stack`` provides a minimal working setup for EasyNav based on a 2D binary occupancy grid.
This stack is ideal for introductory testing and basic indoor navigation scenarios.

It includes the following packages:

- ``easynav_simple_common``: Defines the ``SimpleMap`` structure used across the stack.
- ``easynav_simple_controller``: A simple controller that follows a path using a PID-like approach.
- ``easynav_simple_localizer``: A basic AMCL-like localizer using 2D occupancy grid maps.
- ``easynav_simple_maps_manager``: A manager that loads a static map and maintains a dynamic map updated from perceptions.
- ``easynav_simple_planner``: A basic A* planner that generates paths over the map.


The SimpleMap Representation
============================

At the heart of this stack is the ``SimpleMap`` data structure. It represents the environment as a 2D occupancy grid
where each cell can be either 0 (free), 1 (occupied), or -1 (unknown). ``SimpleMap`` supports:

- Resolution and size configuration
- Conversion to and from ROS 2 ``nav_msgs/msg/OccupancyGrid``
- Metric/cell coordinate transformation
- Basic drawing and manipulation utilities

``SimpleMap`` is used for both static maps (loaded from file) and dynamic maps (updated from perceptions).


HowTo
=====

.. toctree::
   :maxdepth: 1

   ./mapping.rst
   ./navigating.rst


Stack Reference
===============

easynav_simple_maps_manager
---------------------------

Manages the static ``SimpleMap`` (loaded from disk) and a dynamic ``SimpleMap`` updated from perceptions. Also publishes both maps as occupancy grids.

**ROS Parameters (plugin section):**

+----------------+-----------------------------------------------+
| Parameter      | Description                                   |
+================+===============================================+
| package        | Package name that contains the map file       |
+----------------+-----------------------------------------------+
| map_path_file  | Relative path (within ``package``) to ``.map``|
+----------------+-----------------------------------------------+

**Topics:**

Publishes:

- ``maps_manager_node/<plugin_name>/map``: ``nav_msgs/msg/OccupancyGrid`` (static). QoS: transient local, reliable
- `maps_manager_node/<plugin_name>/dynamic_map``: ``nav_msgs/msg/OccupancyGrid`` (dynamic). QoS: depth 100

Subscribes:

- ``maps_manager_node/<plugin_name>/incoming_map``: ``nav_msgs/msg/OccupancyGrid``. QoS: transient local, reliable

**Services:**

- ``maps_manager_node/<plugin_name>/savemap``: ``std_srvs/srv/Trigger`` — saves the current static map back to ``map_path_file``

**NavState Access:**

+------------------+--------------------------+------------+
| Key              | Type                     | Direction  |
+==================+==========================+============+
| points           | PointPerceptions         | Read       |
+------------------+--------------------------+------------+
| map.static       | SimpleMap                | Write      |
+------------------+--------------------------+------------+
| map.dynamic      | SimpleMap                | Write      |
+------------------+--------------------------+------------+


easynav_simple_localizer
------------------------

AMCL-style particle filter localizer. Subscribes to odometry, uses the static map and point perceptions, publishes estimated pose (with covariance), particle cloud, and broadcasts TF.

**ROS Parameters (plugin section):**

+---------------------------------------+------------------------------------+
| Parameter                             | Description                        |
+=======================================+====================================+
| num_particles (int, default: 100)     | Number of particles                |
+---------------------------------------+------------------------------------+
| initial_pose.x (double, default: 0.0) | Initial x                          |
+---------------------------------------+------------------------------------+
| initial_pose.y (double, default: 0.0) | Initial y                          |
+---------------------------------------+------------------------------------+
| initial_pose.yaw (double, 0.0)        | Initial yaw (rad)                  |
+---------------------------------------+------------------------------------+
| initial_pose.std_dev_xy (double, 0.5) | Initial stddev for x,y (m)         |
+---------------------------------------+------------------------------------+
| initial_pose.std_dev_yaw (double,0.5) | Initial stddev for yaw (rad)       |
+---------------------------------------+------------------------------------+
| reseed_freq (double, default: 1.0)    | Particle reseed frequency (Hz)     |
+---------------------------------------+------------------------------------+
| noise_translation (double, 0.01)      | Motion noise (m)                   |
+---------------------------------------+------------------------------------+
| noise_rotation (double, 0.01)         | Motion noise (rad)                 |
+---------------------------------------+------------------------------------+
| noise_translation_to_rotation (0.01)  | Coupling noise (m→rad)             |
+---------------------------------------+------------------------------------+
| min_noise_xy (double, 0.05)           | Minimum XY noise clamp             |
+---------------------------------------+------------------------------------+
| min_noise_yaw (double, 0.05)          | Minimum Yaw noise clamp            |
+---------------------------------------+------------------------------------+

**Topics:**

Publishes:

- ``localizer_node/<plugin_name>/particles``: ``geometry_msgs/msg/PoseArray``
- ``localizer_node/<plugin_name>/pose``: ``geometry_msgs/msg/PoseWithCovarianceStamped``

Subscribes:

- ``odom``: ``nav_msgs/msg/Odometry`` (``rclcpp::SensorDataQoS().reliable()``)

**TF:**

- Broadcasts ``map → odom`` (header.frame_id = ``<tf_prefix>map``, child = ``<tf_prefix>odom``)

**NavState Access:**

+------------------+--------------------------+------------+
| Key              | Type                     | Direction  |
+==================+==========================+============+
| map.static       | SimpleMap                | Read       |
+------------------+--------------------------+------------+
| points           | PointPerceptions         | Read       |
+------------------+--------------------------+------------+
| robot_pose       | nav_msgs/msg/Odometry    | Write      |
+------------------+--------------------------+------------+


easynav_simple_planner
----------------------

A grid-based A* planner over ``SimpleMap``. Produces a path and publishes it.

**ROS Parameters (plugin section):**

+------------------------+-----------------------------------+
| Parameter              | Description                       |
+========================+===================================+
| robot_radius (double)  | Robot radius in meters            |
+------------------------+-----------------------------------+
| clearance_distance     | Additional clearance (meters)     |
| (double)               |                                   |
+------------------------+-----------------------------------+

**Topics:**

Publishes:

- ``planner_node/<plugin_name>/path``: ``nav_msgs/msg/Path``

**NavState Access:**

+------------------+------------------------------+------------+
| Key              | Type                         | Direction  |
+==================+==============================+============+
| map.dynamic      | SimpleMap                    | Read       |
+------------------+------------------------------+------------+
| robot_pose       | nav_msgs/msg/Odometry        | Read       |
+------------------+------------------------------+------------+
| goals            | nav_msgs/msg/Goals           | Read       |
+------------------+------------------------------+------------+
| path             | nav_msgs/msg/Path            | Write      |
+------------------+------------------------------+------------+


easynav_simple_controller
-------------------------

A real-time path-following controller that writes velocity commands into the NavState. It uses internal PID components, but **PID gains are not exposed as ROS parameters** in this plugin.

**ROS Parameters (plugin section):**

+----------------------+-------------------------------------------+
| Parameter            | Description                               |
+======================+===========================================+
| max_linear_speed     | Max forward speed (m/s)                   |
+----------------------+-------------------------------------------+
| max_angular_speed    | Max rotational speed (rad/s)              |
+----------------------+-------------------------------------------+
| max_linear_acc       | Max linear acceleration (m/s²)            |
+----------------------+-------------------------------------------+
| max_angular_acc      | Max angular acceleration (rad/s²)         |
+----------------------+-------------------------------------------+
| look_ahead_dist      | Look-ahead distance for tracking (m)      |
+----------------------+-------------------------------------------+
| tolerance_dist       | Position tolerance to consider goal (m)   |
+----------------------+-------------------------------------------+
| k_rot                | Angular correction factor                 |
+----------------------+-------------------------------------------+

**Topics:**

- *(None in this plugin; velocity is written to NavState. The system bridge ``cmd_vels`` to a ROS topic.)*

**NavState Access:**

+------------------+--------------------------------+------------+
| Key              | Type                           | Direction  |
+==================+================================+============+
| robot_pose       | nav_msgs/msg/Odometry          | Read       |
+------------------+--------------------------------+------------+
| path             | nav_msgs/msg/Path              | Read       |
+------------------+--------------------------------+------------+
| cmd_vel          | geometry_msgs/msg/TwistStamped | Write      |
+------------------+--------------------------------+------------+


Example Configuration
---------------------

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [simple]
        simple:
          plugin: easynav_simple_controller/SimpleController
          max_linear_speed: 0.6
          max_angular_speed: 1.0
          max_linear_acc: 1.0
          max_angular_acc: 2.0
          look_ahead_dist: 0.2
          tolerance_dist: 0.05
          k_rot: 0.5

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [simple]
        simple:
          plugin: easynav_simple_localizer/AMCLLocalizer
          num_particles: 100
          reseed_freq: 1.0
          noise_translation: 0.01
          noise_rotation: 0.01
          noise_translation_to_rotation: 0.01
          min_noise_xy: 0.05
          min_noise_yaw: 0.05
          initial_pose:
            x: 0.0
            y: 0.0
            yaw: 0.0
            std_dev_xy: 0.5
            std_dev_yaw: 0.5

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [simple]
        simple:
          plugin: easynav_simple_maps_manager/SimpleMapsManager
          package: easynav_playground_kobuki
          map_path_file: maps/home.map

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          plugin: easynav_simple_planner/SimplePlanner
          robot_radius: 0.3
          clearance_distance: 0.2

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1, camera1]
        perception_default_frame: odom
        laser1:
          topic: /scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points
        camera1:
          topic: /rgbd_camera/points
          type: sensor_msgs/msg/PointCloud2
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05
