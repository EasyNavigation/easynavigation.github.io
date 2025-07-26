.. easynav_simple_stack:

EasyNav Simple Stack Tutorials
******************************


The ``easynav_simple_stack`` provides a minimal working setup for EasyNav based on a 2D binary occupancy grid.
This stack is ideal for introductory testing and basic indoor navigation scenarios.

It includes the following packages:

- ``easynav_simple_common``: Defines the ``SimpleMap`` structure used across the stack.
- ``easynav_simple_controller``: A simple controller that follows a path using a PID approach.
- ``easynav_simple_localizer``: A basic AMCL-like localizer using 2D occupancy grid maps.
- ``easynav_simple_maps_manager``: A manager that fuses a static map with incoming perceptions.
- ``easynav_simple_planner``: A basic A* planner that generates paths over the map.

-------------------------------
The SimpleMap Representation
-------------------------------

At the heart of this stack is the ``SimpleMap`` data structure. It represents the environment as a 2D occupancy grid
where each cell can be either 0 (free), 1 (occupied), or -1 (unknown). ``SimpleMap`` supports:

- Resolution and size configuration
- Conversion to and from ROS 2 ``nav_msgs/msg/OccupancyGrid``
- Metric/cell coordinate transformation
- Basic drawing and manipulation utilities

``SimpleMap`` is used for both static maps (loaded from file) and dynamic maps (updated from perceptions).


-----
HowTo
-----

.. toctree::
   :maxdepth: 1

   ./mapping.rst



---------------
Stack Reference
---------------

-------------------------------
easynav_simple_maps_manager
-------------------------------

This component manages and fuses the static map and incoming perceptions into a dynamic map.

**ROS Parameters:**

+----------------+----------------------------+
| Parameter      | Description                |
+================+============================+
| freq           | Execution frequency (Hz)   |
+----------------+----------------------------+
| plugin         | Plugin name                |
+----------------+----------------------------+
| package        | Package where map lives    |
+----------------+----------------------------+
| map_path_file  | Path to .map file          |
+----------------+----------------------------+

**Topics:**

Publishes:

- ``/dynamic_map``: ``nav_msgs/msg/OccupancyGrid``

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

-------------------------------
easynav_simple_localizer
-------------------------------

A basic AMCL-style localizer using particles and static maps.

**ROS Parameters:**

+---------------------------+----------------------------+
| Parameter                 | Description                |
+===========================+============================+
| plugin                    | Plugin name                |
+---------------------------+----------------------------+
| rt_freq                   | Real-time update freq (Hz) |
+---------------------------+----------------------------+
| freq                      | Slow cycle update freq     |
+---------------------------+----------------------------+
| reseed_freq               | Reseed frequency (Hz)      |
+---------------------------+----------------------------+
| num_particles             | Number of particles        |
+---------------------------+----------------------------+
| noise_translation         | Noise (m)                  |
+---------------------------+----------------------------+
| noise_rotation            | Noise (rad)                |
+---------------------------+----------------------------+
| initial_pose.{x,y,yaw...} | Initial pose + stddev      |
+---------------------------+----------------------------+

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

-------------------------------
easynav_simple_planner
-------------------------------

An A* planner over ``SimpleMap`` representations.

**ROS Parameters:**

+----------------+------------------------------+
| Parameter      | Description                  |
+================+==============================+
| plugin         | Plugin name                  |
+----------------+------------------------------+
| freq           | Execution frequency (Hz)     |
+----------------+------------------------------+
| robot_radius   | Robot radius in meters       |
+----------------+------------------------------+

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

-------------------------------
easynav_simple_controller
-------------------------------

A PID-based controller that tracks a path in real time.

**ROS Parameters:**

+----------------------+----------------------------+
| Parameter            | Description                |
+======================+============================+
| plugin               | Plugin name                |
+----------------------+----------------------------+
| rt_freq              | Real-time frequency (Hz)   |
+----------------------+----------------------------+
| max_linear_speed     | Max forward velocity (m/s) |
+----------------------+----------------------------+
| max_angular_speed    | Max rotation speed (rad/s) |
+----------------------+----------------------------+
| look_ahead_dist      | Look-ahead distance (m)    |
+----------------------+----------------------------+
| k_rot                | Angular correction factor  |
+----------------------+----------------------------+

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

-------------------------------
Example Configuration
-------------------------------

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [simple]
        simple:
          rt_freq: 30.0 
          plugin: easynav_simple_controller/SimpleController
          max_linear_speed: 0.6
          max_angular_speed: 1.0
          look_ahead_dist: 0.2
          k_rot: 0.5

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
          package: easynav_playground_kobuki
          map_path_file: maps/home.map

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_simple_planner/SimplePlanner
          robot_radius: 0.3

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
        camera1:
          topic: /rgbd_camera/points
          type: sensor_msgs/msg/PointCloud2
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

