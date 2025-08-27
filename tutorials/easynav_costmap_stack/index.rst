.. easynav_costmap_stack:

EasyNav Costmap Stack Tutorials
*******************************

The ``easynav_costmap_stack`` provides a working setup for EasyNav based on
a **costmap representation** instead of a simple binary map.
This stack is suitable for more advanced navigation scenarios, including
obstacle inflation and graded traversability information.

It includes the following packages:

- ``easynav_costmap_common``: Defines the ``Costmap2D`` structure and utility functions.
- ``easynav_costmap_maps_manager``: Handles loading, saving, filtering and publishing costmaps.
- ``easynav_costmap_localizer``: An AMCL-like localizer adapted to costmaps.
- ``easynav_costmap_planner``: A grid-based planner (A*) that operates over costmaps.


The SimpleMap Representation
============================

At the core of this stack lies the ``Costmap2D`` data structure.

``Costmap2D`` extends the binary occupancy grid into a graded cost representation
with values in the range [0–255]:

- ``0``: Free space, no cost to traverse.
- ``1–252``: Gradual cost values, representing increasing difficulty or proximity to obstacles.
- ``253``: "Near obstacle" (inscribed obstacle) cost, traversal strongly discouraged.
- ``254``: Lethal obstacle, occupied cell.
- ``255``: Unknown space.

**Key features of Costmap2D:**

- Resolution, width and height configuration
- World ↔ grid coordinate transformations
- Cell accessors (getCost, setCost)
- Conversion to and from ``nav_msgs/msg/OccupancyGrid``
- Support for layered costmaps via filters (e.g. obstacle inflation)
- Efficient raytracing and Bresenham line traversal utilities

``Costmap2D`` enables representing not only obstacles, but also graded traversability,
which is crucial for safe path planning and smooth motion around obstacles.



HowTo
=====

.. toctree::
   :maxdepth: 1

   ./mapping.rst
   ./navigating.rst
   ./navigating_with_icreate.rst

Stack Reference
===============

easynav_costmap_maps_manager
----------------------------

The **maps manager** is responsible for:
1) Loading the *static* costmap from a YAML+image pair (``.yaml`` + ``.pgm/.png``),
2) Maintaining a *dynamic* costmap derived from the static map and sensor perceptions, and
3) Applying an **ordered pipeline of costmap filters** that progressively modify the working map.

**Filter pipeline (order matters):**

- At the beginning of every cycle, the manager copies the static map into an internal working buffer
  and stores it in the blackboard as ``map.dynamic.filtered``.
- Then it iterates the list in ``filters`` **in order**. Each filter reads and updates
  ``map.dynamic.filtered`` in-place (via ``NavState``).
- After the last filter finishes, the resulting buffer is published and also copied to ``map.dynamic``.

This design lets you chain effects deterministically. For example, you can **first** add obstacles from sensors
and **then** inflate them; reversing the order would inflate nothing (because obstacles would be added after inflation).

**ROS Parameters (plugin section of this maps manager):**

+--------------------+------------------------------------------------------------+
| Parameter          | Description                                                |
+====================+============================================================+
| package            | ROS package that contains the YAML map file                |
+--------------------+------------------------------------------------------------+
| map_path_file      | Path to the YAML file *relative to* ``package``            |
+--------------------+------------------------------------------------------------+
| filters            | **Ordered** list of filter IDs to load and execute         |
+--------------------+------------------------------------------------------------+

For each entry ``<filter_id>`` listed in ``filters``, you must create a peer subsection
``<filter_id>`` with at least its ``plugin`` field and any **filter‑specific** parameters.
Those parameters belong to the filter plugin, not to the maps manager.

**Built‑in filters:**

1) ``easynav_costmap_maps_manager/ObstacleFilter``  
   - **Purpose:** Fuses point‑based perceptions (``points`` in NavState) into the working costmap (``map.dynamic.filtered``), marking cells as obstacles where required.
   - **Parameters:** *None.* This filter uses the costmap resolution and TF prefix, and reads ``points`` from NavState to project measurements into the map.

2) ``easynav_costmap_maps_manager/InflationFilter``  
   - **Purpose:** Inflates obstacles by assigning costs that decay with distance from lethal cells, producing a graded safety buffer around obstacles.
   - **Parameters (declared by the plugin):**

      +------------------------+----------------------------------------------+
      | Parameter              | Meaning                                      |
      +========================+==============================================+
      | inflation_radius       | Max inflation distance (meters)              |
      +------------------------+----------------------------------------------+
      | cost_scaling_factor    | Exponential decay factor for the cost field  |
      +------------------------+----------------------------------------------+

   - **Notes:** The plugin internally derives cell radii from the map resolution and uses standard
     cost levels (``LETHAL_OBSTACLE=254``, ``INSCRIBED_INFLATED_OBSTACLE=253``) from ``cost_values.hpp``.
     Parameters like *inscribed* or *lethal* cost are **not** user parameters in this implementation.

**Topics:**

Publishes:

- ``maps_manager_node/<plugin_name>/map`` : ``nav_msgs/msg/OccupancyGrid`` (static). QoS: transient local, reliable
- ``maps_manager_node/<plugin_name>/dynamic_map`` : ``nav_msgs/msg/OccupancyGrid`` (dynamic). QoS: depth 100

Subscribes:

- ``maps_manager_node/<plugin_name>/incoming_map`` : ``nav_msgs/msg/OccupancyGrid`` (optional override of static map).
  If a message arrives here, it replaces the internal static map and re‑publishes the static layer.

**Services:**

- ``maps_manager_node/<plugin_name>/savemap`` : ``std_srvs/srv/Trigger`` — saves the current static map back to the
  YAML path resolved from (``package``, ``map_path_file``).

**NavState Access:**

+------------------------+------------------+-------------+
| Key                    | Type             | Directione  |
+========================+==================+=============+
| map.static             | Costmap2D        | Read/Write  |
+------------------------+------------------+-------------+
| map.dynamic.filtered   | Costmap2D        | Read/Write  |
+------------------------+------------------+-------------+
| map.dynamic            | Costmap2D        |    Write    |
+------------------------+------------------+-------------+
| points                 | PointPerceptions |    Read     |
+------------------------+------------------+-------------+

**YAML map fields supported (loaded by ``map_io``):**

It is **the same** format used in MoveBase/Nav2.

+------------------+--------------------------------------------------+
| Field            | Meaning                                          |
+==================+==================================================+
| image            | Path to the image file (PGM/PNG)                 |
+------------------+--------------------------------------------------+
| resolution       | Meters per cell                                  |
+------------------+--------------------------------------------------+
| origin           | [x, y, yaw] of the map origin in meters/radians  |
+------------------+--------------------------------------------------+
| negate           | If 1, invert colors when loading                 |
+------------------+--------------------------------------------------+
| free_thresh      | Threshold (0–1) for free cells                   |
+------------------+--------------------------------------------------+
| occupied_thresh  | Threshold (0–1) for occupied cells               |
+------------------+--------------------------------------------------+
| mode             | One of ``Trinary | Scale | Raw``                 |
+------------------+--------------------------------------------------+

**Example:** Ordered filters where obstacles are inserted first and then inflated

.. code-block:: yaml

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [costmap]
        costmap:
          plugin: easynav_costmap_maps_manager/CostmapMapsManager
          package: my_maps_pkg
          map_path_file: maps/office.yaml
          filters: [obstacles, inflation]

          obstacles:
            plugin: easynav_costmap_maps_manager/ObstacleFilter
            # (no extra params)

          inflation:
            plugin: easynav_costmap_maps_manager/InflationFilter
            inflation_radius: 0.45
            cost_scaling_factor: 2.5

Loads static costmaps from file (YAML + PGM), maintains dynamic costmaps, and applies filters
such as inflation or obstacle masking.

**ROS Parameters (plugin section):**

+--------------------+----------------------------------------------+
| Parameter          | Description                                  |
+====================+==============================================+
| package            | Package where the map YAML file is located   |
+--------------------+----------------------------------------------+
| map_path_file      | Path to the YAML map file                    |
+--------------------+----------------------------------------------+
| filters            | List of filters to apply (e.g. inflation)    |
+--------------------+----------------------------------------------+

**Topics:**

Publishes:

- ``maps_manager_node/<plugin>/map``: ``nav_msgs/msg/OccupancyGrid`` (static map)
- ``maps_manager_node/<plugin>/dynamic_map``: ``nav_msgs/msg/OccupancyGrid`` (dynamic map)

Subscribes:

- ``maps_manager_node/<plugin>/incoming_map``: ``nav_msgs/msg/OccupancyGrid``

**Services:**

- ``maps_manager_node/<plugin>/savemap``: ``std_srvs/srv/Trigger`` — saves map back to disk

**NavState Access:**

+------------------+--------------------------+------------+
| Key              | Type                     | Direction  |
+==================+==========================+============+
| points           | PointPerceptions         | Read       |
+------------------+--------------------------+------------+
| map.static       | Costmap2D                | Write      |
+------------------+--------------------------+------------+
| map.dynamic      | Costmap2D                | Write      |
+------------------+--------------------------+------------+


easynav_costmap_localizer
-------------------------

A particle-filter localizer (AMCL-like) that uses costmaps as environment representation.

**ROS Parameters (plugin section):**

- Identical in structure to the simple AMCL localizer, with the addition that it queries ``Costmap2D`` instead of ``SimpleMap``.


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

- Publishes ``map → odom`` transform

**NavState Access:**

+------------------+--------------------------+------------+
| Key              | Type                     | Direction  |
+==================+==========================+============+
| map.static       | Costmap2D                | Read       |
+------------------+--------------------------+------------+
| points           | PointPerceptions         | Read       |
+------------------+--------------------------+------------+
| robot_pose       | nav_msgs/msg/Odometry    | Write      |
+------------------+--------------------------+------------+


easynav_costmap_planner
-----------------------

A planner based on the A* algorithm adapted for ``Costmap2D``.
Takes into account costs for safer and smoother paths.

**ROS Parameters (plugin section):**

+------------------------+------------------------------------+
| Parameter              | Description                        |
+========================+====================================+
| robot_radius           | Robot radius (meters)              |
+------------------------+------------------------------------+
| clearance_distance     | Additional clearance (meters)      |
+------------------------+------------------------------------+
| cost_penalty_factor    | Extra penalty for higher-cost cells|
+------------------------+------------------------------------+

**Topics:**

Publishes:

- ``planner_node/<plugin_name>/path``: ``nav_msgs/msg/Path``

**NavState Access:**

+------------------+------------------------------+------------+
| Key              | Type                         | Direction  |
+==================+==============================+============+
| map.dynamic      | Costmap2D                    | Read       |
+------------------+------------------------------+------------+
| robot_pose       | nav_msgs/msg/Odometry        | Read       |
+------------------+------------------------------+------------+
| goals            | nav_msgs/msg/Goals           | Read       |
+------------------+------------------------------+------------+
| path             | nav_msgs/msg/Path            | Write      |
+------------------+------------------------------+------------+


Example Configuration
---------------------

An example parameters file (from ``costmap.serest.params.yaml``):

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [serest]
        serest:
          plugin: easynav_costmap_controller/SerestController
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
        localizer_types: [amcl]
        amcl:
          plugin: easynav_costmap_localizer/AMCLLocalizer
          num_particles: 200
          reseed_freq: 1.0
          noise_translation: 0.01
          noise_rotation: 0.01
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
        map_types: [costmap]
        costmap:
          plugin: easynav_costmap_maps_manager/CostmapMapsManager
          package: easynav_playground_kobuki
          map_path_file: maps/home.yaml
          filters: [obstacles, inflation]
          obstacles:
            plugin: easynav_costmap_maps_manager/ObstacleFilter
          inflation:
            plugin: easynav_costmap_maps_manager/InflationFilter
            inflation_radius: 0.4
            inscribed_cost: 253
            lethal_cost: 254

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [a_star]
        a_star:
          plugin: easynav_costmap_planner/CostmapPlanner
          robot_radius: 0.3
          clearance_distance: 0.2
          cost_penalty_factor: 2.0

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05
