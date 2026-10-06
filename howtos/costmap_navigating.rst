.. _costmap_navigating:

=================================
Navigating with the Costmap Stack
=================================

This HowTo shows how to perform **navigation** with the **Costmap Stack** in **EasyNavigation
(EasyNav)**: a costmap with obstacles and inflation, AMCL, A* and the **Regulated Pure Pursuit**
controller. It is EasyNav's reference configuration for indoor robots with a 2D lidar.

.. contents:: On this page
   :local:
   :depth: 2

Setup
-----

Build EasyNav and the Kobuki PlayGround as described in :doc:`../getting_started/index`.

The PlayGround ships a map of its world (``maps/home2.yaml``). To navigate in your own map, create
it first following :doc:`costmap_mapping`, or use an existing Nav2 map.

Running it
----------

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml

It starts Gazebo, the Kobuki, EasyNav with ``params/costmap.rpp.params.yaml`` and RViz2. In
**RViz2**, use the **"2D Goal Pose"** tool to send navigation goals. You can disable the Gazebo GUI
with ``gui:=false``, and use your own parameter file with ``params_file:=/path/to/my.params.yaml``.

The Parameter File
------------------

This is ``params/costmap.rpp.params.yaml`` of the Kobuki PlayGround:

- The **Costmap Maps Manager** loads the map, adds the obstacles the sensors see
  (``ObstaclesFilter``) and inflates them (``InflationFilter``).
- The **Costmap Localizer**, an AMCL over the costmap.
- The **Costmap Planner**, an A* over the costmap that keeps paths away from obstacles
  (``cost_factor``) and replans continuously.
- The **Regulated Pure Pursuit** controller, a port of Nav2's.
- The **Diagnostic recovery system**: a collision safety reflex and recoveries (see below).

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        robot_limits:
          max_linear_vel: 0.6
          min_linear_vel: -0.3
          max_angular_vel: 1.2
          max_linear_acc: 1.0
          max_linear_decel: 1.0
          max_angular_acc: 2.0
          max_angular_decel: 2.0
        controller_types: [rpp]
        rpp:
          rt_freq: 30.0
          plugin: easynav_regulated_pp_controller/RegulatedPurePursuitController
          safety_margin: 0.05
          use_dynamic_window: false
          allow_reversing: false
          lookahead_dist: 0.4
          min_lookahead_dist: 0.2
          max_lookahead_dist: 0.6
          lookahead_time: 1.2
          use_velocity_scaled_lookahead_dist: true
          use_rotate_to_heading: true
          rotate_to_heading_angular_vel: 1.0
          rotate_to_heading_min_angle: 0.785
          use_regulated_linear_velocity_scaling: true
          regulated_linear_scaling_min_radius: 0.9
          regulated_linear_scaling_min_speed: 0.15
          use_fixed_curvature_lookahead: false
          curvature_lookahead_dist: 1.0
          interpolate_curvature_after_goal: false
          use_obstacle_regulated_linear_velocity_scaling: true
          obstacle_scaling_dist: 0.4
          obstacle_scaling_gain: 0.8
          min_approach_linear_velocity: 0.05
          approach_velocity_scaling_dist: 0.6
          xy_goal_tolerance: 0.1
          yaw_goal_tolerance: 0.105

    localizer_node:
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
            y: 0.1
            yaw: 0.0
            std_dev_xy: 0.1
            std_dev_yaw: 0.01

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [costmap]
        costmap:
          freq: 10.0
          plugin: easynav_costmap_maps_manager/CostmapMapsManager
          package: easynav_playground_kobuki
          map_path_file: maps/home2.yaml
          filters: [obstacles, inflation]
          obstacles:
            plugin: easynav_costmap_maps_manager/CostmapMapsManager/ObstaclesFilter
          inflation:
            plugin: easynav_costmap_maps_manager/CostmapMapsManager/InflationFilter
            inflation_radius: 1.0
            cost_scaling_factor: 5.0

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_costmap_planner/CostmapPlanner
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
        robot_geometry:
          radius: 0.178
          inscribed_radius: 0.178
          height: 0.5
        use_sim_time: true
        use_real_time: true
        position_tolerance: 0.3
        angle_tolerance: 0.15

The file ends with the ``recovery_node`` section, the **Diagnostic recovery system**:

- a **collision safety reflex**, checked every control cycle, that brakes if the command would hit
  an obstacle;
- **evaluators** that diagnose problems: no path to the goal, an obstacle too close, a robot that
  does not progress, a lost AMCL localization, or a miswired ROS graph;
- **mitigations** that fix them, in priority order: retreat from the obstacle, rotate to
  relocalize, advance a little; terminate EasyNav on a miswired graph; wait for a human; and, as
  the last resort, cancel the mission.

See :ref:`recovery` and the
`easynav_diagnostic_recovery README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_diagnostic_recovery/README.md>`_
for its parameters.

Adapting it to your robot
-------------------------

- **Map**: set ``package`` and ``map_path_file`` (both are needed: the map is looked up in the
  share directory of ``package``). It must be a YAML + image pair, as Nav2's, not the Simple
  stack's ``.map`` file.
- **Robot**: ``system_node.robot_geometry`` (radius, inscribed radius and height) is shared by
  every component; ``controller_node.robot_limits`` holds the velocity and acceleration limits,
  enforced on every command.
- **Sensors**: list your lidar under ``sensors_node.sensors`` with its topic and type.
- **Distance to obstacles**: tune ``inflation_radius`` and ``cost_scaling_factor`` under the
  ``inflation`` filter, and ``cost_factor`` in the planner.
- **Other controllers**: the Kobuki PlayGround has the same stack with MPPI, MPC and SeReST (see
  :doc:`../playgrounds/kobuki`). Every plugin's parameters are in its README (see
  :doc:`../plugins/index`).
