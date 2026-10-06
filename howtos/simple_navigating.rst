.. _simple_navigating:

=======================================
Navigating with SimpleStack and EasyNav
=======================================

This HowTo explains how to perform **navigation using the Simple Stack** in EasyNavigation (EasyNav).
The Simple Stack operates on a **binary occupancy map**.

.. warning::
   The Simple stack is a minimal example, with very basic algorithms, made to show how EasyNav
   works and how plugins are written. Do not expect good navigation from it. For real use, see
   :doc:`costmap_navigating`.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/p4aqNA0JNhA" frameborder="0" allowfullscreen></iframe>
    </div>

This tutorial uses the :doc:`Kobuki PlayGround <../playgrounds/kobuki>`, which ships
a map of its world (``maps/home.map``). To navigate in your own map, create it first following
:doc:`simple_mapping`.

Setup
-----

Build EasyNav and the Kobuki PlayGround as described in :doc:`../getting_started/index`.

The Parameter File
------------------

In this example we will use:

- The **Simple Maps Manager** to load the binary map.
- The **AMCL localizer** of the Simple stack (``easynav_simple_localizer``).
- The **Simple Planner**, an A* over the binary map.
- The **SeReST controller** for path tracking (see :doc:`serest_controller`).
- The **Simple recovery system**, which brakes before an obstacle ahead, rotates to relocalize and
  backs up when stuck (see :ref:`recovery`).

This is ``params/simple.serest.params.yaml`` of the Kobuki PlayGround:

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        robot_limits:
          max_linear_vel: 0.6
          min_linear_vel: -0.6
          max_angular_vel: 1.5
          max_linear_acc: 0.8
          max_linear_decel: 1.0
          max_angular_acc: 2.0
          max_angular_decel: 2.0
        controller_types: [serest]
        serest:
          rt_freq: 30.0
          plugin: easynav_serest_controller/SerestController
          allow_reverse: true
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
          package: easynav_playground_kobuki
          map_path_file: maps/home.map

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_simple_planner/SimplePlanner

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan

    system_node:
      ros__parameters:
        use_sim_time: true
        robot_geometry:
          radius: 0.25
          height: 0.5
        position_tolerance: 0.3
        angle_tolerance: 0.15

    recovery_node:
      ros__parameters:
        use_sim_time: true
        recovery_manager:
          plugin: easynav_simple_recovery/SimpleRecoveryManager
          stop_distance: 0.3
          sensors_timeout: 5.0
          max_position_variance: 1.0

To use your own map, change ``package`` and ``map_path_file`` (both are needed: the map is looked
up in the share directory of ``package``).

Running the Simulation
----------------------

1. **Launch the simulator, EasyNav and RViz2**:

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki easynav_simple_serest.launch.yaml

   You can disable the Gazebo GUI with ``gui:=false``, and use your own parameter file with
   ``params_file:=/path/to/my.params.yaml``.

2. **Send navigation goals**: in RViz2, use the **"2D Goal Pose"** tool. The robot will plan and
   navigate toward them.

Notes
-----

- The **Simple Stack** uses a **binary map**: cells are free or occupied (no graded cost), so paths
  are not kept away from obstacles. For cost-aware planning, use the **Costmap Stack**
  (:doc:`costmap_navigating`).
- ``easynav_simple.launch.yaml`` runs the same stack with the Simple controller, the one of
  :doc:`../getting_started/index`.
