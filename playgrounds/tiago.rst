.. _playground_tiago:

================
TIAGo PlayGround
================

`easynav_playground_tiago <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_tiago>`_
simulates PAL Robotics' **TIAGo** in the AWS RoboMaker small house: its mobile base (pmb2),
lifting torso, pan-tilt head with an RGBD camera, 7-DoF arm with the PAL gripper and wrist
force/torque sensor, base laser, sonars and IMU. The base drives with EasyNav, and the torso,
head, arm and gripper move under ros2_control.

See :ref:`playgrounds` to install it.

Launching EasyNav
-----------------

Each ``easynav_<config>.launch.yaml`` starts Gazebo, TIAGo, EasyNav with that configuration and
RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_tiago easynav_costmap_rpp.launch.yaml

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Launch file
     - Configuration
   * - ``easynav_costmap_rpp.launch.yaml``
     - Costmap from ``home2.yaml``, costmap AMCL with the base laser, A* and Regulated Pure
       Pursuit.
   * - ``easynav_navmap_bonxai_amcl.launch.yaml``
     - NavMap (``home2_20cm.navmap``) and Bonxai (``house.pcd``) maps, NavMap AMCL against the
       Bonxai map with the base laser, NavMap A* and Regulated Pure Pursuit. The head camera can be
       added as a second sensor in the parameters.

They accept ``params_file``, ``rviz_config``, ``gui`` (``false`` runs Gazebo headless) and
``rviz`` (``false`` skips RViz2).

Notes for your own configurations:

- TIAGo's base laser is 0.095 m above the floor. EasyNav's localizers and obstacle filters drop
  points below ``min_height`` (0.1 m by default) as floor hits, so these configurations set
  ``min_height: 0.05``. In the NavMap obstacles filter, ``min_height`` is the height above the
  NavMap surface.
- The Bonxai map's ``resolution`` must be the one ``house.pcd`` was built with (0.05 m): at the
  default 0.3 m, the laser shares a voxel layer with the floor and the NavMap AMCL sees every ray
  blocked.
- The base takes ``geometry_msgs/TwistStamped``: EasyNav runs with ``use_cmd_vel_stamped: true``,
  and the launch files remap ``cmd_vel_stamped`` to ``/mobile_base_controller/cmd_vel`` and
  ``odom`` to ``/mobile_base_controller/odom``.

Simulation only
---------------

To run Gazebo and TIAGo without EasyNav or RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_tiago_worlds gazebo_sim.launch.yaml

The arm and torso start in PAL's ``home`` posture. The torso, head, arm and gripper have
``joint_trajectory_controller``\ s (``torso_controller``, ``head_controller``,
``arm_controller``, ``gripper_controller``), so they take ``FollowJointTrajectory`` goals or
trajectories on ``<controller>/joint_trajectory``. For example, to look down:

.. code-block:: bash

   ros2 topic pub --once /head_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
     "{joint_names: [head_1_joint, head_2_joint], points: [{positions: [0.0, -0.6], time_from_start: {sec: 1}}]}"

Packages
--------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Package
     - Contents
   * - ``easynav_playground_tiago``
     - EasyNav launch files, parameters and RViz2 configurations
   * - ``easynav_playground_tiago_worlds``
     - The small house world, its maps (costmap, NavMap and Bonxai) and the Gazebo launchers
   * - ``easynav_playground_tiago_description``
     - The TIAGo model: URDF, meshes and ros2_control controllers

The TIAGo model is PAL Robotics' work (Copyright (c) 2022-2024 PAL Robotics S.L., Apache-2.0),
expanded from the Gazebo Harmonic ports of its ``tiago_robot``, ``pmb2_robot`` and
``pal_gripper`` packages. TIAGo is a robot by `PAL Robotics <https://pal-robotics.com/>`_.
