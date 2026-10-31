.. _playground_tiago:

================
TIAGo PlayGround
================

`easynav_playground_tiago <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_tiago>`_
simulates PAL Robotics' **TIAGo** in the AWS RoboMaker small house: its mobile base (pmb2),
lifting torso, pan-tilt head with an RGBD camera, 7-DoF arm with the PAL gripper and wrist
force/torque sensor, base laser, sonars and IMU. The base drives with EasyNav, and the torso,
head, arm and gripper move under ros2_control.

It also simulates **TIAGo Pro**: an omnidirectional base (four mecanum wheels) with two base lasers,
two arms with grippers and a head with an RGBD camera. EasyNav drives it as a holonomic robot, with
MPPI.

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
   * - ``easynav_costmap_mppi_tiago_pro.launch.yaml``
     - **TIAGo Pro**, with the maps, localizer and planner of ``easynav_costmap_rpp`` and its two
       base lasers. MPPI as a holonomic robot (``motion_model: omni``), facing along the path
       (``heading_mode: path``), and checking the base's real shape (``robot_geometry.footprint``)
       as it turns.
   * - ``easynav_costmap_mppi_goal_tiago_pro.launch.yaml``
     - The same, keeping the goal's orientation from the start (``heading_mode: goal``, from
       ``params/overlays/mppi_heading_goal.yaml``).

They accept ``params_file``, ``rviz_config``, ``gui`` (``false`` runs Gazebo headless) and
``rviz`` (``false`` skips RViz2). ``easynav_gazebo_sim.launch.yaml``, which they include, also takes
``robot`` (``tiago`` or ``tiago_pro``) and ``params_overlay``, a parameter file loaded over
``params_file``.

Notes for your own configurations:

- TIAGo's base laser is 0.095 m above the floor. EasyNav's localizers and obstacle filters drop
  points below ``min_height`` (0.1 m by default) as floor hits, so these configurations set
  ``min_height: 0.05``. In the NavMap obstacles filter, ``min_height`` is the height above the
  NavMap surface.
- The Bonxai map's ``resolution`` must be the one ``house.pcd`` was built with (0.05 m): at the
  default 0.3 m, the laser shares a voxel layer with the floor and the NavMap AMCL sees every ray
  blocked.
- TIAGo Pro's two lasers (``/scan_front_raw``, ``/scan_rear_raw``) are 0.21 m above the floor, each
  covering 270° from opposite corners of the base. Its base is 0.72 × 0.50 m: its parameters model
  it as a 0.4 m circle (``radius``) with a 0.3 m ``inscribed_radius``, and give MPPI its rectangle
  (``footprint``).
- Both bases take ``geometry_msgs/TwistStamped``: EasyNav runs with ``use_cmd_vel_stamped: true``,
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

``robot:=tiago_pro`` runs TIAGo Pro instead. Its arms start tucked, in PAL's ``home`` posture. Its
base also takes ``linear.y`` (sideways, positive to the left) on
``/mobile_base_controller/cmd_vel``; with the keyboard (shift moves sideways):

.. code-block:: bash

   ros2 launch easynav_playground_tiago_worlds gazebo_sim.launch.yaml robot:=tiago_pro
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true \
     -r cmd_vel:=/mobile_base_controller/cmd_vel

Its torso, head, arms and grippers have ``joint_trajectory_controller``\ s too: ``torso_controller``,
``head_controller``, ``arm_left_controller``, ``arm_right_controller``,
``gripper_left_controller`` and ``gripper_right_controller``.

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
     - The TIAGo and TIAGo Pro models: URDF, meshes and ros2_control controllers

The TIAGo model is PAL Robotics' work (Copyright (c) 2022-2024 PAL Robotics S.L., Apache-2.0),
expanded from the Gazebo Harmonic ports of its ``tiago_robot``, ``pmb2_robot`` and
``pal_gripper`` packages. The TIAGo Pro model is also PAL Robotics' work (Copyright (c) 2024 PAL
Robotics S.L., Apache-2.0), expanded from its ``tiago_pro_robot``, ``omni_base_robot``,
``tiago_pro_head_robot``, ``pal_sea_arm``, ``pal_pro_gripper`` and ``pal_urdf_utils`` packages, with
the camera frames of RealSense's ``realsense2_description`` (Apache-2.0). TIAGo and TIAGo Pro are
robots by `PAL Robotics <https://pal-robotics.com/>`_.
