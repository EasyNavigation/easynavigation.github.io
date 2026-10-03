.. _migration_nav2:

==============================
Migration Guide for Nav2 Users
==============================

You already navigate with Nav2 and want to try EasyNav on the same robot. This guide tells you
what stays the same, what each Nav2 piece becomes in EasyNav, how to write the parameter file and
how your applications send goals. You can follow it in about an hour, and switch back to Nav2 at
any time: EasyNav does not change your robot, your maps or your TF tree.

.. contents:: On this page
   :local:
   :depth: 2

What stays the same
===================

Most of what you built for Nav2 is reused as is:

- **Your robot**: URDF, ``robot_state_publisher``, drivers, the ``odom`` → ``base_footprint``
  transform, and ``/odom``.
- **Your sensors**: the same ``LaserScan`` or ``PointCloud2`` topics.
- **Your maps**: the ``.yaml`` + ``.pgm`` pairs of ``map_server`` and SLAM Toolbox load directly.
- **Your frames**: ``map``, ``odom``, ``base_link``, ``base_footprint`` (REP-105) by default.
- **Your tools**: RViz2, its *2D Pose Estimate* and *2D Goal Pose* buttons, SLAM Toolbox,
  ``use_sim_time`` and your simulator.
- **Your Nav2 clients**, through the Nav2 bridge (see :ref:`migration_goals`).

What changes is how navigation runs: **one process** (``system_main``) with **one parameter file**,
where each component loads the plugin you choose. There is no behavior tree and no lifecycle manager
to configure.

Nav2 and EasyNav, piece by piece
================================

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Nav2
     - EasyNav
   * - ``nav2_bringup``, ``lifecycle_manager`` and one server per function
     - ``system_main``: a single process that configures and activates everything by itself. If
       configuring fails, it says why and exits.
   * - ``bt_navigator`` and behavior trees
     - The ``GoalManager``, inside ``system_node``: it accepts, tracks and finishes goals. There is
       no behavior tree. Mission logic (patrols, tasks) goes in your own node, using a client (see
       :ref:`migration_goals`).
   * - ``map_server``
     - ``maps_manager_node`` with ``CostmapMapsManager``, which loads the same map files.
   * - Global and local costmaps, with layers
     - **One** costmap in ``maps_manager_node``: the static map plus *filters*. ``obstacles``
       (live sensor data) and ``inflation`` (``inflation_radius``, ``cost_scaling_factor``) play
       the role of the obstacle and inflation layers.
   * - ``amcl``
     - ``localizer_node`` with ``easynav_costmap_localizer/AMCLLocalizer``. It publishes
       ``map`` → ``odom`` and listens to ``/initialpose``. ``MHAMCLLocalizer`` adds global
       localization.
   * - ``planner_server`` (NavFn, Smac)
     - ``planner_node`` with ``easynav_costmap_planner/CostmapPlanner`` (A* on the costmap).
   * - ``controller_server`` (RPP, MPPI, DWB)
     - ``controller_node`` with a controller plugin: Regulated Pure Pursuit, MPPI, MPC, SeReST, VFF.
       RPP and MPPI parameters keep Nav2's names wherever they mean the same.
   * - ``velocity_smoother``
     - Built into ``controller_node``: the velocity and acceleration limits (``robot_limits``)
       apply to every command published.
   * - ``collision_monitor``
     - ``CollisionSafetyReflex``, in ``recovery_node``: it brakes before a collision, checked
       every control cycle.
   * - ``behavior_server`` (spin, back up, wait) and recovery subtrees
     - ``recovery_node`` with ``DiagnosticRecoveryManager``: evaluators diagnose problems (robot
       stuck, obstacle too close, localization lost...) and mitigations fix them (advance, retreat,
       relocalize, ask for help, cancel). See :ref:`recovery`.
   * - Costmap ``observation_sources``
     - ``sensors_node``: sensors are declared once, for every component.
   * - ``footprint`` / ``robot_radius``
     - ``system_node.robot_geometry`` (``radius``, ``inscribed_radius``, ``height``), shared by
       every component. Polygon footprints are not supported: use the circumscribed radius.
   * - ``NavigateToPose`` action
     - The ``easynav_control`` topic, through ``GoalManagerClient`` (C++ or Python), or the
       **Nav2 bridge**, which serves ``navigate_to_pose`` on top of EasyNav.
   * - ``NavigateThroughPoses``, ``waypoint_follower``
     - One request with several goals (``GoalManagerClient::send_goals()``).

Step 1: Install EasyNav
=======================

Follow :doc:`../build_install/index`. Besides the core (``ros-<distro>-easynav``), install the
plugins of the configuration in this guide, the closest to a typical Nav2 setup:

.. code-block:: bash

   sudo apt install \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

Some plugins are not packaged for every distribution yet (see the note in
:doc:`../build_install/index`). If one is missing, use Pixi or build ``easynav_plugins`` from
source.

**Let EasyNav run in real time.** Nav2 runs with normal priority, so your system was never asked
for this. EasyNav runs its control cycle with real-time priority (``SCHED_FIFO`` 80), so that a
loaded computer does not delay the commands to the robot. Linux only allows it if your user may use
that priority. Check it:

.. code-block:: bash

   ulimit -r   # must be 80 or more

If it is lower (Ubuntu's default is 0), follow :ref:`realtime_setup`: a one-time change for your
user, a systemd service or a Docker container, then log in again. Without it EasyNav still runs,
with normal priority, and warns at startup: ``Failed to set Real Time (...). Running with normal
priority.``

Step 2: Your map
================

**If you already have a Nav2 map**, use it as it is. Put the ``.yaml`` and the image in a package
of your workspace, e.g. ``my_robot_maps/maps/office.yaml``, or anywhere and give an absolute
path (see Step 3).

**If you need a new one**, map exactly as you would for Nav2: SLAM Toolbox, teleoperation, and
``map_saver`` or ``slam_toolbox/save_map``. :doc:`../howtos/simple_mapping` walks through it
step by step in simulation, and :doc:`../howtos/costmap_mapping` shows how to check the result
with the Costmap maps manager.

Step 3: The parameter file
==========================

A single YAML file configures everything, one section per node. Each node lists the plugin it uses
in ``<kind>_types`` (one entry), and that entry holds the plugin and its parameters. This is a
complete file for a robot with a 2D lidar, Nav2-style: AMCL, A* on a costmap and Regulated Pure
Pursuit. It is ``easynav_indoor_testcase/robots_params/costmap.rpp.params.yaml``, simplified.
The comments say where each value comes from in your Nav2 file.

.. code-block:: yaml

   system_node:
     ros__parameters:
       use_sim_time: true
       # costmap robot_radius / footprint
       robot_geometry:
         radius: 0.3             # circumscribed radius (m)
         inscribed_radius: 0.25  # largest circle inside the robot (m)
         height: 0.5
       # goal_checker xy_goal_tolerance / yaw_goal_tolerance: when a goal is reached
       position_tolerance: 0.3
       angle_tolerance: 0.15
       use_real_time: true       # the default; see "Let EasyNav run in real time" in Step 1
       # amcl base_frame_id, odom_frame_id, global_frame_id (these are the defaults)
       robot_frame: base_link
       odom_frame: odom
       map_frame: map

   sensors_node:
     ros__parameters:
       use_sim_time: true
       # costmap observation_sources, and amcl scan_topic
       sensors: [laser1]
       laser1:
         topic: /scan
         type: sensor_msgs/msg/LaserScan

   maps_manager_node:
     ros__parameters:
       use_sim_time: true
       map_types: [costmap]
       costmap:
         plugin: easynav_costmap_maps_manager/CostmapMapsManager
         freq: 10.0
         # map_server yaml_filename: a package and a path inside it, or an absolute path alone
         package: my_robot_maps
         map_path_file: maps/office.yaml
         # costmap plugins: obstacle_layer and inflation_layer
         filters: [obstacles, inflation]
         obstacles:
           plugin: easynav_costmap_maps_manager/CostmapMapsManager/ObstaclesFilter
         inflation:
           plugin: easynav_costmap_maps_manager/CostmapMapsManager/InflationFilter
           inflation_radius: 0.8
           cost_scaling_factor: 3.0

   localizer_node:
     ros__parameters:
       use_sim_time: true
       localizer_types: [amcl]
       amcl:
         plugin: easynav_costmap_localizer/AMCLLocalizer
         rt_freq: 50.0
         freq: 5.0
         reseed_freq: 1.0
         num_particles: 100        # amcl max_particles
         # amcl set_initial_pose / initial_pose (or use RViz's 2D Pose Estimate)
         initial_pose:
           x: 0.0
           y: 0.0
           yaw: 0.0
           std_dev_xy: 0.1
           std_dev_yaw: 0.01

   planner_node:
     ros__parameters:
       use_sim_time: true
       planner_types: [costmap]
       costmap:
         plugin: easynav_costmap_planner/CostmapPlanner
         freq: 0.5                 # planner_server expected_planner_frequency
         continuous_replan: true   # replan while navigating, as Nav2's default tree does
         cost_factor: 10.0

   controller_node:
     ros__parameters:
       use_sim_time: true
       # velocity_smoother max_velocity / max_accel / max_decel, and RPP desired_linear_vel
       robot_limits:
         max_linear_vel: 0.5
         min_linear_vel: -0.2      # 0: never backwards
         max_angular_vel: 1.0
         max_linear_acc: 1.0
         max_linear_decel: 1.0
         max_angular_acc: 2.0
         max_angular_decel: 2.0
       controller_types: [rpp]
       rpp:
         plugin: easynav_regulated_pp_controller/RegulatedPurePursuitController
         rt_freq: 30.0             # controller_server controller_frequency
         # From here on, the same names as Nav2's RPP
         lookahead_dist: 0.5
         min_lookahead_dist: 0.3
         max_lookahead_dist: 0.9
         lookahead_time: 1.2
         use_velocity_scaled_lookahead_dist: true
         use_rotate_to_heading: true
         rotate_to_heading_min_angle: 0.785
         use_regulated_linear_velocity_scaling: true
         regulated_linear_scaling_min_radius: 0.9
         regulated_linear_scaling_min_speed: 0.15
         allow_reversing: false
         xy_goal_tolerance: 0.1
         yaw_goal_tolerance: 0.105

A few things are worth knowing when you translate your own file:

- **Velocity limits live in one place**, ``controller_node.robot_limits``, not in each controller.
  Every command published respects them, whatever produced it.
- **Frequencies**: ``rt_freq`` is the rate of the part of a component that runs in the real-time
  control cycle (localization prediction, control), and ``freq`` the rate of the rest (map
  updates, planning, AMCL correction).
- **Topics**: velocities go out on ``/cmd_vel`` (``geometry_msgs/Twist``). If your base expects
  ``TwistStamped`` (Nav2's ``enable_stamped_cmd_vel``), set
  ``controller_node.use_cmd_vel_stamped: true``: they then go out on ``/cmd_vel_stamped``, so remap it
  to what your base listens to. AMCL reads odometry from ``/odom``, or from TF with
  ``compute_odom_from_tf: true``.
- **Other controllers**: for MPPI, start from ``costmap.mppi.params.yaml`` in
  `easynav_indoor_testcase <https://github.com/EasyNavigation/easynav_indoor_testcase>`_. Every
  plugin's parameters are in its README (see :doc:`../plugins/index`).
- **Recoveries and collision protection** are optional: without ``recovery_node`` in the file,
  nothing intervenes, like Nav2 without a ``behavior_server``. Once navigation works, copy the
  ``recovery_node`` section of ``costmap.rpp.params.yaml``, which brakes before collisions and
  handles a stuck robot, an obstacle too close or a lost localization (see :ref:`recovery`).

Step 4: Run it
==============

Start your robot or simulator as usual, then EasyNav, instead of ``nav2_bringup``:

.. code-block:: bash

   ros2 run easynav_system system_main --ros-args --params-file my_robot_easynav.yaml

or, from a launch file:

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import Shutdown
   from launch_ros.actions import Node


   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='easynav_system',
               executable='system_main',
               parameters=['/path/to/my_robot_easynav.yaml'],
               output='screen',
               on_exit=Shutdown(reason='EasyNav exited'),
           ),
       ])

In **RViz2**:

- add a *Map* display on ``/maps_manager_node/costmap/map`` (static map) or
  ``/maps_manager_node/costmap/dynamic_map`` (with obstacles and inflation). Set the static map's
  QoS durability to **Transient Local**;
- add a *Path* display on ``/planner_node/costmap/path``;
- give the initial pose with **2D Pose Estimate**, as with Nav2;
- send a goal with **2D Goal Pose**.

To see what is going on inside (the navigation state, each plugin's timing, diagnostics), run the
terminal dashboard in another terminal:

.. code-block:: bash

   ros2 run easynav_tools tui

and see :doc:`../howtos/ros2_easynav_cli` for the ``ros2 easynav`` commands.

.. _migration_goals:

Step 5: Send goals from your applications
=========================================

Choose by what you already have:

- **Code that uses Nav2's action** (the Simple Commander, a BT ``NavigateToPose`` node, the Nav2
  RViz panel, ``ros2 action send_goal``): use the **Nav2 bridge**. You change nothing in them.
- **New code**, or code you are willing to adapt: use **GoalManagerClient**. It is EasyNav's own
  interface, and it also does several goals in one request, pause and resume.

Option A: the Nav2 bridge
-------------------------

`easynav_nav2_bridge <https://github.com/EasyNavigation/easynav_nav2_bridge>`_ is a
``nav2_msgs/action/NavigateToPose`` server on ``navigate_to_pose`` that forwards goals to EasyNav,
and EasyNav's feedback and result back. Any Nav2 client cannot tell it from ``bt_navigator``.

It is built from source, in the workspace where you have EasyNav:

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_nav2_bridge.git
   cd ~/easynav_ws
   rosdep install --from-paths src --ignore-src -y -r
   colcon build --symlink-install --packages-select easynav_nav2_bridge
   source install/setup.bash

Run it next to EasyNav (or add it to your launch file):

.. code-block:: bash

   ros2 run easynav_nav2_bridge nav2_bridge_main

and send goals as you did to Nav2:

.. code-block:: bash

   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}" \
     --feedback

or with the Simple Commander:

.. code-block:: python

   from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

   navigator = BasicNavigator()
   # Not navigator.waitUntilNav2Active(): it waits for Nav2's lifecycle nodes (amcl,
   # bt_navigator), which do not exist with EasyNav.
   navigator.goToPose(goal)   # a PoseStamped in the map frame
   while not navigator.isTaskComplete():
       feedback = navigator.getFeedback()
   if navigator.getResult() == TaskResult.SUCCEEDED:
       print('Goal reached')

What to expect:

- **Feedback**: ``current_pose``, ``navigation_time`` and ``distance_remaining`` (straight-line
  distance to the goal), 10 times per second (parameter ``feedback_rate``).
  ``estimated_time_remaining`` is not computed yet: it stays at zero.
- **Result**: ``SUCCEEDED`` when EasyNav reaches the goal; ``ABORTED`` if it fails (the reason is
  logged by the bridge); ``CANCELED`` when the client cancels.
- **A new goal preempts the current one**, as with ``bt_navigator``.
- Only ``NavigateToPose``: the goal's ``behavior_tree`` field is ignored, and there is no
  ``NavigateThroughPoses`` (use ``GoalManagerClient::send_goals()``).

Option B: GoalManagerClient
---------------------------

``GoalManagerClient`` talks to EasyNav on the ``easynav_control`` topic (see :ref:`commanding`).
It is a small state machine you drive from your node, usually from a timer:

- ``send_goal(pose)`` or ``send_goals(goals)`` (several, in order) start a navigation. Calling
  them again while navigating replaces the goals;
- ``get_state()`` tells how it goes: ``SENT_GOAL``, ``ACCEPTED_AND_NAVIGATING``, and then one of
  ``NAVIGATION_FINISHED``, ``NAVIGATION_FAILED``, ``NAVIGATION_REJECTED``,
  ``NAVIGATION_CANCELLED`` or ``ERROR``;
- ``get_feedback()`` and ``get_result()`` give the details (current pose, navigation time,
  straight-line distance to the goal; the reason of a failure in ``status_message``);
- ``cancel()``, ``pause()`` and ``resume()``;
- after a final state, ``reset()`` before the next goal.

In **Python** (package ``easynav_support_py``):

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import PoseStamped
   from easynav_goalmanager_py import GoalManagerClient, ClientState


   class GoToKitchen(Node):

       def __init__(self):
           super().__init__('go_to_kitchen')
           self.client = GoalManagerClient(self)
           self.sent = False
           self.timer = self.create_timer(0.2, self.cycle)

       def cycle(self):
           state = self.client.get_state()
           if not self.sent:
               goal = PoseStamped()
               goal.header.frame_id = 'map'
               goal.header.stamp = self.get_clock().now().to_msg()
               goal.pose.position.x = 2.0
               goal.pose.orientation.w = 1.0
               self.client.send_goal(goal)
               self.sent = True
           elif state == ClientState.ACCEPTED_AND_NAVIGATING:
               fb = self.client.get_feedback()
               self.get_logger().info(f'{fb.distance_to_goal:.2f} m to go')
           elif state == ClientState.NAVIGATION_FINISHED:
               self.get_logger().info('Arrived')
               self.client.reset()
               self.timer.cancel()
           elif state in (ClientState.NAVIGATION_FAILED, ClientState.NAVIGATION_REJECTED,
                          ClientState.NAVIGATION_CANCELLED, ClientState.ERROR):
               self.get_logger().error(self.client.get_result().status_message)
               self.client.reset()
               self.timer.cancel()


   rclpy.init()
   rclpy.spin(GoToKitchen())

In **C++** (``easynav_system/GoalManagerClient.hpp``), the same, with the states in
``easynav::GoalManagerClient::State``:

.. code-block:: cpp

   #include "easynav_system/GoalManagerClient.hpp"

   // In your node's constructor:
   client_ = easynav::GoalManagerClient::make_shared(shared_from_this());

   // In a timer callback:
   using State = easynav::GoalManagerClient::State;
   switch (client_->get_state()) {
     case State::IDLE:
       client_->send_goal(goal);  // geometry_msgs::msg::PoseStamped, frame "map"
       break;
     case State::NAVIGATION_FINISHED:
     case State::NAVIGATION_FAILED:
     case State::NAVIGATION_REJECTED:
     case State::NAVIGATION_CANCELLED:
     case State::ERROR:
       RCLCPP_INFO(get_logger(), "%s", client_->get_result().status_message.c_str());
       client_->reset();
       break;
     default:
       break;
   }

``shared_from_this()`` is not available in a constructor: create the client in an ``init()``
method called after the node is created, or in the first timer callback. For a full example
with several waypoints, see :doc:`../howtos/patrolling_behavior` (C++ and Python).

Option C: just a pose
---------------------

EasyNav also listens to ``/goal_pose`` (``geometry_msgs/PoseStamped``), which is what RViz's
*2D Goal Pose* publishes. It is the quickest way to try, without feedback or result.

Differences to keep in mind
===========================

- **No behavior trees.** Navigation itself (plan, follow, replan, recover) needs none. What you
  did with a custom tree on top (sequences of goals, tasks between them, conditions) goes in a
  node of yours with ``GoalManagerClient``, or in your own behavior tree with its own action nodes.
- **One costmap.** There is no separate local costmap: the controller and the planner use the
  same map, updated with live sensor data by the ``obstacles`` filter. Controllers such as RPP also
  slow down near obstacles using the sensors directly.
- **Real-time control cycle.** Localization prediction, control and collision checking run in a
  real-time thread, at ``rt_freq``; the rest runs apart, so a slow planner or map update never
  delays control. It needs the system set up for it once (:ref:`realtime_setup`). See
  :doc:`../developer_guide/design`.
- **Stopping is automatic.** If the controller stops producing commands, the robot brakes to zero
  after ``controller_node.cmd_timeout`` (0.5 s); when EasyNav is stopped (Ctrl+C included) it brakes
  within its deceleration limits and leaves a zero command.
- **It may exit by itself.** With the recovery system, a problem it cannot solve (no sensor data, a
  miswired ROS graph) may end EasyNav; the reason is printed when ``system_main`` exits.
- **Safety-rated setups** (safety PLC, safety scanner) have their own page: :ref:`safety`.

Troubleshooting
===============

- **EasyNav exits right after starting**: a parameter is wrong or a plugin is not installed. The
  message just before it exits names it.
- **The robot does not move**: check that every node has the same ``use_sim_time`` as the
  simulator, that your base listens to ``/cmd_vel`` (or ``/cmd_vel_stamped``, see Step 3), and that
  the robot is localized (set the initial pose).
- **"Failed to set Real Time ... Running with normal priority"**: your user may not use
  real-time priority; see :ref:`realtime_setup`. It works anyway, but control may be delayed when
  the computer is loaded.
- **The map does not show up in RViz2**: set the *Map* display's durability to Transient Local.
- **AMCL does not converge**: give the initial pose with *2D Pose Estimate*, or set
  ``initial_pose``; check ``robot_frame`` and the ``odom`` → base transform.
- **A Nav2 client waits forever**: do not wait for Nav2's lifecycle nodes (e.g.
  ``waitUntilNav2Active()``), and check the bridge is running (``ros2 action list`` shows
  ``/navigate_to_pose``).

Next steps
==========

- :doc:`../getting_started/index`: a first run in simulation, if you want to see EasyNav working
  before touching your robot.
- :doc:`../howtos/costmap_navigating`: the same stack, step by step, in simulation.
- :doc:`../plugins/index`: every plugin and its parameters.
- :doc:`../developer_guide/design`: how EasyNav works inside.
