.. _playground_kobuki:

=================
Kobuki PlayGround
=================

`easynav_playground_kobuki <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_kobuki>`_ is
EasyNav's **indoor reference**: a Turtlebot2 (Kobuki) with a 2D lidar in the AWS RoboMaker small
house, with costmap and simple map configurations.

.. image:: ../images/kobuki_sim.png
   :align: center
   :alt: Turtlebot2 simulation in Gazebo

See :ref:`playgrounds` to install it.

Launching EasyNav
-----------------

Each ``easynav_<config>.launch.yaml`` starts Gazebo, the Kobuki, EasyNav with that configuration
and RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

Costmap configurations
~~~~~~~~~~~~~~~~~~~~~~

All of them use the ``maps/home2.yaml`` occupancy map, the Costmap maps manager, AMCL over the
costmap and A* (``CostmapPlanner``). ``easynav_costmap_rpp`` is the recommended one.

.. list-table::
   :header-rows: 1
   :widths: 38 22 20 20

   * - Launch file
     - Controller
     - Localizer
     - Notes
   * - ``easynav_costmap_rpp.launch.yaml``
     - Regulated Pure Pursuit
     - AMCL
     - Tuned for the Kobuki
   * - ``easynav_costmap_rpp_mhamcl.launch.yaml``
     - Regulated Pure Pursuit
     - Multi-hypothesis AMCL
     - See :doc:`../howtos/mhamcl_localization`
   * - ``easynav_costmap_rpp_reflex.launch.yaml``
     - Regulated Pure Pursuit
     - AMCL
     - For trying the collision safety reflex by hand
   * - ``easynav_costmap_rpp_safe.launch.yaml``
     - Regulated Pure Pursuit
     - AMCL
     - Safety mode, see below
   * - ``easynav_costmap_serest.launch.yaml``
     - SeReST
     - AMCL
     -
   * - ``easynav_costmap_mppi.launch.yaml``
     - MPPI
     - AMCL
     -
   * - ``easynav_costmap_mpc.launch.yaml``
     - MPC
     - AMCL
     -
   * - ``easynav_routes.launch.yaml``
     - Regulated Pure Pursuit
     - AMCL
     - Routes from ``maps/routes_1.yaml``, see :doc:`../howtos/routes_costmap_manager`
   * - ``easynav_costmap_mppi_routed.launch.yaml``
     - MPPI
     - AMCL
     - Routes from ``maps/routes_1.yaml``
   * - ``easynav_costmap_mapping.launch.yaml``
     - —
     - —
     - Only the Costmap maps manager, see :doc:`../howtos/costmap_mapping`

Simple map configurations
~~~~~~~~~~~~~~~~~~~~~~~~~

They use the ``maps/home.map`` binary map and the Simple plugins (maps manager, AMCL localizer and
A* planner). The Simple stack is a minimal example, not meant for real use.

.. list-table::
   :header-rows: 1
   :widths: 38 22 40

   * - Launch file
     - Controller
     - Notes
   * - ``easynav_simple.launch.yaml``
     - Simple
     - Used in :doc:`../getting_started/index`
   * - ``easynav_simple_serest.launch.yaml``
     - SeReST
     -
   * - ``easynav_simple_mapping.launch.yaml``
     - —
     - Only the Simple maps manager, see :doc:`../howtos/simple_mapping`

Launch arguments
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Argument
     - Default
     - Description
   * - ``params_file``
     - Per launch file
     - EasyNav parameter file
   * - ``rviz_config``
     - Per launch file
     - RViz2 configuration file
   * - ``gui``
     - ``true``
     - ``false`` runs Gazebo headless
   * - ``rviz``
     - ``true``
     - ``false`` skips RViz2
   * - ``lidar_range``
     - ``3.0``
     - Maximum lidar range, in meters
   * - ``camera``
     - ``false``
     - Enables the RGBD camera

For example, headless with your own parameters:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml \
     gui:=false params_file:=/path/to/my.params.yaml

Safety mode
~~~~~~~~~~~

``easynav_costmap_rpp_safe.launch.yaml`` runs EasyNav in safety mode (see :ref:`safety_mode`) with
the process memory locked. It needs the real-time limits of :ref:`realtime_setup`
(``rtprio`` >= 80 and ``memlock`` unlimited); otherwise EasyNav does not start.

It also publishes an "all clear" safety status on ``/easynav_safety_status``, standing in for a
safety PLC or scanner (see :ref:`safety_channel`). Launch it with ``safety_channel:=false`` to
publish your own status, for example a protective stop.

Multirobot
----------

Two Kobukis, ``r1`` at (0, 0) and ``r2`` at (2, 1), each with its own EasyNav and RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_multirobot.launch.yaml

Each robot uses its own namespace for topics (``/r1/scan_raw``), its own TF topic (``/r1/tf``) and
prefixed frames (``r1/map``, ``r1/base_link``). The parameters are in
``params/costmap_multirobot.params.yaml``, with one section per robot. See
:doc:`../howtos/costmap_multirobot`.

Simulation only
---------------

To run Gazebo and the Kobuki without EasyNav or RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki_worlds gazebo_sim.launch.yaml

or, with two robots, ``multirobot_gazebo_sim.launch.yaml``. To add a robot to a running
simulation, use ``kobuki.launch.yaml`` with a ``namespace`` and a spawn pose (``x``, ``y``,
``Y``...).

The simulated Kobuki publishes ``/scan_raw`` (``sensor_msgs/LaserScan``), ``/odom``,
``/joint_states`` and TF, and listens on ``/cmd_vel``. With ``camera:=true`` it also publishes
``/rgbd_camera/{image,depth_image,points,camera_info}``.

Launch files
------------

The launch files (in ``launch/``, in YAML) build on each other:

.. code-block:: text

   easynav_<config>.launch.yaml            # sets params_file and rviz_config
   └── easynav_gazebo_sim.launch.yaml      # + EasyNav + RViz2
       └── gazebo_sim.launch.yaml          # Gazebo + one Kobuki
           ├── world.launch.yaml
           └── kobuki.launch.yaml

   easynav_multirobot.launch.yaml          # + two EasyNav + two RViz2
   └── multirobot_gazebo_sim.launch.yaml   # Gazebo + two Kobukis
       ├── world.launch.yaml
       └── kobuki.launch.yaml (x2)

What each one starts:

``world.launch.yaml``
   The Gazebo server with the small house world (``gz sim -r -s``), the Gazebo GUI client (if
   ``gui``), and a bridge for ``/clock``. Gazebo is started without a shell, so Ctrl+C stops it
   and no server or GUI is left behind.

``kobuki.launch.yaml``
   One Kobuki: ``robot_state_publisher`` with the robot model (lidar, optional RGBD camera), the
   spawner (``ros_gz_sim create``), and the ROS–Gazebo bridge for its topics (``cmd_vel``,
   ``scan_raw``, ``odom``, ``joint_states``, ``tf``; and the camera's, with ``camera:=true``). With
   ``namespace``, everything goes in that namespace, with its own TF topics and prefixed frames.

``gazebo_sim.launch.yaml``
   ``world.launch.yaml`` and one ``kobuki.launch.yaml``: the simulation, without EasyNav.

``multirobot_gazebo_sim.launch.yaml``
   ``world.launch.yaml`` and two Kobukis: ``kobuki_1`` in namespace ``r1`` at (0, 0) and
   ``kobuki_2`` in ``r2`` at (2, 1).

``easynav_gazebo_sim.launch.yaml``
   ``gazebo_sim.launch.yaml``, then, after 5 s, EasyNav (``easynav_system system_main``) with
   ``params_file``, and RViz2 with ``rviz_config`` (if ``rviz``). If EasyNav exits, the whole launch
   ends.

``easynav_<config>.launch.yaml``
   ``easynav_gazebo_sim.launch.yaml`` with the parameter file and RViz2 configuration of that
   configuration (table below). ``easynav_costmap_rpp_safe`` also publishes the "all clear" safety
   status (``ros2 topic pub`` on ``/easynav_safety_status``, if ``safety_channel``).

``easynav_multirobot.launch.yaml``
   ``multirobot_gazebo_sim.launch.yaml``, then an EasyNav and an RViz2 per robot, in its namespace
   and with its TF topics, all with ``params/costmap_multirobot.params.yaml``.

.. list-table::
   :header-rows: 1
   :widths: 38 34 28

   * - Launch file
     - Parameter file (``params/``)
     - RViz2 configuration (``rviz/``)
   * - ``easynav_costmap_rpp``
     - ``costmap.rpp.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_rpp_mhamcl``
     - ``costmap.rpp.mhamcl.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_rpp_reflex``
     - ``costmap.rpp.reflex.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_rpp_safe``
     - ``costmap.rpp.safe.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_serest``
     - ``costmap.serest.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_mppi``
     - ``costmap.mppi.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_costmap_mpc``
     - ``costmap.mpc.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_routes``
     - ``costmap.rpp.routed.params.yaml``
     - ``easynav_costmap_routed.rviz``
   * - ``easynav_costmap_mppi_routed``
     - ``costmap.mppi.routed.params.yaml``
     - ``easynav_costmap_routed.rviz``
   * - ``easynav_costmap_mapping``
     - ``costmap.mapping.params.yaml``
     - ``easynav_costmap.rviz``
   * - ``easynav_simple``
     - ``simple.params.yaml``
     - ``easynav_simple.rviz``
   * - ``easynav_simple_serest``
     - ``simple.serest.params.yaml``
     - ``easynav_simple.rviz``
   * - ``easynav_simple_mapping``
     - ``simple.mapping.params.yaml``
     - ``easynav_simple.rviz``
   * - ``easynav_multirobot``
     - ``costmap_multirobot.params.yaml``
     - ``easynav_multirobot.rviz``

Package layout
--------------

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Directory
     - Contents
   * - ``launch/``
     - Launch files, in YAML
   * - ``params/``
     - EasyNav parameters, one file per configuration
   * - ``maps/``
     - ``home2.yaml`` (costmap), ``home.map`` (simple map) and ``routes_1.yaml`` (routes)
   * - ``rviz/``
     - RViz2 configurations
   * - ``config/bridge/``
     - ROS–Gazebo bridge topics
   * - ``urdf/``, ``meshes/``
     - Kobuki model
   * - ``worlds/``, ``models/``, ``photos/``
     - Small house world
