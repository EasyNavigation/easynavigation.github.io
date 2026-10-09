.. _playground_summit:

=================
Summit PlayGround
=================

`easynav_playground_summit <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_summit>`_ is
EasyNav's **outdoor reference**: a Robotnik Summit XL with a 3D lidar, a depth camera, an IMU and
a GPS, navigating with NavMap and Bonxai maps in two worlds:

- the **URJC excavation**, an outdoor 3D terrain;
- a **small warehouse**, an indoor logistics scenario with shelves, pallets and clutter.

See :ref:`playgrounds` to install it.

Launching EasyNav
-----------------

Each ``easynav_<config>.launch.yaml`` starts Gazebo, the Summit XL, EasyNav with that configuration
and RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_summit easynav_bonxai_amcl.launch.yaml

Once RViz2 is up, send a goal with the **2D Goal Pose** tool.

Outdoor configurations (excavation)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All of them plan with A* over the NavMap ``maps/excavation_urjc.navmap``.

.. list-table::
   :header-rows: 1
   :widths: 36 20 20 24

   * - Launch file
     - Controller
     - Localizer
     - Notes
   * - ``easynav_bonxai_amcl.launch.yaml``
     - Regulated Pure Pursuit
     - NavMap AMCL
     - AMCL corrects against the Bonxai 3D map (``maps/excavation_urjc.pcd``)
   * - ``easynav_mppi.launch.yaml``
     - MPPI
     - NavMap AMCL
     -
   * - ``easynav_mpc.launch.yaml``
     - MPC
     - NavMap AMCL
     -
   * - ``easynav_gps.launch.yaml``
     - Regulated Pure Pursuit
     - ``FusionLocalizer`` (UKF)
     - Fuses the GPS position, wheel odometry and IMU heading; no Bonxai map
   * - ``easynav_navmap_dummy.launch.yaml``
     - —
     - —
     - Only loads and shows ``maps/excavation_urjc_2.navmap``

In the excavation the terrain is smooth and the Bonxai cloud is sparse, so NavMap AMCL has little
to correct against: it keeps the heading from the IMU, but its position can drift 0.5–1 m from the
true one. The GPS configuration is the accurate one outdoors (about 0.15 m): its ``map`` frame is
the Gazebo world's (``latitude_origin``/``longitude_origin`` are the world's
``spherical_coordinates``).

Warehouse configuration
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ros2 launch easynav_playground_summit easynav_warehouse_amcl.launch.yaml

.. list-table::
   :header-rows: 1
   :widths: 36 20 20 24

   * - Launch file
     - Controller
     - Localizer
     - Maps
   * - ``easynav_warehouse_amcl.launch.yaml``
     - Regulated Pure Pursuit
     - NavMap AMCL
     - Flat NavMap ``maps/warehouse_20cm.navmap``; Bonxai ``maps/warehouse.pcd``

The NavMap is flat, built from a 2D occupancy grid. Its ``obstacles`` filter keeps the static map
and adds the points the sensors see within 3 m and below 1.2 m (``max_range``, ``max_height``);
the ``inflation`` filter (radius 2.0 m, scaling 1.5) keeps paths in the middle of the aisles. The
RViz2 view (``rviz/easynav_warehouse.rviz``) shows the inflated layer. Localization error is about
0.15 m. See :doc:`../howtos/navmap_navigating` for a walk through this configuration.

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

Real-time cycle
~~~~~~~~~~~~~~~

The configurations run EasyNav's real-time cycle (``use_real_time: true``) at 50 Hz
(``system_node.rt_freq``; the default 200 Hz is too tight for this simulation). The recovery
system watches it: if many cycles in a row start late, it holds the mission, asks for human
assistance and finally cancels it. See :ref:`realtime_setup`.

Simulation only
---------------

To run Gazebo and the Summit XL without EasyNav or RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_summit_worlds gazebo_sim.launch.yaml

It loads the excavation; pass ``world:=<path to a .world file>`` for another one, e.g.
``worlds/small_warehouse.world``. To spawn the robot in a running simulation, use
``summit.launch.yaml`` with a spawn pose (``x``, ``y``, ``z``).

The simulated Summit XL publishes:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Topic
     - Type
   * - ``/front_laser/points``
     - ``sensor_msgs/PointCloud2`` (Robosense Helios 16P)
   * - ``/front_camera/depth/color/points``
     - ``sensor_msgs/PointCloud2`` (ZED2)
   * - ``/imu/data``
     - ``sensor_msgs/Imu``
   * - ``/gps/fix``
     - ``sensor_msgs/NavSatFix``
   * - ``/robotnik_base_control/odom``
     - ``nav_msgs/Odometry``
   * - ``/ground_truth``
     - ``nav_msgs/Odometry``

It also publishes TF, and listens on ``/cmd_vel`` (``geometry_msgs/Twist``).

Launch files
------------

The launch files (in ``launch/``, in YAML) build on each other:

.. code-block:: text

   easynav_<config>.launch.yaml            # sets params_file, rviz_config (and world)
   └── easynav_gazebo_sim.launch.yaml      # + EasyNav + RViz2
       └── gazebo_sim.launch.yaml          # Gazebo + the Summit XL
           ├── world.launch.yaml
           └── summit.launch.yaml

What each one starts:

``world.launch.yaml``
   The Gazebo server with ``world`` (``gz sim -r -s``; the excavation by default) and the Gazebo
   GUI client (if ``gui``). Gazebo is started without a shell, so Ctrl+C stops it and no server or
   GUI is left behind.

``summit.launch.yaml``
   The Summit XL: ``robot_state_publisher`` with the robot model, the spawner
   (``ros_gz_sim create``), the ROS–Gazebo bridge (``/clock``, TF, sensors and ground truth), the
   ros2_control spawner for ``joint_state_broadcaster`` and the ``robotnik_base_control`` diff
   drive controller, and ``twist_stamper``, which forwards ``/cmd_vel`` (``Twist``) to the
   controller as ``TwistStamped``.

``gazebo_sim.launch.yaml``
   ``world.launch.yaml`` and ``summit.launch.yaml``: the simulation, without EasyNav.

``easynav_gazebo_sim.launch.yaml``
   ``gazebo_sim.launch.yaml``, then, after 5 s, EasyNav (``easynav_system system_main``) with
   ``params_file``, and RViz2 with ``rviz_config`` (if ``rviz``). If EasyNav exits, the whole launch
   ends.

``easynav_<config>.launch.yaml``
   ``easynav_gazebo_sim.launch.yaml`` with the parameter file, RViz2 configuration and world of
   that configuration:

.. list-table::
   :header-rows: 1
   :widths: 30 26 24 20

   * - Launch file
     - Parameter file (``params/``)
     - RViz2 configuration (``rviz/``)
     - World (``worlds/``)
   * - ``easynav_bonxai_amcl``
     - ``bonxai.amcl.params.yaml``
     - ``easynav_bonxai_amcl.rviz``
     - ``urjc_excavation.world``
   * - ``easynav_mppi``
     - ``mppi.params.yaml``
     - ``easynav_bonxai_amcl.rviz``
     - ``urjc_excavation.world``
   * - ``easynav_mpc``
     - ``mpc.params.yaml``
     - ``easynav_bonxai_amcl.rviz``
     - ``urjc_excavation.world``
   * - ``easynav_gps``
     - ``gps.params.yaml``
     - ``easynav_navmap.rviz``
     - ``urjc_excavation.world``
   * - ``easynav_navmap_dummy``
     - ``navmap.dummy.params.yaml``
     - ``easynav_navmap.rviz``
     - ``urjc_excavation.world``
   * - ``easynav_warehouse_amcl``
     - ``warehouse.amcl.params.yaml``
     - ``easynav_warehouse.rviz``
     - ``small_warehouse.world``

Building maps of a world
------------------------

The warehouse maps were generated from the simulation with NavMap's ``navmap_tools``:

1. ``navmap_map_builder`` builds the 3D cloud (``warehouse.pcd``, for Bonxai). It teleports the
   robot through the free space and puts its sensors' clouds together at the ground-truth poses.
2. ``navmap_map2d_from_pcd`` builds the 2D occupancy grid (``warehouse.pgm``/``.yaml``, for the
   flat NavMap) from that cloud: points between 0.1 and 1.2 m high are obstacles.

.. code-block:: bash

   ros2 launch easynav_playground_summit_worlds gazebo_sim.launch.yaml gui:=false \
     world:=$(ros2 pkg prefix easynav_playground_summit_worlds)/share/easynav_playground_summit_worlds/worlds/small_warehouse.world
   ros2 run navmap_tools navmap_map_builder /tmp/warehouse --world warehouse --model summit_xl \
     --cloud-topic /front_laser/points --ground-truth-topic /ground_truth
   ros2 run navmap_tools navmap_map2d_from_pcd /tmp/warehouse.pcd /tmp/warehouse

Run each tool with ``--help`` for its options. The README of ``easynav_playground_summit_worlds``
explains the whole process.

Package layout
--------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Package and directory
     - Contents
   * - ``easynav_playground_summit``: ``launch/``, ``params/``, ``rviz/``
     - EasyNav launch files, parameters (one file per configuration) and RViz2 configurations
   * - ``easynav_playground_summit_worlds``: ``launch/``, ``config/bridge/``
     - Gazebo launchers (``gazebo_sim``, ``world``, ``summit``) and ROS–Gazebo bridge topics
   * - ``easynav_playground_summit_worlds``: ``maps/``
     - NavMap (``.navmap``), Bonxai (``.pcd``) and 2D occupancy grid (``warehouse.pgm``/``.yaml``)
       maps
   * - ``easynav_playground_summit_worlds``: ``worlds/``, ``models/``
     - URJC excavation and small warehouse worlds
   * - ``easynav_playground_summit_description``: ``urdf/``, ``meshes/``, ``config/``
     - Summit XL model and its ros2_control controllers
