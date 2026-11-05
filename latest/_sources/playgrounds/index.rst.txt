.. _playgrounds:

======================
Playgrounds and Robots
======================

EasyNav comes with ready-to-run configurations for simulated robots, the PlayGrounds, and for real
robots.

Playgrounds
-----------

The **PlayGrounds** are complete Gazebo Harmonic simulations integrated with EasyNav: a robot, its
world, the maps and ready-to-run EasyNav configurations. They all live in one repository,
`easynav_playgrounds <https://github.com/EasyNavigation/easynav_playgrounds>`_, and each
configuration is a launch file that starts Gazebo, the robot, EasyNav and RViz2.

Each PlayGround is split in three packages, so that the robot model and the simulation can be used
without EasyNav:

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Package
     - Contents
   * - ``easynav_playground_<robot>_description``
     - The robot model: URDF, meshes and controllers. Neither Gazebo nor EasyNav.
   * - ``easynav_playground_<robot>_worlds``
     - The Gazebo simulation: worlds, their maps, and the launchers that spawn the robot
       (``gazebo_sim.launch.yaml`` starts Gazebo and the robot). No EasyNav.
   * - ``easynav_playground_<robot>``
     - The EasyNav configurations (``params/``) and launchers (``easynav_*.launch.yaml``).

.. note::

   The PlayGrounds need Gazebo Harmonic or newer, so they run on Jazzy and later distributions,
   but not on Humble, whose Gazebo is Fortress. EasyNav itself (core and plugins) does run on
   Humble: only these simulations do not.

They are the reference configurations of EasyNav: the HowTos (:doc:`../howtos/index`) and the
:doc:`../getting_started/index` guide use them, and they are a good starting point for your own
robot's parameter file.

.. list-table::
   :header-rows: 1
   :widths: 22 26 52

   * - PlayGround
     - Robot and world
     - Configurations
   * - :doc:`kobuki` (**indoor reference**)
     - Turtlebot2 (Kobuki) in a small house
     - Costmap (RPP, MPPI, MPC, SeReST, MH-AMCL, routes, safety mode), Simple, mapping,
       multirobot
   * - :doc:`summit` (**outdoor reference**)
     - Robotnik Summit XL in an outdoor excavation and in an indoor warehouse
     - NavMap and Bonxai (RPP, MPPI, MPC), GPS fusion, warehouse logistics
   * - :doc:`tiago`
     - PAL Robotics' TIAGo and TIAGo Pro (holonomic) in a small house
     - Costmap with RPP; NavMap and Bonxai with NavMap AMCL; TIAGo Pro with holonomic MPPI
   * - :doc:`omni`
     - Three- to six-wheel omnidirectional robots in two mazes
     - Costmap with holonomic MPPI, in each heading mode
   * - :doc:`ackermann`
     - Toyota Prius (Ackermann steering) on the Sonoma Raceway
     - NavMap with RPP or Ackermann MPC, GPS fusion
   * - :doc:`aerial`
     - A UFO (x, y, z, yaw) in a town
     - Bonxai 3D planner, aerial MPC, GPS fusion, 3D goals

See :doc:`installation` to install them.

.. toctree::
   :hidden:

   installation
   kobuki
   summit
   tiago
   omni
   ackermann
   aerial

Real robots
-----------

EasyNav runs on real robots with the same parameter files: change the sensor topics, the robot's
geometry and limits, and the map. See:

- :doc:`icreate3` — the configurations and launchers of an iRobot iCreate3 with a 2D lidar.
- :doc:`../howtos/costmap_navigating_with_icreate` — how to deploy EasyNav on that iCreate3.
- :doc:`tiago_robot` — the configurations and launchers of a PAL TIAGo, with EasyNav built on
  ROS 2 Humble.
- :doc:`../migration_guide/index` — moving a robot that runs Nav2 to EasyNav.
- :ref:`realtime_setup` — the real-time limits EasyNav needs on the robot's computer.

.. toctree::
   :hidden:

   icreate3
   tiago_robot
