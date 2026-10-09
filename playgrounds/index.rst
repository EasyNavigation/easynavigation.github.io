.. _playgrounds:

===========
PlayGrounds
===========

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
     - PAL Robotics' TIAGo in a small house
     - Costmap with RPP; NavMap and Bonxai with NavMap AMCL
   * - :doc:`omni`
     - Three- to six-wheel omnidirectional robots in two mazes
     - Costmap with RPP

Installation
------------

Install the PlayGround you want with APT in jazzy, kilted or lyrical, or with
:ref:`Pixi <install_pixi>` (also rolling). They are not available for humble. Each one brings its robot, worlds and EasyNav launchers:

.. code-block:: bash

   sudo apt install ros-<distro>-easynav-playground-kobuki

To modify them, build them from source instead: clone the repository into your workspace, install
their dependencies and build them (all, or ``--packages-up-to`` the one you want):

.. only:: not latest

   .. code-block:: bash

      cd ~/easynav_ws/src
      git clone -b 0.5.0 https://github.com/EasyNavigation/easynav_playgrounds.git
      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      source install/setup.bash

.. only:: latest

   .. code-block:: bash

      cd ~/easynav_ws/src
      git clone -b rolling https://github.com/EasyNavigation/easynav_playgrounds.git
      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      source install/setup.bash

Every PlayGround's launch files accept, at least, ``params_file`` (EasyNav parameter file) and
``rviz_config`` (RViz2 configuration), so you can try your own parameters on a PlayGround robot:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml \
     params_file:=/path/to/my.params.yaml

Real robots
-----------

EasyNav runs on real robots with the same parameter files: change the sensor topics, the robot's
geometry and limits, and the map. See:

- :doc:`../howtos/costmap_navigating_with_icreate` — an iRobot iCreate3 with a 2D lidar.
- :doc:`../migration_guide/index` — moving a robot that runs Nav2 to EasyNav.
- :ref:`realtime_setup` — the real-time limits EasyNav needs on the robot's computer.

.. toctree::
   :hidden:

   kobuki
   summit
   tiago
   omni
