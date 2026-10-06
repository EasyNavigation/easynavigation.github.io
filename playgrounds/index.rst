.. _playgrounds:

===========
PlayGrounds
===========

The **PlayGrounds** are complete Gazebo Harmonic simulations integrated with EasyNav: a robot, its
world, the maps and ready-to-run EasyNav configurations. Each one is a single, self-contained
package, and each configuration is a launch file that starts Gazebo, the robot, EasyNav and RViz2.

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
   * - :doc:`omni`
     - Three- to six-wheel omnidirectional robots in two mazes
     - Costmap with RPP

Installation
------------

The PlayGrounds are only distributed as source. Clone the ones you want into the workspace where
you built EasyNav, install their dependencies and build:

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone -b rolling https://github.com/EasyNavigation/easynav_playground_kobuki.git
   git clone -b rolling https://github.com/EasyNavigation/easynav_playground_summit.git
   git clone -b rolling https://github.com/EasyNavigation/easynav_playground_omni.git
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
   omni
