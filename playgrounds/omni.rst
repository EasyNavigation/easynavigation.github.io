.. _playground_omni:

===============
Omni PlayGround
===============

`easynav_playground_omni <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_omni>`_ simulates
three- to six-wheel **omnidirectional robots** in two mazes, navigating with the Costmap plugins
and the Regulated Pure Pursuit controller.

See :ref:`playgrounds` to install it.

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/8tPknIoeD1M" frameborder="0" allowfullscreen></iframe>
    </div>

Launching EasyNav
-----------------

.. code-block:: bash

   ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml

It starts Gazebo, the ``3w_v2`` robot in ``maze2``, EasyNav and RViz2. Send a goal with the
**2D Goal Pose** tool.

The configuration (``config/easynav_costmap_rpp.params.yaml``) uses the Costmap maps manager, AMCL
over the costmap, A* (``CostmapPlanner``) and Regulated Pure Pursuit. All the robots share EasyNav
limits of 0.6 m/s linear and 0.5 rad/s angular velocity.

Launch arguments
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 30 48

   * - Argument
     - Default
     - Description
   * - ``world``
     - ``maze2``
     - Gazebo world and its EasyNav map: ``maze1`` or ``maze2``
   * - ``robot``
     - ``3w_v2``
     - Robot model: ``3w``, ``3w_v2``, ``4w``, ``5w`` or ``6w``
   * - ``params_file``
     - ``config/easynav_costmap_rpp.params.yaml``
     - EasyNav parameter file
   * - ``rviz_config``
     - ``rviz/easynav_costmap.rviz``
     - RViz2 configuration file
   * - ``map_override_file``
     - ``config/map_overrides/<world>/map.yaml``
     - Parameters that select the map matching ``world``

For example:

.. code-block:: bash

   ros2 launch easynav_playground_omni easynav_navigation_gazebo_sim.launch.yaml robot:=5w world:=maze1

Simulation only
---------------

To run Gazebo and the robot, without EasyNav or RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_omni_worlds gazebo_sim.launch.yaml robot:=4w world:=maze2

It also accepts ``gui:=false`` to run Gazebo headless.

Launch files
------------

``gazebo_sim.launch.yaml``
   The simulation, without EasyNav: ``robot_state_publisher`` with the ``robot`` model, the Gazebo
   server with the ``world`` (``gz sim -r -s``) and its GUI client (if ``gui``), the spawner
   (``ros_gz_sim create``), the ROS–Gazebo bridge (``/clock``, ``/imu``, ``/scan``, camera), the
   ros2_control spawner for the robot's wheel controllers, and ``kinematics``, a node of the
   package that turns ``cmd_vel`` into wheel commands and publishes the odometry.

``easynav_navigation_gazebo_sim.launch.yaml``
   ``gazebo_sim.launch.yaml``, then, after 5 s, EasyNav (``easynav_system system_main``) with
   ``params_file`` and ``map_override_file``, and RViz2 with ``rviz_config``. If EasyNav exits, the
   whole launch ends.
