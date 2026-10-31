.. _playground_omni:

===============
Omni PlayGround
===============

`easynav_playground_omni <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_omni>`_ simulates
three- to six-wheel **omnidirectional robots** in two mazes, navigating with the Costmap plugins
and the MPPI controller as holonomic robots: they also move sideways.

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

The configuration (``params/easynav_costmap_mppi.params.yaml``) uses the Costmap maps manager, AMCL
over the costmap, A* (``CostmapPlanner``) and MPPI with ``motion_model: omni``, which also commands
``linear.y``. All the robots share EasyNav limits of 0.6 m/s in any direction and 0.5 rad/s angular
velocity.

The ``heading_mode`` argument sets the robot's orientation while it moves; it always reaches the
goal's orientation at the end:

.. list-table::
   :header-rows: 1
   :widths: 18 50 32

   * - ``heading_mode``
     - Orientation while moving
     - Launcher
   * - ``path`` (default)
     - Facing along the path; it corrects sideways
     - ``easynav_mppi_path.launch.yaml``
   * - ``goal``
     - The goal's orientation, from the start
     - ``easynav_mppi_goal.launch.yaml``
   * - ``free``
     - Any: the one that costs least
     - ``heading_mode:=free``
   * - ``hold``
     - The robot's when the goal is received
     - ``heading_mode:=hold``
   * - ``fixed``
     - ``heading_yaw`` (1.5708 rad in ``params/heading_modes/fixed.yaml``)
     - ``heading_mode:=fixed``

.. code-block:: bash

   ros2 launch easynav_playground_omni easynav_mppi_goal.launch.yaml world:=maze1

An application can also set the heading while the robot moves, writing ``heading_reference`` (rad,
map frame) in NavState: while present, it overrides ``heading_mode`` (see the MPPI README in
:doc:`../plugins/index`).

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
   * - ``heading_mode``
     - ``path``
     - MPPI orientation while moving: ``path``, ``goal``, ``free``, ``hold`` or ``fixed``
   * - ``params_file``
     - ``params/easynav_costmap_mppi.params.yaml``
     - EasyNav parameter file
   * - ``rviz_config``
     - ``rviz/easynav_costmap.rviz``
     - RViz2 configuration file
   * - ``map_override_file``
     - ``params/map_overrides/<world>/map.yaml``
     - Parameters that select the map matching ``world``
   * - ``gui``
     - ``true``
     - ``false`` runs Gazebo headless
   * - ``rviz``
     - ``true``
     - ``false`` does not start RViz2

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
   ``params_file``, ``map_override_file`` and ``params/heading_modes/<heading_mode>.yaml``, and
   RViz2 with ``rviz_config``. If EasyNav exits, the whole launch ends.

``easynav_mppi_path.launch.yaml``, ``easynav_mppi_goal.launch.yaml``
   ``easynav_navigation_gazebo_sim.launch.yaml`` with ``heading_mode`` ``path`` or ``goal``.
