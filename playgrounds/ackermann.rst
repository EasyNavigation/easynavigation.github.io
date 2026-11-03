.. _playground_ackermann:

====================
Ackermann PlayGround
====================

`easynav_playground_ackermann <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_ackermann>`_
simulates a **Toyota Prius**, a car with Ackermann steering, on the **Sonoma Raceway**. The car is
localized with GPS, IMU and odometry, plans with A* over a NavMap of the track and follows the path
with RPP or MPC.

See :ref:`playgrounds` to install it.

Launching EasyNav
-----------------

.. code-block:: bash

   ros2 launch easynav_playground_ackermann easynav_rpp.launch.yaml   # Regulated Pure Pursuit
   ros2 launch easynav_playground_ackermann easynav_mpc.launch.yaml   # MPC

Each starts Gazebo, the Prius on the start line facing along the track, EasyNav and RViz2. Send
goals on the track with the **2D Goal Pose** tool.

.. list-table::
   :header-rows: 1
   :widths: 18 32 50

   * - Configuration
     - Parameters
     - Controller
   * - RPP
     - ``params/rpp.params.yaml``
     - Regulated Pure Pursuit
   * - MPC
     - ``params/mpc.params.yaml``
     - MPC with ``motion_model: ackermann``: it keeps the curvature within the minimum turning
       radius (4.2 m) and slows down before curves

Both share the rest of the configuration:

- **Localization:** ``FusionLocalizer`` fuses the GPS, the IMU and the wheel odometry. The fixes are
  turned into map coordinates east-north-up at the world's origin (``projection: enu``, as Gazebo
  defines its ``spherical_coordinates``), and corrected for the GPS antenna's mounting on the car.
  There is no AMCL: a race track has few features for a lidar.
- **Map:** the NavMap of the track (``maps/sonoma_track.navmap`` in
  ``easynav_playground_ackermann_worlds``), with the walls as obstacles and a wide inflation, so
  that the cheapest path runs along the middle of the track.
- **Planner:** A* over that NavMap (``easynav_navmap_planner``).
- **Limits:** the car's limits in ``robot_limits``, with ``min_turning_radius`` for its steering
  (see :ref:`velocity_output`).

Launch arguments
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 30 48

   * - Argument
     - Default
     - Description
   * - ``params_file``
     - ``params/rpp.params.yaml`` (``mpc.params.yaml`` for MPC)
     - EasyNav parameter file
   * - ``rviz_config``
     - ``rviz/easynav_navmap.rviz``
     - RViz2 configuration: the map, the path, the lidar, the camera and a view that follows the car
   * - ``world``
     - ``worlds/sonoma_raceway.sdf``
     - Gazebo world
   * - ``gui``
     - ``true``
     - ``false`` runs Gazebo headless
   * - ``rviz``
     - ``true``
     - ``false`` does not start RViz2

Simulation only
---------------

To run Gazebo and the Prius, without EasyNav or RViz2:

.. code-block:: bash

   ros2 launch easynav_playground_ackermann_worlds gazebo_sim.launch.yaml

The ``x``, ``y``, ``z`` and ``yaw`` arguments set where the car is spawned; ``prius.launch.yaml``
spawns it in a running simulation. The car publishes a 3D lidar (``/top_laser/points``), a front
camera, ``/imu/data``, ``/gps/fix`` and ``/ground_truth``, and listens on ``/cmd_vel``: a
``twist_stamper`` forwards it to the Ackermann steering controller.

The track map is built from the raceway's mesh with the tools in
``easynav_playground_ackermann_worlds`` (see its README). The Sonoma Raceway world is by Juan
Sebastián Cely Gutierrez (`easynav_wheels_worlds <https://github.com/juanscelyg/easynav_wheels_worlds>`_,
GPL-3.0-only), and its model is Open Robotics' in Gazebo Fuel (CC0-1.0).
