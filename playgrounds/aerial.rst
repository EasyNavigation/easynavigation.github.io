.. _playground_aerial:

=================
Aerial PlayGround
=================

`easynav_playground_aerial <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_aerial>`_
simulates an **aerial robot**, a 1 m flying saucer (the UFO), in a **town**. The UFO moves in x, y,
z and yaw. It plans 3D paths over a Bonxai map of the town, keeping its distance from buildings and
the ground, and follows them with MPC.

See :ref:`playgrounds` to install it.

Launching EasyNav
-----------------

.. code-block:: bash

   ros2 launch easynav_playground_aerial easynav_mpc.launch.yaml

It starts Gazebo, the UFO hovering at 6 m in a street of the town, EasyNav and RViz2.

Goals in the air
~~~~~~~~~~~~~~~~

RViz2's **2D Goal Pose** puts goals at z = 0, on the ground. To send goals in the air, use the
**3D Goal Pose** tool of ``navmap_rviz_plugin`` (shortcut ``h``), already in the toolbar of this
PlayGround's RViz2 configuration:

- a left click places the goal on the line from the camera through the cursor, 10 m away by
  default;
- the mouse wheel changes that distance;
- dragging sets the goal's yaw;
- a line down to the ground, and a cross where it falls, show where the goal is.

The tool can be added to any RViz2 configuration, for any robot that takes goals above the ground.

Configuration
~~~~~~~~~~~~~

``params/mpc.params.yaml``:

- **Localization:** ``FusionLocalizer`` in 3D, fusing the GPS (east-north-up at the world's
  origin, ``projection: enu``), the IMU and the odometry.
- **Map:** the Bonxai map of the town (``maps/nathan_benderson_town.pcd`` in
  ``easynav_playground_aerial_worlds``), at 0.5 m.
- **Planner:** ``easynav_bonxai_planner``. It treats buildings as solid blocks down to the ground
  (``fill_below: highest``), keeps at least 2 m from them and prefers 5 m, and flies no higher than
  70 m.
- **Controller:** MPC with ``motion_model: aerial``. It follows the 3D path within the UFO's
  horizontal and vertical limits (``robot_limits.max_vertical_vel`` and the rest, see
  :ref:`velocity_output`).

The launcher accepts ``params_file``, ``gui`` (``false`` runs Gazebo headless) and ``rviz``
(``false`` does not start RViz2).

Simulation only
---------------

To fly the UFO by hand, with Gazebo and RViz2 but without EasyNav:

.. code-block:: bash

   ros2 launch easynav_playground_aerial ufo_sim.launch.yaml

Without RViz2 either:

.. code-block:: bash

   ros2 launch easynav_playground_aerial_worlds gazebo_sim.launch.yaml

The ``x``, ``y``, ``z`` and ``yaw`` arguments set where the UFO is spawned; ``ufo.launch.yaml``
spawns it in a running simulation. It publishes two 3D lidars (``/top_laser/points``,
``/bottom_laser/points``), a front camera, ``/imu/data``, ``/gps/fix`` and ``/odom``, and listens on
``/cmd_vel`` (body frame: linear x, y, z and angular z).

The UFO has no gravity: a flight controller in ``easynav_playground_aerial_worlds`` turns the
command into its velocity as a body with inertia would (first-order response, within its velocity,
acceleration and deceleration limits), and brakes to hover without commands for 0.5 s.

The town world is Juan Sebastián Cely Gutierrez's
`disaster_world <https://github.com/juanscelyg/disaster_world>`_ (Apache-2.0), cut down to the
ground, the roads and the buildings. How the world and the map were built is in the
``easynav_playground_aerial_worlds`` README.
