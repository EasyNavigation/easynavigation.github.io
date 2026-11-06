.. _playground_marine:

=================
Marine PlayGround
=================

`easynav_playground_marine <https://github.com/EasyNavigation/easynav_playgrounds/tree/rolling/playground_marine>`_
simulates **marine vehicles** in a **synthetic sea**: a surface boat, the WAM-V catamaran, and a
submarine, the MBARI Tethys LRAUV. Each one carries the sensors of its kind. EasyNav drives the
submarine: it plans 3D paths under the surface and over the sea floor on a Bonxai map, localizes it
by dead reckoning as an AUV does, and follows the paths with MPC.

See :ref:`playgrounds_install` to install it. The vehicles are downloaded from Gazebo Fuel the
first time (internet).

.. raw:: html

    <div align="center">
      <iframe width="560" height="315" src="https://www.youtube.com/embed/7PKJ5NL_lLQ" frameborder="0" allowfullscreen></iframe>
    </div>

The world
---------

The synthetic sea is generated offline, the same every time: 1 km x 1 km, with z = 0 at sea level.
It has a beach to the west and a floor that deepens to the east (21 m at the origin, 48 m at the
edge), an islet, a shoal 4 m under the surface, a seamount and a ridge. Optionally, waves move the
vehicle and are drawn on the water (``waves:=true``).

Navigating with the submarine
-----------------------------

.. code-block:: bash

   ros2 launch easynav_playground_marine easynav_submarine_mpc.launch.yaml

It starts Gazebo, the Tethys at 3 m deep, EasyNav and RViz2. RViz2 shows the Bonxai map of the sea
floor, the planned path, the MPC's prediction, the estimated and the true pose, the goal and the
sensors: the imaging sonar (its fan image and its echoes), the DVL and the depth gauge.

Goals under the water
~~~~~~~~~~~~~~~~~~~~~

Goals go under the surface: use the **3D Goal Pose** tool of ``navmap_rviz_plugin`` (shortcut
``h``, see :ref:`playground_aerial` for how it works), already in the toolbar of this PlayGround's
RViz2 configuration, or publish them:

.. code-block:: bash

   ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
     "{header: {frame_id: map}, pose: {position: {x: 60.0, y: 0.0, z: -6.0}, orientation: {w: 1.0}}}"

Configuration
~~~~~~~~~~~~~

``params/submarine_mpc.params.yaml``:

- **Localization:** ``FusionLocalizer``, dead reckoning. The local filter (``odom -> base_footprint``)
  fuses the IMU and the DVL's velocity over the floor; the global one (``map -> odom``) adds the depth
  gauge and the GNSS fix, which is only available at the surface. Both start where the launcher puts
  the vehicle, (0, 0) at 3 m deep.
- **Map:** the Bonxai map of the sea floor (``maps/synthetic_sea.pcd`` in
  ``easynav_playground_marine_worlds``, a point every 2 m), at 1 m.
- **Planner:** ``easynav_bonxai_planner`` with 2 m cells. The floor is solid (``fill_below: lowest``),
  the vehicle stays under the surface (``max_altitude: -1``) and at least 2 m from the relief, and
  prefers 5 m.
- **Controller:** MPC with ``motion_model: aerial``, forward only and with no lateral motion, within
  the Tethys' speed (1.5 m/s), vertical speed and yaw rate (0.05 rad/s), with the path's corners
  rounded to its turning circle.

The Tethys is a torpedo: it turns and dives only while it moves forward, with a turning radius of
some 30 m. When the MPC asks it to turn at a standstill, to face a goal behind it, its
autopilot does the turn on the move. With no thrust at the goal, it coasts on some 10-30 m.

The launcher accepts ``params_file``, ``gui`` (``false`` runs Gazebo headless), ``rviz`` (``false``
does not start RViz2) and ``waves``.

Sensors
-------

The place, rate and noise of the sensors are in one table (``tools/marine_sensors.py`` in
``easynav_playground_marine_worlds``), from which the launcher makes the SDF, the bridge and the
static transforms.

.. list-table::
   :header-rows: 1
   :widths: 18 82

   * - Vehicle
     - Sensors (topics)
   * - Boat (WAM-V)
     - IMU (``/imu/data``), magnetometer (``/imu/mag``), GNSS (``/gps/fix``), 3D lidar on a mast
       (``/lidar/points``, only what is above the water), front camera (``/camera/image_raw``),
       echosounder (``/echosounder``)
   * - Submarine (Tethys)
     - IMU (``/imu/data``), magnetometer (``/imu/mag``), depth gauge (``/pressure``, ``/depth``),
       DVL (``/dvl/data``, ``/dvl/twist``, ``/dvl/altitude``), GNSS (``/gps/fix``, a fix only at the
       surface), forward-looking imaging sonar (``/sonar/image``, ``/sonar/image_fan``,
       ``/sonar/points``)

Gazebo has no acoustic sensor: the imaging sonar is a fan of rays whose hits a node turns into the
echoes of a multibeam sonar (backscatter, beam pattern, absorption, shadows, speckle). It has the
look and the geometry of a sonar image, not a physical simulation of sound. The true pose of the
vehicle is ``/ground_truth/odom``.

Simulation only
---------------

To drive the vehicles by hand, with Gazebo and RViz2 showing their sensors but without EasyNav:

.. code-block:: bash

   ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=boat
   ros2 launch easynav_playground_marine easynav_marine.launch.yaml robot:=submarine

Both listen on ``/cmd_vel``: ``linear.x`` forward, ``angular.z`` yaw rate and, for the submarine,
``linear.z`` up. A command older than a second stops the engines.

.. code-block:: bash

   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.5, z: -0.2}, angular: {z: 0.1}}"

Other arguments are ``altitude`` (the submarine's depth), ``radius`` (the size of the sea), ``waves``
and its ``wave_height``, ``wave_period`` and ``wave_direction``. The README of the PlayGround
explains how the waves move each vehicle.

The WAM-V (Carlos Agüero and Cole Biesemeyer, Open Robotics) and the MBARI Tethys LRAUV (Mabel
Zhang) come from Gazebo Fuel, under CC-BY-4.0.
