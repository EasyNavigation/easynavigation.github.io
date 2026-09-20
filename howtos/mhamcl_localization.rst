.. _mhamcl_localization:

==========================================================
Global Localization and Kidnapping Recovery with MH-AMCL
==========================================================

This HowTo shows how to use the **Multi-Hypothesis AMCL (MH-AMCL) localizer** in **EasyNavigation (EasyNav)**.
Unlike a regular AMCL, which tracks a single set of particles, MH-AMCL keeps **several hypotheses** about
where the robot is at the same time. This lets the robot:

- **Localize without knowing its initial pose**, since new hypotheses are created wherever the last
  sensor reading fits the map.
- **Recover from wrong estimates and kidnapping**, since a hypothesis that stops explaining what the robot
  sees is discarded and a better one takes over.

It assumes that the environment has already been mapped (see :doc:`costmap_mapping`) and that you can run
the Costmap Stack (see :doc:`costmap_navigating`).

.. contents:: On this page
   :local:
   :depth: 2

How it works
------------

Each hypothesis is an independent particle filter (prediction, correction and reseed). On top of them,
the localizer manages the population:

- **Creation:** periodically, a *cascade map matching* looks in the **whole map** for poses from which the
  last perception could have been obtained. The map is stored in a pyramid of resolutions; the coarsest
  level is scanned entirely and only the promising cells are refined in the finer levels. A new hypothesis
  starts at every candidate that is far enough from the existing ones.
- **Destruction:** a hypothesis is removed if it lies outside the free space of the map or if its quality
  is too low.
- **Merge:** hypotheses that converge to the same pose are merged.
- **Output:** the pose of the hypothesis with the best *quality*. The *quality* is the fraction of the last
  perception that falls on an obstacle of the map. It measures how consistent a hypothesis is with what the
  robot sees, which is more informative than the covariance. Another hypothesis replaces the current one only
  if it is clearly better.

The details are in *Portable Multi-Hypothesis Monte Carlo Localization for Mobile Robots*
(A. García, F. Martín, J. M. Guerrero, F. J. Rodríguez and V. Matellán, ICRA 2023).
The plugin is the EasyNav port of the original `mh_amcl <https://github.com/fmrico/mh_amcl>`_ for Nav2.

---

Setup
-----

Complete the installation steps in :doc:`../build_install/index`. The localizer is in the
``easynav_mhamcl_localizer`` package of ``easynav_plugins``, so it is already built if you cloned that
repository as described in :ref:`build_from_source`. You will also need the Costmap Maps Manager and Planner,
a controller, and the demo repositories, as in :doc:`costmap_navigating`.

---

Configuring the Localizer
-------------------------

Replace the ``localizer_node`` section of a Costmap Stack parameter file with:

.. code-block:: yaml

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [mhamcl]
        mhamcl:
          rt_freq: 50.0     # prediction
          freq: 10.0        # correction
          plugin: easynav_mhamcl_localizer/MHAMCLLocalizer
          initial_pose:
            x: 0.0
            y: 0.1
            yaw: 0.0
            std_dev_xy: 0.1
            std_dev_yaw: 0.01
          max_particles: 200
          min_particles: 30
          noise_translation: 0.05
          noise_rotation: 0.1
          noise_translation_to_rotation: 0.1
          distance_perception_error: 0.05
          reseed_freq: 0.33
          multihypothesis: true
          max_hypotheses: 5
          hypotheses_freq: 0.33

The localizer reads the odometry as a perception, so add an odometry sensor to ``sensors_node``
(if there is none, it reads ``odom -> base_footprint`` from the TF buffer):

.. code-block:: yaml

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1, odom]
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan
        odom:
          topic: odom
          type: nav_msgs/msg/Odometry

A complete, ready-to-use example (with the Regulated Pure Pursuit controller) is shipped in
``easynav_indoor_testcase/robots_params/costmap.rpp.mhamcl.params.yaml``.

.. note::
   The recovery plugins ``AmclConvergenceEvaluator`` and ``AmclRelocalizeMitigation`` belong to
   ``easynav_costmap_localizer``. They are not needed with MH-AMCL, which relocalizes by itself.

---

Running the Simulation
----------------------

1. **Launch the simulator:**

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

2. **Launch EasyNav and RViz2** with the shipped launcher:

   .. code-block:: bash

      ros2 launch easynav_indoor_testcase easynav_costmap_rpp_mhamcl.launch.py

3. In **RViz2**, add a ``MarkerArray`` display on the topic
   ``/localizer_node/mhamcl/hypotheses``. Every hypothesis is drawn with its own color, with an arrow on its
   estimated pose. The arrow of the selected hypothesis is thicker. The particles of the selected hypothesis
   are also published as a ``PoseArray`` on ``/localizer_node/mhamcl/particles``.

---

Testing Global Localization and Kidnapping
------------------------------------------

- **Unknown initial pose:** start the robot far from ``initial_pose``. After a few seconds
  (``1 / hypotheses_freq``), new hypotheses appear where the laser fits the map. When one of them is clearly
  better than the initial one, it becomes the output and the ``map -> odom`` transform jumps to it.
- **Kidnapping:** move the robot in the simulator (or send a wrong pose with the *2D Pose Estimate* tool).
  The current hypothesis loses quality, and the map matching creates a new one at the true pose.
- **Corridors and symmetric places:** several hypotheses may coexist for a while. Move the robot until the
  perception disambiguates them.

Sending a pose to ``initialpose`` (*2D Pose Estimate*) discards all the hypotheses and starts a new one there.

---

Tuning
------

The most relevant parameters are:

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Effect
   * - ``max_hypotheses``
     - ``5``
     - Maximum number of simultaneous hypotheses. More hypotheses cost more CPU.
   * - ``hypotheses_freq``
     - ``0.33``
     - How often the map matching runs and the population is managed (Hz). It runs in a background thread.
   * - ``min_candidate_weight``
     - ``0.5``
     - Minimum match for a map pose to become a hypothesis. Lower it in sparse environments.
   * - ``good_hypo_threshold``
     - ``0.6``
     - Minimum quality for a hypothesis to become the selected one.
   * - ``min_hypo_diff_winner``
     - ``0.2``
     - How much better a hypothesis must be than the selected one to replace it. Low values may produce
       flip-flopping between hypotheses.
   * - ``very_low_q_hypo_threshold``
     - ``0.1``
     - Below this quality a hypothesis is always removed.
   * - ``distance_perception_error``
     - ``0.05``
     - Sensor precision (sigma, in meters).
   * - ``matcher_angle_step``
     - ``0.3927``
     - Angular resolution of the map matching (rad). Increase it on very large maps.

The full list of parameters, topics and NavState keys is in the
`easynav_mhamcl_localizer README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_mhamcl_localizer/README.md>`_.

---

Notes
-----

- The localizer works over the ``map.base`` ``Costmap2D``. Obstacles are the ``LETHAL_OBSTACLE`` cells and
  hypotheses must be in ``FREE_SPACE`` cells.
- Set ``multihypothesis: false`` to use a single hypothesis, like a plain AMCL.
- The map matching is more costly on large, high-resolution maps. If the CPU load is high, lower
  ``hypotheses_freq`` or increase ``matcher_angle_step``.
