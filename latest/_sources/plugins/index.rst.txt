.. _plugins:

================
EasyNav Plugins
================

**EasyNav Plugins** (`easynav_plugins <https://github.com/EasyNavigation/easynav_plugins>`_)
provides the official collection of plugins for the `Easy Navigation (EasyNav)
<https://github.com/EasyNavigation>`_ framework. These plugins extend the navigation core with
planners, controllers, map managers, localizers and recovery systems.

Each plugin resides in its own ROS 2 package and is registered through ``pluginlib``, enabling
dynamic loading at runtime. To list the plugins installed in your system:

.. code-block:: bash

   ros2 easynav plugins

.. contents:: On this page
   :local:
   :depth: 2


Supported ROS 2 versions
------------------------

- **ROS 2 rolling**
- **ROS 2 lyrical**
- **ROS 2 kilted**
- **ROS 2 jazzy**

See :doc:`../build_install/index` for the install methods per distro, and
:ref:`package_availability` for the plugins that are not in the binary packages yet.

.. image:: https://img.shields.io/badge/ROS%202-rolling-blue
   :alt: ROS 2 rolling
   :target: #
.. image:: https://img.shields.io/badge/ROS%202-lyrical-blue
   :alt: ROS 2 lyrical
   :target: #
.. image:: https://img.shields.io/badge/ROS%202-kilted-blue
   :alt: ROS 2 kilted
   :target: #
.. image:: https://img.shields.io/badge/ROS%202-jazzy-blue
   :alt: ROS 2 jazzy
   :target: #

.. image:: https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml/badge.svg
   :target: https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml
   :alt: rolling CI status

Repository overview
-------------------

This repository groups all the official plugins for EasyNav into five main categories:

1. **Planners** – generate paths from the robot’s current pose to the goal.
2. **Controllers** – convert paths into motion commands.
3. **Maps Managers** – manage and update spatial representations of the environment.
4. **Localizers** – estimate the robot’s pose using map and sensor data.
5. **Recoveries** – detect navigation problems and handle them.

Each plugin type implements a well-defined C++ interface and can be configured in the EasyNav
parameter file. The plugins are grouped in **stacks** by the map representation they work on:
*Simple*, *Costmap*, and *NavMap* + *Bonxai*. The :doc:`../playgrounds/index` have
ready-to-run configurations for each of them.

.. |readme| replace:: README

🧭 Planners
-----------

.. list-table::
   :header-rows: 1
   :widths: 32 48 20

   * - Package
     - Description
     - Documentation
   * - ``easynav_costmap_planner``
     - A* planner over ``Costmap2D``.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_costmap_planner/README.md>`__
   * - ``easynav_navmap_planner``
     - A* planner over a NavMap mesh.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_navmap_planner/README.md>`__
   * - ``easynav_simple_planner``
     - Simple A* planner for ``SimpleMap``.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_simple_planner/README.md>`__

⚙️ Controllers
--------------

.. list-table::
   :header-rows: 1
   :widths: 32 48 20

   * - Package
     - Description
     - Documentation
   * - ``easynav_regulated_pp_controller``
     - Regulated Pure Pursuit, a port of Nav2's, with optional Dynamic Window (DWPP) extension.
       The recommended controller for differential robots.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_regulated_pp_controller/README.md>`__
   * - ``easynav_mppi_controller``
     - Model Predictive Path Integral (MPPI) controller.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_mppi_controller/README.md>`__
   * - ``easynav_mpc_controller``
     - Model Predictive Controller (MPC) for trajectory tracking.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_mpc_controller/README.md>`__
   * - ``easynav_serest_controller``
     - SeReST (Smooth Error-Responsive Speed and Turning) path tracking controller.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_serest_controller/README.md>`__
   * - ``easynav_vff_controller``
     - VFF reactive obstacle avoidance controller.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_vff_controller/README.md>`__
   * - ``easynav_simple_controller``
     - Simple PID path follower, for testing.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_simple_controller/README.md>`__

🗺️ Maps Managers
----------------

.. list-table::
   :header-rows: 1
   :widths: 32 48 20

   * - Package
     - Description
     - Documentation
   * - ``easynav_costmap_maps_manager``
     - ``Costmap2D`` maps (YAML + image, as Nav2), with obstacle and inflation filters.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_costmap_maps_manager/README.md>`__
   * - ``easynav_navmap_maps_manager``
     - NavMap triangulated 3D surfaces, with obstacle and inflation filters.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_navmap_maps_manager/README.md>`__
   * - ``easynav_bonxai_maps_manager``
     - Bonxai probabilistic 3D voxel maps.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_bonxai_maps_manager/README.md>`__
   * - ``easynav_routes_maps_manager``
     - Navigation routes, editable in RViz2, and a filter that keeps costmap paths on them.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_routes_maps_manager/README.md>`__
   * - ``easynav_simple_maps_manager``
     - Minimal example map manager for ``SimpleMap`` (binary occupancy).
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_simple_maps_manager/README.md>`__
   * - ``easynav_octomap_maps_manager``
     - **Deprecated**: no longer maintained by EasyNavigation. OctoMap 3D occupancy trees.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_octomap_maps_manager/README.md>`__

📍 Localizers
-------------

.. list-table::
   :header-rows: 1
   :widths: 32 48 20

   * - Package
     - Description
     - Documentation
   * - ``easynav_costmap_localizer``
     - AMCL over ``Costmap2D``. Also provides the ``AmclConvergenceEvaluator`` and
       ``AmclRelocalizeMitigation`` recovery plugins.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_costmap_localizer/README.md>`__
   * - ``easynav_mhamcl_localizer``
     - Multi-Hypothesis AMCL over ``Costmap2D``: global localization and kidnapping recovery.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_mhamcl_localizer/README.md>`__
   * - ``easynav_navmap_localizer``
     - AMCL for NavMap stacks, scoring 3D point clouds against a Bonxai map.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_navmap_localizer/README.md>`__
   * - ``easynav_fusion_localizer``
     - Multi-sensor fusion (UKF, based on ``robot_localization``): GPS, odometry, IMU.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_fusion_localizer/README.md>`__
   * - ``easynav_simple_localizer``
     - AMCL over ``SimpleMap``.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_simple_localizer/README.md>`__

🛟 Recoveries
-------------

Recovery systems, loaded by ``recovery_node`` (see :ref:`recovery`). ``easynav_diagnostic_recovery``
is itself made of plugins (safety reflexes, evaluators and mitigations), documented in its README.

.. list-table::
   :header-rows: 1
   :widths: 32 48 20

   * - Package
     - Description
     - Documentation
   * - ``easynav_diagnostic_recovery``
     - Diagnosis-driven recovery system: safety reflexes, evaluators and mitigations.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_diagnostic_recovery/README.md>`__
   * - ``easynav_simple_recovery``
     - Simple recovery system, written as a tutorial.
     - `README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_simple_recovery/README.md>`__

Other plugins
-------------

Outside ``easynav_plugins``:

- `easynav_alt_imu_sensor <https://github.com/EasyNavigation/easynav_alt_imu_sensor>`_: an example
  perception plugin, see :doc:`../howtos/custom_perception_plugin`.
- `easynav_gridmap_stack <https://github.com/EasyNavigation/easynav_gridmap_stack>`_ and
  `easynav_lidarslam_ros2 <https://github.com/EasyNavigation/easynav_lidarslam_ros2>`_:
  **deprecated**, no longer maintained by EasyNavigation.

License
-------

All packages in this repository are released under **Apache-2.0**, unless stated otherwise in their respective package directories.

.. toctree::
   :hidden:
