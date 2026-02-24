.. _plugins:

================
EasyNav Plugins
================

**EasyNav Plugins** provides the official collection of plugins for the `Easy Navigation (EasyNav) <https://github.com/EasyNavigation>`_ framework.  
These plugins extend the navigation core with planners, controllers, map managers, and localizers compatible with ROS 2.

Each plugin resides in its own ROS 2 package and is registered through ``pluginlib``, enabling dynamic loading at runtime.

.. contents:: On this page
   :local:
   :depth: 2


Supported ROS 2 versions
------------------------

- **ROS 2 kilted** (Ubuntu 24.04, recommended)  
- **ROS 2 rolling**

.. image:: https://img.shields.io/badge/ROS%202-kilted-blue
   :alt: ROS 2 kilted
   :target: #
.. image:: https://img.shields.io/badge/ROS%202-rolling-blue
   :alt: ROS 2 rolling
   :target: #

.. image:: https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml/badge.svg
   :target: https://github.com/EasyNavigation/easynav_plugins/actions/workflows/rolling.yaml
   :alt: rolling CI status

Repository overview
-------------------

This repository groups all the official plugins for EasyNav into four main categories:

1. **Planners** – generate paths from the robot’s current pose to the goal.  
2. **Controllers** – convert paths into motion commands.  
3. **Maps Managers** – manage and update spatial representations of the environment.  
4. **Localizers** – estimate the robot’s pose using map and sensor data.  

Each plugin type implements a well-defined C++ interface and can be configured dynamically in the EasyNav parameter files.

---

🧭 Planners
-----------

Path-planning plugins implementing A*, costmap-based, and NavMap-based methods.

+----------------------------------------+--------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+
| **Package**                            | **Description**                            | **Documentation**                                                                                                                             |
+========================================+============================================+===============================================================================================================================================+
| ``easynav_costmap_planner``            | A* planner over ``Costmap2D``.             | `easynav_costmap_planner README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_costmap_planner/README.md>`_ |
+----------------------------------------+--------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_simple_planner``             | Simple A* planner for ``SimpleMap``.       | `easynav_simple_planner README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_simple_planner/README.md>`_   |
+----------------------------------------+--------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_navmap_planner``             | A* planner operating on a NavMap mesh.     | `easynav_navmap_planner README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/planners/easynav_navmap_planner/README.md>`_   |
+----------------------------------------+--------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------+

---

⚙️ Controllers
--------------

Motion controllers for trajectory tracking and reactive behaviors.

+----------------------------------------+---------------------------------------------------+------------------------------------------------------------------------------------------------------------------------------------------------------+
| **Package**                            | **Description**                                  | **Documentation**                                                                                                                                     |
+========================================+===================================================+======================================================================================================================================================+
| ``easynav_vff_controller``             | Vector Field Force (VFF) reactive controller.     | `easynav_vff_controller README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_vff_controller/README.md>`_       |
+----------------------------------------+---------------------------------------------------+------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_mppi_controller``            | Model Predictive Path Integral (MPPI) controller. | `easynav_mppi_controller README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_mppi_controller/README.md>`_     |
+----------------------------------------+---------------------------------------------------+------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_simple_controller``          | Simple proportional controller for testing.       | `easynav_simple_controller README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_simple_controller/README.md>`_ |
+----------------------------------------+---------------------------------------------------+------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_serest_controller``          | SeReST (Safe Reactive Steering) controller.       | `easynav_serest_controller README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/controllers/easynav_serest_controller/README.md>`_ |
+----------------------------------------+---------------------------------------------------+------------------------------------------------------------------------------------------------------------------------------------------------------+

---

🗺️ Maps Managers
----------------

Map-management plugins that provide, update, and store different environment representations.

+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **Package**                            | **Description**                                       | **Documentation**                                                                                                                                              |
+========================================+=======================================================+================================================================================================================================================================+
| ``easynav_navmap_maps_manager``        | Manages NavMap mesh layers.                           | `easynav_navmap_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_navmap_maps_manager/README.md>`_     |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_bonxai_maps_manager``        | Manages Bonxai probabilistic voxel maps.              | `easynav_bonxai_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_bonxai_maps_manager/README.md>`_     |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_octomap_maps_manager``       | Manages OctoMap 3D occupancy trees.                   | `easynav_octomap_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_octomap_maps_manager/README.md>`_   |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_costmap_maps_manager``       | Manages ``Costmap2D`` layers with filters.            | `easynav_costmap_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_costmap_maps_manager/README.md>`_   |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_routes_maps_manager``        | Manages route segments and route-based map filters.   | `easynav_routes_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_routes_maps_manager/README.md>`_     |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_simple_maps_manager``        | Minimal example map manager for ``SimpleMap``.        | `easynav_simple_maps_manager README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/maps_managers/easynav_simple_maps_manager/README.md>`_     |
+----------------------------------------+-------------------------------------------------------+----------------------------------------------------------------------------------------------------------------------------------------------------------------+

---

📍 Localizers
-------------

Localization plugins based on different map types and sensors.

+----------------------------------------+-----------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------+
| **Package**                            | **Description**                                           | **Documentation**                                                                                                                                   |
+========================================+===========================================================+=====================================================================================================================================================+
| ``easynav_gps_localizer``              | GPS-based localizer for outdoor navigation.               | `easynav_gps_localizer README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_gps_localizer/README.md>`_         |
+----------------------------------------+-----------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_simple_localizer``           | Basic localizer for ``SimpleMap``-based setups.           | `easynav_simple_localizer README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_simple_localizer/README.md>`_   |
+----------------------------------------+-----------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_navmap_localizer``           | AMCL-like localizer operating on NavMap meshes.           | `easynav_navmap_localizer README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_navmap_localizer/README.md>`_   |
+----------------------------------------+-----------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------+
| ``easynav_costmap_localizer``          | AMCL-like localizer using ``Costmap2D``.                  | `easynav_costmap_localizer README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/localizers/easynav_costmap_localizer/README.md>`_ |
+----------------------------------------+-----------------------------------------------------------+-----------------------------------------------------------------------------------------------------------------------------------------------------+

---

License
-------

All packages in this repository are released under **GPL-3.0-only**, unless stated otherwise in their respective package directories.

.. toctree::
   :hidden:
