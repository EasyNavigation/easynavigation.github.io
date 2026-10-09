.. _howtos:

===========================
HowTos and Practical Guides
===========================

This section contains a curated collection of **HowTos** and short guides for EasyNavigation (EasyNav).
Each document provides step-by-step instructions to accomplish specific tasks, from mapping to
configuring controllers and running navigation examples.

If you are new to EasyNav, start with :doc:`../build_install/index` and
:doc:`../getting_started/index`. Most HowTos run on the :doc:`../playgrounds/index`.

.. contents:: On this page
   :local:
   :depth: 2

📘 Overview
-----------

The HowTos are grouped by category:

- **Costmap navigation** – 2D costmaps for mapping, localization, planning and routes. The
  reference for indoor robots.
- **NavMap + Bonxai navigation** – 3D surfaces and voxel maps, outdoors and indoors.
- **Simple navigation** – the minimal Simple stack, to learn how EasyNav works.
- **Controllers** – configuring and tuning controllers for different robots.
- **Behaviors** – using EasyNav from any application or behavior.
- **General** – ways to do something independent of a specific stack.
- **Deprecated** – guides for components no longer maintained by EasyNavigation.

Costmap Stack
-------------

- :doc:`costmap_mapping`
- :doc:`costmap_navigating`
- :doc:`mhamcl_localization`
- :doc:`routes_costmap_manager`
- :doc:`costmap_navigating_with_icreate`

.. toctree::
   :hidden:

   costmap_mapping
   costmap_navigating
   mhamcl_localization
   routes_costmap_manager
   costmap_navigating_with_icreate

NavMap + Bonxai Stack
---------------------

- :doc:`navmap_navigating`
- :doc:`bonxai_navmap_from_rosbag`

.. toctree::
   :hidden:

   navmap_navigating
   bonxai_navmap_from_rosbag

Simple Stack
-------------

- :doc:`simple_mapping`
- :doc:`simple_navigating`

.. toctree::
   :hidden:

   simple_mapping
   simple_navigating

Controllers
------------

- :doc:`serest_controller`

.. toctree::
   :hidden:

   serest_controller

Behaviors
---------

- :doc:`patrolling_behavior`

.. toctree::
   :hidden:

   patrolling_behavior

General
-------

- :doc:`costmap_multirobot`
- :doc:`ros2_easynav_cli`
- :doc:`docker_crossdistro`
- :doc:`custom_perception_plugin`

.. toctree::
   :hidden:

   costmap_multirobot
   ros2_easynav_cli
   docker_crossdistro
   custom_perception_plugin

Deprecated
----------

GridMap, OctoMap and LidarSLAM are no longer maintained by EasyNavigation.

- :doc:`gridmap_mapping`
- :doc:`gridmap_navigating`

.. toctree::
   :hidden:

   gridmap_mapping
   gridmap_navigating
