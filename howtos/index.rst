.. _howtos:

===========================
HowTos and Practical Guides
===========================



This section contains a curated collection of **HowTos** and short guides for EasyNavigation (EasyNav).  
Each document provides step‑by‑step instructions to accomplish specific tasks — from setting up costmaps to configuring controllers and running navigation examples.

If you are new to EasyNav, we recommend starting with :doc:`../getting_started/index` and :doc:`../build_install/index` before diving into these guides.

.. contents:: On this page
   :local:
   :depth: 2

📘 Overview
-----------

The HowTos are grouped by category:

- **Simple navigation** – basic examples using the Simple stack.
- **Costmap navigation** – using 2D costmaps for mapping and planning.
- **GridMap navigation** – elevation‑aware mapping and path planning.
- **Controllers** – configuring and tuning controllers for different robots.

Use these guides as templates or quick references while developing with EasyNav.

Simple Stack
-------------

- :doc:`simple_mapping`
- :doc:`simple_navigating`

.. toctree::
   :hidden:

   simple_mapping
   simple_navigating

Costmap Stack
--------------

- :doc:`costmap_mapping`
- :doc:`costmap_navigating`
- :doc:`costmap_navigating_with_icreate`

.. toctree::
   :hidden:

   costmap_mapping
   costmap_navigating
   costmap_navigating_with_icreate

GridMap Stack
--------------

- :doc:`gridmap_mapping`

.. toctree::
   :hidden:

   gridmap_mapping

Controllers
------------

- :doc:`serest_controller`

.. toctree::
   :hidden:

   serest_controller

Behaviors
------------

- :doc:`patrolling_behavior`

.. toctree::
   :hidden:

   patrolling_behavior

.. note::
   Each HowTo is self‑contained and can be executed independently, provided that the EasyNav core and its dependencies are installed.

