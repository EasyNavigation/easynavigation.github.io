.. _software:

EasyNav Software
****************

The EasyNav software is developed under the `EasyNavigation GitHub organization <https://github.com/EasyNavigation>`_, and it follows a **highly modular structure**. This modularity is reflected in the separation of functionality across multiple repositories, each dedicated to a specific role or typology:


C++ API
=======

- `EasyNav Core <https://easynavigation.github.io/EasyNavigation/>`_
- `EasyNav Simple Stack <https://easynavigation.github.io/easynav_simple_stack/>`_
- `EasyNav GridMap Stack <https://easynavigation.github.io/easynav_gridmap_stack/>`_

**HowTo's**:

.. toctree::
   :maxdepth: 1

   ./commanding.rst
   ./blackboard.rst
   ./perceptions.rst


Repositories
============

Core
----

- `EasyNavigation <https://github.com/EasyNavigation/EasyNavigation>`_  
  This is the main repository that contains the core infrastructure of EasyNav, including the lifecycle management, execution engine, and common interfaces.

Plugins
-------

- `easynav_plugins <https://github.com/EasyNavigation/easynav_plugins>`_  
  A collection of lightweight plugins that do not depend on external libraries. It includes generic implementations for controllers, planners, localizers, etc.

Stacks
------

Stacks are collections of plugins that typically share a common representation of the environment and often rely on specific third-party libraries.

- `easynav_simple_stack <https://github.com/EasyNavigation/easynav_simple_stack>`_  
  Plugins based on 2D gridmaps representing free (0) and occupied (1) cells. Designed to serve as an introduction to EasyNav and to validate the core system.

- `easynav_gridmap_stack <https://github.com/EasyNavigation/easynav_gridmap_stack>`_  
  Plugins that use the `GridMap <https://github.com/ANYbotics/grid_map>`_ library to support elevation maps and metric layers.

- `easynav_octomap_stack <https://github.com/EasyNavigation/easynav_octomap_stack>`_  
  Plugins that rely on the `OctoMap <https://github.com/OctoMap>`_ library for 3D occupancy mapping.

- `easynav_pointcloud_stack <https://github.com/EasyNavigation/easynav_pointcloud_stack>`_  
  Plugins that use the `PCL (Point Cloud Library) <https://pointclouds.org/>`_ to process and interpret point cloud data.

Test Cases
----------

These repositories provide maps and configuration files tailored to common application scenarios:

- `easynav_indoor_testcase <https://github.com/EasyNavigation/easynav_indoor_testcase>`_  
  Contains test environments for indoor navigation with flat 2D maps.

- `easynav_outdoor_testcase <https://github.com/EasyNavigation/easynav_outdoor_testcase>`_  
  Focused on outdoor scenarios involving uneven terrain and unstructured environments.

Playgrounds
-----------

Playgrounds define simulation setups for specific robot platforms in various environments.

- `easynav_playground_kobuki <https://github.com/EasyNavigation/easynav_playground_kobuki>`_  
  Simulation models and example scenarios for the Turtlebot2 (Kobuki) robot.

- `easynav_playground_summit <https://github.com/EasyNavigation/easynav_playground_summit>`_  
  Simulation of the SUMMIT XL robot from Robotnik, designed for both indoor and outdoor navigation tasks.