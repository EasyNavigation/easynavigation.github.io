.. _costmap_mapping:

==============================
Mapping with the Costmap Stack
==============================

This HowTo explains how to perform mapping using the **Costmap Stack** in EasyNavigation (EasyNav).  
It builds upon the concepts introduced in the *Simple Stack Mapping* tutorial, but replaces the binary map representation
with a **graded Costmap2D** that encodes traversal costs and supports inflation around obstacles.

If you have not set up EasyNav yet, please complete the steps in :doc:`../build_install/index` first.

.. contents:: On this page
   :local:
   :depth: 2

Setup
------

Before running this tutorial, make sure that:

1. You have a working **EasyNav workspace** (for example `~/ros/ros2/easynav_ws`) built and sourced.
2. Ensure that the following repositories are present inside your ``src/`` folder:

   - ``EasyNavigation``
   - ``easynav_plugins``
   - ``easynav_playground_kobuki`` *(optional, for simulation)*
   - ``easynav_indoor_testcase`` *(optional, for maps and configuration examples)*

3. Your workspace is sourced:

   .. code-block:: bash

      cd ~/ros/ros2/easynav_ws
      source install/setup.bash

You can run this tutorial either in simulation (e.g., Gazebo) or using a static map file.

Overview
--------

The mapping workflow with the **Costmap Stack** mirrors the one used in the *Simple Stack*.

1. **Generate a map** — for example, using SLAM Toolbox. The result should be a **YAML + image** pair (`.yaml` + `.pgm/.png`) defining resolution, origin, and free/occupied thresholds.  
2. **Place the map files** inside a ROS 2 package in your workspace. The **Costmap Maps Manager** will later load it using the `package` and `map_path_file` parameters (or via an absolute path).  
3. **Prepare an EasyNav parameter file** specifying dummy plugins for planner, controller, and localizer, plus the Costmap Maps Manager.  
   This configuration is sufficient to validate that the costmap loads and visualizes correctly.

Example parameters
------------------

Below is a minimal working configuration to verify the Costmap mapping pipeline.  
It mirrors the *Simple Stack* structure but uses the Costmap-based maps manager.  
Replace `my_maps_pkg` and the YAML path with your own package and map.

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [dummy]
        dummy:
          rt_freq: 30.0
          plugin: easynav_controller/DummyController
          cycle_time_nort: 0.01
          cycle_time_rt: 0.001

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [dummy]
        dummy:
          rt_freq: 50.0
          freq: 5.0
          reseed_freq: 0.1
          plugin: easynav_localizer/DummyLocalizer
          cycle_time_nort: 0.01
          cycle_time_rt: 0.001

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [costmap]
        costmap:
          freq: 10.0
          plugin: easynav_costmap_maps_manager/CostmapMapsManager
          package: my_maps_pkg
          map_path_file: maps/office.yaml

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [dummy]
        dummy:
          freq: 1.0
          plugin: easynav_planner/DummyPlanner
          cycle_time_nort: 0.2
          cycle_time_rt: 0.001

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

Running and visualizing
-----------------------

1. Launch your simulator (for example, `easynav_playground_kobuki`) or start a static map server.  
2. Run EasyNav using the parameter file above:

   .. code-block:: bash

      ros2 run easynav_system system_main \
         --ros-args --params-file src/my_maps_pkg/config/costmap_mapping.yaml

3. Open **RViz2** and add an *OccupancyGrid* display for either:

   - ``maps_manager_node/costmap/map`` (static)
   - ``maps_manager_node/costmap/dynamic_map`` (dynamic)

You should see the costmap as a grayscale image, where darker regions correspond to higher traversal cost.

Saving maps
-----------

The **Costmap Maps Manager** reads and writes maps using the same **YAML + image** format as MoveBase and Nav2.  
Therefore, you do not need to rely on internal save paths if your map is produced by an external SLAM node such as SLAM Toolbox.

- **Option A – Using the Maps Manager service:**  
  Call the service ``maps_manager_node/costmap/savemap`` to save the current static map to the configured path.

- **Option B – Using SLAM Toolbox (recommended):**  
  Call the service ``/slam_toolbox/save_map`` (`slam_toolbox/srv/SaveMap`), which writes the map pair in the same format the Costmap Maps Manager can later load via `package` + `map_path_file`.

.. note::

   This tutorial is analogous to :doc:`simple_mapping`, with the only conceptual difference being the internal map representation.  
   The Costmap2D structure encodes graded values (0–255) with inflation, allowing planners and controllers to reason about
   proximity to obstacles instead of using a simple free/occupied binary map.
