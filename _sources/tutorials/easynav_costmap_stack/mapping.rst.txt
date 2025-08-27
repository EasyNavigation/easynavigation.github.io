.. mapping:


Mapping with the Costmap Stack
------------------------------

The mapping workflow with the ``easynav_costmap_stack`` mirrors the one used in the Simple Stack.
For background and step-by-step context, see :doc:`the Simple Stack Mapping tutorial <../easynav_simple_stack/mapping>`.
The main difference is that here the environment is represented internally as a graded **Costmap2D** instead of a
binary occupancy grid.

Overview
^^^^^^^^

1. **Generate a map** (e.g., using a SLAM solution). The output should be a **YAML + image** pair
   (``.yaml`` + ``.pgm/.png``) describing resolution, origin and free/occupied thresholds.
2. **Place the map files** inside a package in your workspace. The Costmap Maps Manager will later load it
   using the pair of parameters ``package`` and ``map_path_file`` (you can also provide an absolute path).
3. **Prepare an EasyNav parameters file**. In this tutorial we will use *dummy* plugins for controller,
   planner and localizer, and the **Costmap Maps Manager** to load and publish the map. This is enough to
   validate that the costmap is properly loaded and visualized.

Example parameters
^^^^^^^^^^^^^^^^^^

Below is a minimal working configuration you can use to verify mapping. It mirrors the structure of the Simple Stack
example but uses the Costmap-based maps manager. You may replace ``my_maps_pkg`` and the YAML path with your own.

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
^^^^^^^^^^^^^^^^^^^^^^^

1. Launch your simulator (or a static map server) and start EasyNav with the parameter file above.
2. Open ``rviz2`` and add an *OccupancyGrid* display for the topic
   ``maps_manager_node/<plugin_name>/map`` (static) or ``maps_manager_node/<plugin_name>/dynamic_map`` (dynamic).
   You should see the costmap as a grayscale image where darker values denote higher traversal cost.

Saving maps (SLAM Toolbox vs. Maps Manager)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Costmap Maps Manager reads and writes maps using the **same YAML + image format** as MoveBase/Nav2.
That means you do *not* need to rely on the manager's internal save path if you are producing the map with SLAM Toolbox.

- **Option A (Maps Manager service):** The manager exposes a ``savemap`` service at
  ``maps_manager_node/<plugin_name>/savemap`` which saves its current static map to the configured path.
- **Option B (Recommended with SLAM Toolbox):** Call the SLAM Toolbox service
  ``/slam_toolbox/save_map`` (service type ``slam_toolbox/srv/SaveMap``) directly. This will write the
  YAML + image pair in the standard format that the Costmap Maps Manager can later load with
  ``package`` + ``map_path_file``.

.. note::

   This tutorial is analogous to :doc:`../easynav_simple_stack/mapping`. The only conceptual change is the
   internal map representation (``Costmap2D`` with values 0–255 and obstacle inflation support) which allows
   subsequent components (planner/controller) to leverage graded costs instead of a hard free/occupied dichotomy.
