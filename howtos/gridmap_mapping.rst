.. _gridmap_mapping:

==========================================
GridMap Mapping with LidarSLAM and EasyNav
==========================================

This HowTo explains how to build a **multi-layer GridMap** from a **LidarSLAM** point cloud and save it
to disk (YAML + one PGM per layer) so it can be loaded later by the **Gridmap Maps Manager** and the planner.

.. warning::
   **Deprecated / unmaintained.** The GridMap stack is not part of EasyNav's regularly
   tested/maintained plugins and is currently **not known to build** on ROS 2 Rolling
   (it fails against current toolchains in several of its dependencies — Eigen API
   changes in ``grid_map_core``, the ``ament_target_dependencies`` CMake macro removed
   from ``grid_map_ros``, and a third-party LidarSLAM dependency pinned to an old
   branch that hits the same issue). This page is kept only as a historical reference
   of the intended workflow; expect to do real fixing work before it runs. If you want
   to pick this up, the relevant repositories are:

   - `EasyNavigation/easynav_gridmap_stack <https://github.com/EasyNavigation/easynav_gridmap_stack>`_
     — the ``GridmapMapsManager``, ``GridMapAStarPlanner`` and ``GridMapRRTStarPlanner`` plugins.
   - `ANYbotics/grid_map <https://github.com/ANYbotics/grid_map>`_ — the underlying GridMap
     library (``rolling`` branch does not currently build; a partial rolling-compatibility
     fix exists at `fmrico/grid_map <https://github.com/fmrico/grid_map>`_, untested beyond
     compiling).
   - `EasyNavigation/easynav_lidarslam_ros2 <https://github.com/EasyNavigation/easynav_lidarslam_ros2>`_
     — the LidarSLAM localizer/mapping nodes used below; excluded from the default build
     (``COLCON_IGNORE``) and depends on the third-party submodule
     `rsasaki0109/ndt_omp_ros2 <https://github.com/rsasaki0109/ndt_omp_ros2>`_, pinned to
     its ``humble`` branch, which also needs a rolling-compatibility fix.
   - `EasyNavigation/easynav_outdoor_testcase <https://github.com/EasyNavigation/easynav_outdoor_testcase>`_
     — example maps/params for this stack. Note ``robots_params/maps_builder.params.yaml``
     uses a stale node name (``pointcloud_maps_builder_node`` instead of the current
     ``gridmap_maps_builder_node``), and ``launch/summit.launch.py`` /
     ``robots_params/maps_manager.params.yaml`` reference a
     ``easynav_pointcloud_maps_manager/PointCloudMapsManager`` plugin that does not exist
     anywhere in this ecosystem — ``robots_params/summit_gridmap_ls_params.yaml`` is the
     one example file in that repo that actually matches current plugin names.

.. contents:: On this page
   :local:
   :depth: 2

Setup
------

Before starting, ensure that:

1. You have completed the installation described in :doc:`../build_install/index`.  
2. You have a working workspace containing the following repositories:

   - ``EasyNavigation``
   - ``easynav_plugins``
   - ``easynav_gridmap_stack`` *(for GridMap representation and planner)*
   - ``easynav_lidarslam_ros2`` *(for SLAM)*
   - ``easynav_playground_summit`` *(for the Summit simulation world)*

.. warning::

   The ``easynav_gridmap_stack`` [repository](https://github.com/EasyNavigation/easynav_gridmap_stack.git) **is not included** in the default EasyNav build or in
   ``easynav_plugins``.  
   You must clone it manually into your workspace as shown above before building.  
   It provides the GridMap representation, its Maps Manager, and the corresponding planner.

3. All packages build successfully and are sourced:

   .. code-block:: bash

      cd ~/ros/ros2/easynav_ws
      colcon build --symlink-install
      source install/setup.bash

4. **RViz2** is installed for visualization.  
5. Simulation nodes use ``use_sim_time: true``.

---

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/mZR1CqNV5aU" frameborder="0" allowfullscreen></iframe>
    </div>

The workflow consists of:

1. Launching the **Summit playground** simulation.  
2. Running **LidarSLAM** to build a 3D point cloud of the environment.  
3. Converting the resulting map point cloud into a **GridMap** using the **GridMap Builder**.  
4. Feeding the built GridMap to the **Gridmap Maps Manager**.  
5. Saving the map to disk for later use.

---

1. Start the Simulator
----------------------

Start the Summit world simulation in Gazebo with RViz configured automatically.

.. code-block:: bash

   ros2 launch easynav_playground_summit playground_summit.launch.py

Keep the RViz window open to visualize topics such as the LIDAR scan and the map.

---

2. Launch LidarSLAM
-------------------

Start the LidarSLAM pipeline (scan matcher + graph-based SLAM).  
If your package provides a consolidated launch file, you can start it directly:

.. code-block:: bash

   ros2 launch lidarslam lidarslam.launch.py

You can close any *extra* RViz instance this launch may open; we will use the one started in step (1).

As the robot moves, LidarSLAM will publish a **map point cloud**:

- **Topic:** ``/map``  
- **Type:** ``sensor_msgs/msg/PointCloud2``  
- **Frame:** ``map``

In RViz, add a *PointCloud2* display for ``/map`` and increase the point size to improve visibility.

---

3. Build the Map with Teleoperation
-----------------------------------

Drive the robot around the environment to accumulate the map:

.. code-block:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard

Continue until the environment is sufficiently covered.

---

4. Run the GridMap Builder (PointCloud → GridMap)
-------------------------------------------------

The **GridMap Builder** converts the accumulated ``/map`` point cloud into a multi-layer GridMap.  
The example parameters file listens to ``/map`` and downsamples the cloud before rasterization.

Run the builder:

.. code-block:: bash

   ros2 run easynav_gridmap_maps_manager gridmap_maps_builder_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_outdoor_testcase/robots_params/maps_builder.params.yaml

Example configuration:

.. code-block:: yaml

    gridmap_maps_builder_node:
      ros__parameters:
        use_sim_time: true
        sensors: [map]
        downsample_resolution: 0.1
        perception_default_frame: map
        map:
          topic: map
          type: sensor_msgs/msg/PointCloud2
          group: points

The builder publishes the resulting GridMap on:

- **Topic:** ``/map_builder_gridmap/gridmap``  
- **Type:** ``grid_map_msgs/msg/GridMap``

---

5. Run the Gridmap Maps Manager
-------------------------------

The **Gridmap Maps Manager** can ingest the GridMap published by the builder via a remap of its ``incoming_map`` input.

**Option A (recommended):** start the full EasyNav system with a minimal parameter file where all plugins are *dummy* except the Gridmap Maps Manager:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_playground_summit/config/summit_building_params.yaml \
     -r /maps_manager_node/gridmap/incoming_map:=/map_builder_gridmap/gridmap

**Option B (standalone):** if available, run a dedicated manager executable with the same remap.  
If your workspace does not include a standalone binary, use **Option A**.

Example parameter file (``summit_building_params.yaml``):

.. code-block:: yaml

    controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [dummy]
        dummy:
          plugin: easynav_controller/DummyController

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [dummy]
        dummy:
          plugin: easynav_localizer/DummyLocalizer

    maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [gridmap]
        gridmap:
          freq: 10.0
          plugin: easynav_gridmap_maps_manager/GridmapMapsManager

    planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [dummy]
        dummy:
          plugin: easynav_planner/DummyPlanner

    sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: /scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

This setup runs a lightweight EasyNav system in which the **Gridmap Maps Manager** receives the GridMap from the builder.

---

6. Save the GridMap to Disk
---------------------------

Once the GridMap has been received, you can save it to disk by calling the Maps Manager service.  
This will create a `.yaml` file and **one `.pgm` per layer** (for example, `_elevation.pgm`, `_traversability.pgm`).

.. code-block:: bash

   ros2 service call /maps_manager_node/gridmap/savemap std_srvs/srv/Trigger

You can later reload this map in the **Gridmap Maps Manager** by setting the same ``package`` and ``map_path_file`` parameters in your configuration.

.. note::
   GridMap supports multi-layer data structures (elevation, traversability, etc.) and is ideal for 3D or semi-structured terrain.
   You can visualize individual layers in RViz using the *GridMap* plugin or by converting to an image topic.
