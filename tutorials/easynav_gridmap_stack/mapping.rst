
.. mapping:

Gridmap Mapping with LidarSLAM and EasyNav
------------------------------------------

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/mZR1CqNV5aU" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

This tutorial shows how to **build a multi‑layer GridMap** from a LidarSLAM point cloud and save it
to disk (YAML + one PGM per layer) so it can be consumed later by the Gridmap Maps Manager and planner.

Prerequisites
^^^^^^^^^^^^^

- A working build of the *easynav_gridmap_stack* and *easynav_lidarslam_ros2*.
- The summit playground world from ``easynav_playground_summit``.
- RViz2 installed for visualization.
- ``use_sim_time: true`` in all nodes participating in simulation.

1) Start the simulator
^^^^^^^^^^^^^^^^^^^^^^

This launch starts Gazebo and RViz preconfigured for the Summit world.

.. code-block:: bash

   ros2 launch easynav_playground_summit playground_summit.launch.py

RViz will open automatically—keep it to visualize topics.

2) Launch LidarSLAM
^^^^^^^^^^^^^^^^^^^

Start the LidarSLAM pipeline (scan matcher + graph‑based SLAM). If your package provides
a consolidated launch file, use it as follows:

.. code-block:: bash

   ros2 launch lidarslam lidarslam.launch.py

You can close any *extra* RViz instance this launch may open; we will use the one from step (1).

As you move the robot, LidarSLAM will publish a **map point cloud** on:

- Topic: ``/map``
- Type: ``sensor_msgs/msg/PointCloud2``
- Frame: ``map``

In RViz, add a *PointCloud2* display for ``/map`` and increase the point size to make it clearer.

3) Open a teleop and build the map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Drive the robot around to cover the environment while LidarSLAM accumulates the map:

.. code-block:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard

4) Start the **GridMap Builder** (PointCloud → GridMap)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the builder that converts the accumulated ``/map`` point cloud into a multi‑layer GridMap.
The example parameters file listens to ``/map`` and downsamples the cloud before rasterization.

.. code-block:: bash

   ros2 run easynav_gridmap_maps_manager gridmap_maps_builder_main \
     --ros-args --params-file /home/fmrico/ros/ros2/easynav_ws/src/easynav_outdoor_testcase/robots_params/maps_builder.params.yaml

Example parameters for the builder:

.. code-block:: yaml

    pointcloud_maps_builder_node:
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

- Topic: ``/maps_builder_gridmap/gridmap``
- Type: ``grid_map_msgs/msg/GridMap``

5) Start the **Gridmap Maps Manager** to receive the built map
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The maps manager will ingest the GridMap from the builder via a **remap** of its ``incoming_map`` input.
You can run it in two ways:

**Option A (recommended):** start the EasyNav system with a minimal parameter file where all plugins are *dummy* except the Gridmap Maps Manager:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file /home/fmrico/ros/ros2/easynav_ws/src/easynav_playground_summit/config/summit_building_params.yaml \
     -r /maps_manager_node/gridmap/incoming_map:=/maps_builder_gridmap/gridmap

**Option B (standalone binary, if available in your build):** run a dedicated manager executable and apply the same remap.
If your workspace doesn’t provide a standalone binary, use **Option A**.

Example parameters (``summit_building_params.yaml``) used in Option A:

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

This file keeps all runtime lightweight while the **Gridmap Maps Manager** receives the GridMap from the builder.

6) Save the GridMap to disk (YAML + PGM per layer)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Call the maps manager service to write the current GridMap to the path configured by
``package`` and ``map_path_file`` (or an absolute path, if set). The manager will produce
a ``.yaml`` file plus **one PGM per map layer** (e.g., ``*_elevation.pgm``).

.. code-block:: bash

   ros2 service call /maps_manager_node/gridmap/savemap std_srvs/srv/Trigger

You can later load this map by setting the same ``package`` and ``map_path_file`` in your
Gridmap Maps Manager plugin configuration.
