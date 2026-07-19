.. _gridmap_mapping:

==========================================================
Outdoor Mapping from a Point Cloud: NavMap and Bonxai Maps
==========================================================

.. warning::

   **This page originally described a "GridMap" stack** (``easynav_gridmap_stack``,
   a ``GridmapMapsManager`` plugin, a separate ``gridmap_maps_builder_main`` executable, and an
   ``easynav_lidarslam_ros2``/``easynav_outdoor_testcase`` integration for the Summit robot).
   None of these exist in the current EasyNav codebase: there is no "gridmap"/``grid_map``
   concept, no ``easynav_gridmap_stack`` or ``easynav_gridmap_maps_manager`` package, and no
   ``easynav_lidarslam_ros2``/``easynav_outdoor_testcase`` packages in this workspace. The
   3D/outdoor-capable map representations that **do** exist today are:

   - **NavMap** — a triangulated 3D surface mesh (`easynav_navmap_maps_manager`), which can encode
     uneven/outdoor terrain and per-cell cost layers.
   - **Bonxai** — a probabilistic voxel occupancy map (`easynav_bonxai_maps_manager`).

   Both maps managers build their representation **directly** from an incoming
   ``sensor_msgs/msg/PointCloud2`` — there is no separate "map builder" node to run. This page has
   been rewritten to reflect that; for a fully worked, verified example (playing back a recorded
   point-cloud map and building both a NavMap and a Bonxai map from it), see
   :doc:`bonxai_navmap_from_rosbag`, which covers the same ground with a concrete, working
   configuration.

This HowTo explains how to build a **NavMap** (or a **Bonxai** map) from a point cloud produced by
a SLAM algorithm, and save it to disk so it can be loaded later by the corresponding Maps Manager
and planner.

.. contents:: On this page
   :local:
   :depth: 2

Setup
------

Before starting, ensure that:

1. You have completed the installation described in :doc:`../build_install/index`.
2. You have a working workspace containing the following repositories:

   - ``EasyNavigation`` (provides ``easynav_system``, ``easynav_sensors``, etc.)
   - ``easynav_plugins`` (provides ``easynav_navmap_maps_manager`` and ``easynav_bonxai_maps_manager``)
   - ``NavMap`` (the core NavMap library and ROS message/conversion utilities)
   - ``easynav_playground_summit`` *(for the Summit simulation world, if you want an outdoor scene)*
   - A SLAM package of your choice able to publish a map as a ``sensor_msgs/msg/PointCloud2``
     (this workspace does not currently bundle a specific outdoor LiDAR-SLAM integration).

.. warning::

   Unlike the previous version of this page, there is **no dedicated "GridMap" repository to
   clone**. If you were pointed here expecting ``easynav_gridmap_stack`` or
   ``easynav_lidarslam_ros2``, those packages are not part of the current EasyNav plugin
   ecosystem — use NavMap/Bonxai instead, as described below.

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

1. Launching a simulation (e.g. the **Summit playground**) or otherwise obtaining a live sensor feed.
2. Running a SLAM algorithm that publishes an accumulated map as a ``PointCloud2``.
3. Launching EasyNav with the **NavMap Maps Manager** and/or **Bonxai Maps Manager**, which build
   their map representation directly from that point cloud (no separate "builder" step).
4. Saving the resulting map to disk for later use.

---

1. Start the Simulator
----------------------

Start the Summit world simulation in Gazebo.

.. code-block:: bash

   ros2 launch easynav_playground_summit playground_summit.launch.py

This launch file (``easynav_playground_summit/launch/playground_summit.launch.py``) only starts the
Gazebo simulation (it includes ``summit_simulator``'s ``robot_gazebo.launch.py``) — it does not
start RViz2 or any EasyNav node, so start those separately as shown below.

---

2. Run SLAM to Produce a Point-Cloud Map
-----------------------------------------

Run whichever SLAM pipeline you use to accumulate a 3D map of the environment, and confirm it
publishes the map as a point cloud, for example:

- **Topic:** ``/map``
- **Type:** ``sensor_msgs/msg/PointCloud2``
- **Frame:** ``map`` (or a frame you statically link to ``map`` — see
  :doc:`bonxai_navmap_from_rosbag` for an example of aligning frames with
  ``tf2_ros static_transform_publisher``).

In RViz, add a *PointCloud2* display for that topic to check that the map is accumulating as the
robot moves.

---

3. Drive the Robot to Build the Map
-------------------------------------

Drive the robot around the environment to accumulate the map:

.. code-block:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard

Continue until the environment is sufficiently covered.

---

4. Launch EasyNav with the NavMap/Bonxai Maps Manager
--------------------------------------------------------

The **NavMapMapsManager** and **BonxaiMapsManager** each subscribe directly to an
``incoming_pc2_map`` topic and build their map representation from the accumulated cloud — there is
no separate "map builder" executable to run first.

Run EasyNav with a parameter file in which all plugins are *dummy* except the Maps Manager(s) you
want to build, and remap ``incoming_pc2_map`` to your SLAM's map topic:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file /path/to/your/mapping.dummy.params.yaml \
     -r /maps_manager_node/navmap/incoming_pc2_map:=/map \
     -r /maps_manager_node/bonxai/incoming_pc2_map:=/map

Example parameter file (mirroring the verified configuration in
:doc:`bonxai_navmap_from_rosbag`):

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
        map_types: [navmap, bonxai]
        navmap:
          freq: 10.0
          plugin: easynav_navmap_maps_manager/NavMapMapsManager
        bonxai:
          freq: 10.0
          plugin: easynav_bonxai_maps_manager/BonxaiMapsManager

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

    system_node:
      ros__parameters:
        use_sim_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05

This runs a lightweight EasyNav system in which the Maps Manager(s) receive the point cloud and
build their map representation in real time.

---

5. Save the Map to Disk
------------------------

Once the map has been built, save it via the Maps Manager's ``savemap`` service.

**NavMap:**

.. code-block:: bash

   ros2 service call /maps_manager_node/navmap/savemap std_srvs/srv/Trigger

This unconditionally writes the mesh to ``/tmp/map.navmap``, regardless of any
``navmap_path_file`` you configured; move/rename it afterwards.

**Bonxai:**

.. code-block:: bash

   ros2 service call /maps_manager_node/bonxai/savemap std_srvs/srv/Trigger

This writes back to whichever path was used to *load* the map: ``/tmp/bonxai_map.pcd`` if
``package``/``bonxai_path_file`` were left unset, or the configured ``package``/``bonxai_path_file``
location otherwise (see the caveats in :doc:`bonxai_navmap_from_rosbag`).

You can later reload either map by setting the corresponding ``package`` and
``navmap_path_file``/``bonxai_path_file`` parameters in your configuration.

.. note::
   NavMap supports per-layer cost data (e.g. ``obstacles``, ``inflated_obstacles``) on top of its
   triangulated surface and is the closest current equivalent to a multi-layer, terrain-aware map
   representation. Bonxai instead stores a probabilistic 3D occupancy voxel grid. Neither is a
   drop-in replacement for the previously-documented multi-layer ``grid_map`` format (there is no
   elevation/traversability-layer PGM export in the current codebase).
