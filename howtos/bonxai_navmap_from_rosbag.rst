.. _bonxai_navmap_from_rosbag:

======================================================
Building Bonxai and NavMap Maps from a Recorded ROSBag
======================================================

This HowTo shows how to generate both **Bonxai** and **NavMap** representations
from a **recorded ROSBag** containing a ``PointCloud2`` map, typically produced
by a SLAM algorithm. The input point cloud must be referenced to the ``world``
or ``map`` frame.

.. contents:: On this page
   :local:
   :depth: 2

---

Overview
--------

You will:

1. Align coordinate frames (``world`` → ``map`` if needed).  
2. Play a pre-recorded ROS bag containing the ``/map`` point cloud.  
3. Launch EasyNav with both **BonxaiMapsManager** and **NavMapMapsManager**
   to build the two maps simultaneously.  
4. Visualize both in **RViz2**.  
5. Save the resulting maps to disk.

---

1. Align the Frames (world → map)
---------------------------------

If your ROS bag publishes a ``PointCloud2`` in the frame ``world``,
you need to publish a static transform so that the **map managers**
receive data in ``map``.

In a new terminal, run:

.. code-block:: bash

   ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id map

Keep this terminal **running for the entire session**.  
If your point cloud is already in the ``map`` frame, skip this step.

---

2. Play the ROSBag
------------------

Next, replay the recorded ROS bag containing the point cloud map.

The example bag is about 1600 seconds long.  
To jump close to the end (around 1500 seconds, when the map is already dense)
but still leave a few seconds to build the maps, run:

.. code-block:: bash

   ros2 bag play rosbag_excavation_urjc_map_tf_only --clock --start-offset 1500

This will replay the point cloud topic ``/map`` and the TF tree
needed by the map managers.

---

3. Launch EasyNav with Bonxai and NavMap Maps Managers
------------------------------------------------------

In a new terminal, launch **EasyNav System** with both map managers active.

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args \
     --params-file /absolute/path/to/bonxai-navmap.dummy.params.yaml \
     -r /maps_manager_node/navmap/incoming_pc2_map:=/map \
     -r /maps_manager_node/bonxai/incoming_pc2_map:=/map

This setup assumes:

- The input cloud topic is ``/map``.  
- The QoS settings are standard (reliable, transient local).  
- Both managers will build their respective maps automatically.

The file ``bonxai-navmap.dummy.params.yaml`` defines **dummy plugins**
for all nodes except ``maps_manager_node``.

Example configuration:

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
        map_types: [bonxai, navmap]
        bonxai:
          freq: 10.0 
          plugin: easynav_bonxai_maps_manager/BonxaiMapsManager
          package: easynav_indoor_testcase
          bonxai_path_file: maps/excavation_urjc.pcd
        navmap:
          freq: 10.0 
          plugin: easynav_navmap_maps_manager/NavMapMapsManager
          package: easynav_indoor_testcase
          navmap_path_file: maps/excavation_urjc.navmap

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
        perception_default_frame: odom

    system_node:
      ros__parameters:
        use_sim_time: true
        use_real_time: false
        position_tolerance: 0.3
        angle_tolerance: 0.15

---

4. NavMap Build Parameters
--------------------------

The **NavMapMapsManager** allows tuning of mesh generation parameters
via YAML configuration. You may add the following optional fields
under the ``navmap`` plugin section.

**Available parameters**

+----------------------+----------------------------------------------------------+
| **Parameter**        | **Description**                                          |
+======================+==========================================================+
| ``resolution``       | In-plane sampling resolution (m) for voxelization.       |
+----------------------+----------------------------------------------------------+
| ``max_edge_len``     | Maximum triangle edge length (m).                        |
+----------------------+----------------------------------------------------------+
| ``max_dz``           | Maximum allowed vertical jump (m) between vertices.      |
+----------------------+----------------------------------------------------------+
| ``max_slope_deg``    | Maximum slope (degrees) relative to the vertical axis.   |
+----------------------+----------------------------------------------------------+
| ``neighbor_radius``  | Neighborhood radius (m) for triangle connectivity.       |
+----------------------+----------------------------------------------------------+
| ``k_neighbors``      | Alternative to radius: number of nearest neighbors.      |
+----------------------+----------------------------------------------------------+
| ``min_area``         | Minimum triangle area (m²) to reject degenerate faces.   |
+----------------------+----------------------------------------------------------+
| ``use_radius``       | Use radius-based vs. k-NN connectivity. (bool)           |
+----------------------+----------------------------------------------------------+
| ``min_angle_deg``    | Minimum interior angle (degrees) to avoid slivers.       |
+----------------------+----------------------------------------------------------+
| ``max_surfaces``     | Keep only the N largest connected surfaces (0 = all).    |
+----------------------+----------------------------------------------------------+

These parameters allow fine control of NavMap mesh reconstruction quality.

---

5. Visualize in RViz2
---------------------

Open **RViz2** to monitor both map managers.

.. code-block:: bash

   ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

In RViz:

- Add a **PointCloud2** display for **Bonxai** maps.
  - Select the topic published by BonxaiMapsManager (e.g., ``/maps_manager_node/bonxai/map``).  
  - QoS: **Transient Local**.  
  - You can visualize the cloud as boxes with configurable size for clearer 3D structure.

- Add a **NavMapDisplay** (custom display type).
  - Select the topic published by NavMapMapsManager (e.g., ``/maps_manager_node/navmap/map``).  
  - QoS: **Transient Local**.

.. figure:: ../images/mapping_tut_bonxai.png
   :align: center
   :width: 70%

   Bonxai map visualization in RViz.

.. figure:: ../images/mapping_tut_navmap.png
   :align: center
   :width: 70%

   NavMap mesh visualization in RViz.

---

6. Save the Maps
----------------

When both maps are visible and complete, you can store them to disk using
their respective map manager services.

**Save NavMap**

.. code-block:: bash

   ros2 service call /maps_manager_node/navmap/savemap std_srvs/srv/Trigger

This saves the NavMap to ``/tmp/map.navmap``.  
Rename and move it to your desired location (e.g. inside ``maps/``).

**Save Bonxai Map**

.. code-block:: bash

   ros2 service call /maps_manager_node/bonxai/savemap std_srvs/srv/Trigger

This saves the Bonxai point cloud to ``/tmp/bonxai_map.pcd``.

---

7. Summary
----------

You have:

- ✅ Aligned frames between ``world`` and ``map``
- ✅ Played a recorded ROS bag with point cloud data  
- ✅ Built Bonxai and NavMap maps simultaneously  
- ✅ Visualized them in RViz  
- ✅ Saved both maps to disk for future use

These maps can now be used in EasyNav navigation stacks (e.g. with
the **Costmap**, **GridMap**, or **NavMap** planners).

---

**Next steps:**

- :doc:`../developer_guide/design`
- :doc:`../howtos/gridmap_mapping`
