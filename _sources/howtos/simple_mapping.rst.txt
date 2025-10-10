.. _simple_mapping:

=====================================
Mapping with SLAM Toolbox and EasyNav
=====================================

This HowTo explains how to create a **2D occupancy map** using **SLAM Toolbox** and store it for later use with
the **Simple Stack** in EasyNavigation (EasyNav).

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/n1vDA4ZeG6M" frameborder="0" allowfullscreen></iframe>
    </div>

This tutorial uses the **Turtlebot2 (Kobuki)** simulator from the *easynav_playground_kobuki* repository and
the **Simple Stack**, which relies on binary occupancy grids (`0` for free, `1` for occupied).

The workflow consists of:

1. Running the simulator.  
2. Starting **SLAM Toolbox** to build the map.  
3. Using **EasyNav** to receive and save the map through the Simple Maps Manager.  
4. Saving the generated YAML + image files for later use.

---

Setup
-----

Before starting, ensure you have completed :doc:`../build_install/index` and cloned the following repositories
in your workspace:

- ``EasyNavigation``  
- ``easynav_plugins``  
- ``easynav_playground_kobuki`` *(for simulation)*  
- ``easynav_indoor_testcase`` *(for configuration and maps)*  

All packages should build correctly with:

.. code-block:: bash

   cd ~/easynav_ws
   colcon build --symlink-install
   source install/setup.bash

---

Running SLAM Toolbox
--------------------

**SLAM Toolbox** is part of the Nav2 ecosystem and can be installed via apt:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-slam-toolbox

Alternatively, it can be built from source:  
https://github.com/SteveMacenski/slam_toolbox

SLAM Toolbox publishes a ``nav_msgs/msg/OccupancyGrid`` on the ``/map`` topic.  
EasyNav’s **Simple Maps Manager** listens on:

.. code-block:: none

   /maps_manager_node/simple/incoming_map

and can consume this topic directly.

---

Step-by-Step Instructions
-------------------------

1. **Launch the simulator (with or without GUI)**

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

2. **Open RViz2**

   .. code-block:: bash

      ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

   .. note::
      The `/map` topic and `map` frame will only appear after launching SLAM Toolbox.

3. **Launch SLAM Toolbox**

   SLAM Toolbox expects a topic named `/scan`.  
   In this setup, the Kobuki’s LIDAR publishes on `/scan_raw`, so you must remap it.

   Edit or copy the launcher file from:

   .. code-block:: bash

      /opt/ros/${ROS_DISTRO}/share/slam_toolbox/launch/online_async_launch.py

   Add the following remapping:

   .. code-block:: python

      remappings=[
          ('/scan', '/scan_raw'),
      ],

   Then launch SLAM Toolbox:

   .. code-block:: bash

      ros2 launch slam_toolbox online_async_launch.py

4. **Teleoperate the robot to build the map**

   .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard

   As you move the robot, SLAM Toolbox will publish the growing map on `/map`, visible in RViz2.

---

Using EasyNav to Receive and Save the Map
-----------------------------------------

Next, run **EasyNav** in *mapping mode*, where only the **Maps Manager** is active and the rest of the nodes use dummy plugins.

Example configuration (`simple.mapping.params.yaml`):

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
        map_types: [simple]
        simple:
          freq: 10.0
          plugin: easynav_simple_maps_manager/SimpleMapsManager

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

Launch EasyNav and remap the incoming map topic:

.. code-block:: bash

   ros2 run easynav_system system_main --ros-args \
     --params-file ~/easynav_ws/src/easynav_indoor_testcase/robots_params/simple.mapping.params.yaml \
     -r /maps_manager_node/simple/incoming_map:=/map

At this point, the **Simple Maps Manager** receives the map directly from SLAM Toolbox.

---

Saving and Reusing the Map
--------------------------

1. **Save the generated map**

   .. code-block:: bash

      ros2 service call /maps_manager_node/simple/savemap std_srvs/srv/Trigger

   By default, the map is stored under `/tmp/default.yaml` and `/tmp/default.pgm`.

2. **Rename and move the map for later use**

   .. code-block:: bash

      mv /tmp/default.yaml ~/easynav_ws/src/easynav_indoor_testcase/maps/house.yaml
      mv /tmp/default.pgm  ~/easynav_ws/src/easynav_indoor_testcase/maps/house.pgm

3. **Update your navigation parameters**

   Modify the map section in your configuration file as follows:

   .. code-block:: yaml

      maps_manager_node:
        ros__parameters:
          use_sim_time: true
          map_types: [simple]
          simple:
            freq: 10.0
            plugin: easynav_simple_maps_manager/SimpleMapsManager
            package: easynav_indoor_testcase
            map_path_file: maps/house.yaml

You can now reuse this map for any *Simple Stack* navigation tutorial (see :doc:`simple_navigating`).

---

Notes
-----

- The saved format (YAML + PGM) is fully compatible with MoveBase, Nav2, and other ROS 2 mapping systems.  
- To perform navigation with graded cost values instead of binary occupancy, use the *Costmap Stack*
  (:doc:`costmap_mapping`).  
- You can visualize both SLAM and EasyNav map topics in RViz2 to confirm synchronization.
