.. _simple_mapping:

=====================================
Mapping with SLAM Toolbox and EasyNav
=====================================

This HowTo explains how to create a **2D occupancy map** using **SLAM Toolbox** and store it for later use with
the **Simple Stack** in EasyNavigation (EasyNav).

.. warning::
   The Simple stack is a minimal example, with very basic algorithms. For real use, map with the
   Costmap stack instead (:doc:`costmap_mapping`): the steps are the same.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/n1vDA4ZeG6M" frameborder="0" allowfullscreen></iframe>
    </div>

This tutorial uses the :doc:`Kobuki PlayGround <../playgrounds/kobuki>` and the
**Simple Stack**, which relies on binary occupancy grids (``0`` for free, ``1`` for occupied).

The workflow consists of:

1. Running the simulator.
2. Starting **SLAM Toolbox** to build the map.
3. Using **EasyNav** to receive and save the map through the Simple Maps Manager.
4. Using the saved map for navigation.

Setup
-----

Build EasyNav and the Kobuki PlayGround as described in :doc:`../getting_started/index`.

**SLAM Toolbox** can be installed via apt:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-slam-toolbox

or built from source: https://github.com/SteveMacenski/slam_toolbox

Step-by-Step Instructions
-------------------------

1. **Launch the simulator (with or without GUI)**

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki_worlds gazebo_sim.launch.yaml gui:=false

2. **Launch SLAM Toolbox**

   SLAM Toolbox reads the laser on ``/scan``, but the Kobuki publishes it on ``/scan_raw``. Copy
   its default parameter file and set ``scan_topic: /scan_raw`` in it:

   .. code-block:: bash

      cp /opt/ros/${ROS_DISTRO}/share/slam_toolbox/config/mapper_params_online_async.yaml ~/slam_kobuki.yaml
      # edit ~/slam_kobuki.yaml: scan_topic: /scan_raw

   Then launch it:

   .. code-block:: bash

      ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=true slam_params_file:=$HOME/slam_kobuki.yaml

   SLAM Toolbox publishes the map (``nav_msgs/msg/OccupancyGrid``) on ``/map``.

3. **Start EasyNav in mapping mode**

   In *mapping mode* only the **Maps Manager** does something; the rest of the nodes use dummy
   plugins. The Kobuki PlayGround ships this configuration as
   ``params/simple.mapping.params.yaml``. The Simple Maps Manager receives external maps on
   ``/maps_manager_node/simple/incoming_map``, so remap SLAM Toolbox's ``/map`` to it:

   .. code-block:: bash

      ros2 run easynav_system system_main --ros-args \
        --params-file $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/params/simple.mapping.params.yaml \
        -r /maps_manager_node/simple/incoming_map:=/map

   The relevant part of the configuration is the maps manager, with no map file:

   .. code-block:: yaml

       maps_manager_node:
         ros__parameters:
           use_sim_time: true
           map_types: [simple]
           simple:
             freq: 10.0
             plugin: easynav_simple_maps_manager/SimpleMapsManager

4. **Open RViz2**

   .. code-block:: bash

      ros2 run rviz2 rviz2 \
        -d $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/rviz/easynav_simple.rviz \
        --ros-args -p use_sim_time:=true

5. **Teleoperate the robot to build the map**

   .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard

   As you move the robot, SLAM Toolbox publishes the growing map on ``/map``, and the Simple Maps
   Manager republishes it as its own map.

Saving and Reusing the Map
--------------------------

1. **Save the generated map**

   .. code-block:: bash

      ros2 service call /maps_manager_node/simple/savemap std_srvs/srv/Trigger

   With no map configured (``package`` and ``map_path_file``), the map is stored in
   ``/tmp/default.map``.

2. **Move the map into a package of your workspace**, for example ``my_maps_pkg``, in a directory
   that the package installs (e.g. ``maps/``):

   .. code-block:: bash

      mv /tmp/default.map ~/easynav_ws/src/my_maps_pkg/maps/house.map

3. **Update your navigation parameters** to load it. The map is located with ``package`` (a ROS 2
   package, searched in its share directory) and ``map_path_file`` (the path inside it); both are
   needed:

   .. code-block:: yaml

      maps_manager_node:
        ros__parameters:
          use_sim_time: true
          map_types: [simple]
          simple:
            freq: 10.0
            plugin: easynav_simple_maps_manager/SimpleMapsManager
            package: my_maps_pkg
            map_path_file: maps/house.map

You can now use this map with the Simple Stack (see :doc:`simple_navigating`).

Notes
-----

- The **Simple Maps Manager** saves and loads maps using its own lightweight text format
  (a single ``.map`` file: width, height, resolution and origin on the first line, followed by the
  binary occupancy data). It is **not** the YAML + PGM format used by Nav2.
  If you need a Nav2-compatible YAML + PGM map, use the *Costmap Stack* instead
  (:doc:`costmap_mapping`).
- You can visualize both SLAM and EasyNav map topics in RViz2 to confirm synchronization.
