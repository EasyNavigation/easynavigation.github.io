.. _costmap_mapping:

==============================
Mapping with the Costmap Stack
==============================

This HowTo explains how to build a map for the **Costmap Stack** in EasyNavigation (EasyNav) with
**SLAM Toolbox**. The Costmap Stack uses a **graded Costmap2D** that encodes traversal costs and
supports inflation around obstacles, and reads and writes maps in the same **YAML + image** format
as Nav2.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/n1vDA4ZeG6M" frameborder="0" allowfullscreen></iframe>
    </div>

This tutorial uses the :doc:`Kobuki PlayGround <../playgrounds/kobuki>`. The
workflow is:

1. Run the simulator.
2. Build the map with **SLAM Toolbox**, driving the robot around.
3. Run **EasyNav** in *mapping mode*, so that the Costmap Maps Manager receives and shows the map.
4. Save the map (YAML + image) and use it for navigation.

If you already have a Nav2 map, you can skip to :doc:`costmap_navigating`: the Costmap Maps
Manager loads it as it is.

Setup
-----

Build EasyNav and the Kobuki PlayGround as described in :doc:`../getting_started/index`, and
install **SLAM Toolbox**:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-slam-toolbox

Step-by-Step Instructions
-------------------------

1. **Launch the simulator**

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki gazebo_sim.launch.yaml gui:=false

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

3. **Start EasyNav in mapping mode**

   In *mapping mode* only the **Maps Manager** does something; the rest of the nodes use dummy
   plugins. The Kobuki PlayGround ships this configuration as
   ``params/costmap.mapping.params.yaml``. The Costmap Maps Manager receives external maps on
   ``/maps_manager_node/costmap/incoming_map``, so remap SLAM Toolbox's ``/map`` to it:

   .. code-block:: bash

      ros2 run easynav_system system_main --ros-args \
        --params-file $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/params/costmap.mapping.params.yaml \
        -r /maps_manager_node/costmap/incoming_map:=/map

   The relevant part of the configuration is the maps manager, with no map file:

   .. code-block:: yaml

       maps_manager_node:
         ros__parameters:
           use_sim_time: true
           map_types: [costmap]
           costmap:
             freq: 10.0
             plugin: easynav_costmap_maps_manager/CostmapMapsManager

4. **Open RViz2**

   .. code-block:: bash

      ros2 run rviz2 rviz2 \
        -d $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/rviz/easynav_costmap.rviz \
        --ros-args -p use_sim_time:=true

   The Costmap Maps Manager publishes the map on ``/maps_manager_node/costmap/map`` (static, QoS
   *Transient Local*) and ``/maps_manager_node/costmap/dynamic_map`` (with obstacles and
   inflation, when the configuration has those filters).

5. **Teleoperate the robot to build the map**

   .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard

Saving the map
--------------

Save the map with SLAM Toolbox, which writes the YAML + image pair:

.. code-block:: bash

   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'office'}}"

It writes ``office.yaml`` and ``office.pgm`` in the directory where SLAM Toolbox was launched
(or use Nav2's ``map_saver_cli``, if you have it). Put both files in a directory installed by a
package of your workspace, e.g. ``my_maps_pkg/maps/office.yaml`` and ``office.pgm``. If you
rename them, make sure the ``image`` field of the YAML matches the image file name.

Then load it in your navigation parameter file. The map is located with ``package`` (a ROS 2
package, searched in its share directory) and ``map_path_file`` (the path inside it); both are
needed:

.. code-block:: yaml

   maps_manager_node:
     ros__parameters:
       map_types: [costmap]
       costmap:
         freq: 10.0
         plugin: easynav_costmap_maps_manager/CostmapMapsManager
         package: my_maps_pkg
         map_path_file: maps/office.yaml

Continue with :doc:`costmap_navigating`.

.. note::

   This tutorial is analogous to :doc:`simple_mapping`; the difference is the internal map
   representation. The Costmap2D structure encodes graded values (0–255) with inflation, allowing
   planners and controllers to reason about proximity to obstacles instead of using a simple
   free/occupied binary map.
