.. _costmap_multirobot:

===================================
Multi-Robot Navigation with EasyNav
===================================

This HowTo demonstrates how to run multiple EasyNav robots simultaneously in a shared simulation.
Each robot operates independently using its own namespaced set of nodes, topics, and frames.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/BDOEr_L4mq8" frameborder="0" allowfullscreen></iframe>
    </div>

Multi-robot navigation with EasyNav is straightforward, but it **requires discipline** when naming
topics and managing TF frames. This tutorial explains how to set up multiple robots without topic
or TF conflicts, using the :doc:`Kobuki PlayGround <../playgrounds/kobuki>`.

Setup
-----

Build EasyNav and the Kobuki PlayGround as described in :doc:`../getting_started/index`.

1. Topic Naming and TF Management
---------------------------------

Fully-qualified topic names (plugin developers)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each plugin should publish topics under its **fully qualified name**, which includes the node and
plugin names. This prevents collisions when several robots run the same plugins.

Example (in C++):

.. code-block:: cpp

   path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
     node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path",
     10);

This ensures topics look like ``/r1/planner_node/simple/path`` and ``/r2/planner_node/simple/path``
instead of clashing on ``/path``.

TF topic remapping
^^^^^^^^^^^^^^^^^^

Each robot typically has its **own TF tree**. To isolate TF data, remap the global TF topics
(``/tf`` and ``/tf_static``) to **relative ones**, so they are automatically namespaced:

.. code-block:: bash

   -r /tf:=tf -r /tf_static:=tf_static

This yields separate TF topics for each robot: ``/r1/tf`` and ``/r1/tf_static``, ``/r2/tf`` and
``/r2/tf_static``.

.. note::
   Do **not** apply this remap if both robots are designed to share the same TF tree (which is rare).

Namespaced frames
^^^^^^^^^^^^^^^^^

If you configure a TF prefix for each system node (``system_node.tf_prefix``, e.g. ``r1``), then
all frames include this prefix: ``r1/map``, ``r1/odom``, ``r1/base_link``, etc.

.. code-block:: yaml

   r1/system_node:
     ros__parameters:
       tf_prefix: r1

The simulated robots must publish their frames with the same prefix: the PlayGround's
``kobuki.launch.yaml`` does it when launched with a ``namespace``.

2. Launching in Simulation
--------------------------

The whole system (two Kobukis, ``r1`` at (0, 0) and ``r2`` at (2, 1), each with its own EasyNav and
RViz2) starts with:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_multirobot.launch.yaml

Send **2D Goal Poses** independently in each RViz2 window.

Step by step, this is what the launch file does, and what you would do with your own robots:

1. **Start the simulator with two robots** (``multirobot_gazebo_sim.launch.yaml``): each Kobuki
   in its namespace, with its TF topics (``/r1/tf``) and prefixed frames (``r1/base_link``).

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki_worlds multirobot_gazebo_sim.launch.yaml gui:=false

2. **Start EasyNav for each robot**, with the same parameter file, its own namespace and the TF
   remapping:

   .. code-block:: bash

      ros2 run easynav_system system_main \
        --ros-args \
        --params-file $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/params/costmap_multirobot.params.yaml \
        -r __ns:=/r1 -r /tf:=tf -r /tf_static:=tf_static

   and the same with ``__ns:=/r2`` in another terminal.

3. **Start RViz2 for each robot**, in its namespace, with the TF remapping and its ``map`` frame
   as fixed frame:

   .. code-block:: bash

      ros2 run rviz2 rviz2 \
        -d $(ros2 pkg prefix easynav_playground_kobuki)/share/easynav_playground_kobuki/rviz/easynav_multirobot.rviz \
        -f r1/map \
        --ros-args -r __ns:=/r1 -r /tf:=tf -r /tf_static:=tf_static -p use_sim_time:=true

   The RViz2 configuration uses relative topic names, so the same file works for both robots.

3. Example Parameters
---------------------

``params/costmap_multirobot.params.yaml`` has one section per robot (``r1/...``, ``r2/...``),
because each EasyNav runs in its namespace. Each section is the configuration of
:doc:`costmap_navigating`, with that robot's ``tf_prefix`` and initial pose. Robot ``r1``:

.. code-block:: yaml

   r1/controller_node:
     ros__parameters:
       use_sim_time: true
       robot_limits:
         max_linear_vel: 0.6
         min_linear_vel: -0.3
         max_angular_vel: 1.2
         max_linear_acc: 1.0
         max_linear_decel: 1.0
         max_angular_acc: 2.0
         max_angular_decel: 2.0
       controller_types: [rpp]
       rpp:
         rt_freq: 30.0
         plugin: easynav_regulated_pp_controller/RegulatedPurePursuitController
         # ... (as in costmap.rpp.params.yaml)

   r1/localizer_node:
     ros__parameters:
       use_sim_time: true
       localizer_types: [costmap]
       costmap:
         rt_freq: 50.0
         freq: 5.0
         reseed_freq: 1.0
         plugin: easynav_costmap_localizer/AMCLLocalizer
         num_particles: 100
         noise_translation: 0.05
         noise_rotation: 0.1
         noise_translation_to_rotation: 0.1
         initial_pose:
           x: 0.0
           y: 0.0
           yaw: 0.0
           std_dev_xy: 0.1
           std_dev_yaw: 0.01

   r1/maps_manager_node:
     ros__parameters:
       use_sim_time: true
       map_types: [costmap]
       costmap:
         freq: 10.0
         plugin: easynav_costmap_maps_manager/CostmapMapsManager
         package: easynav_playground_kobuki
         map_path_file: maps/home2.yaml
         filters: [obstacles, inflation]
         obstacles:
           plugin: easynav_costmap_maps_manager/CostmapMapsManager/ObstaclesFilter
         inflation:
           plugin: easynav_costmap_maps_manager/CostmapMapsManager/InflationFilter
           inflation_radius: 1.0
           cost_scaling_factor: 5.0

   r1/planner_node:
     ros__parameters:
       use_sim_time: true
       planner_types: [simple]
       simple:
         freq: 0.5
         plugin: easynav_costmap_planner/CostmapPlanner
         cost_factor: 10.0
         continuous_replan: true

   r1/sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser1]
       laser1:
         topic: scan_raw
         type: sensor_msgs/msg/LaserScan

   r1/system_node:
     ros__parameters:
       robot_geometry:
         radius: 0.178
         inscribed_radius: 0.178
         height: 0.5
       use_sim_time: true
       use_real_time: true
       tf_prefix: r1
       position_tolerance: 0.3
       angle_tolerance: 0.15

   r1/recovery_node:
     ros__parameters:
       # ... (as in costmap.rpp.params.yaml)

The ``r2/...`` sections are the same, with ``tf_prefix: r2`` and ``initial_pose`` at
``x: 2.0, y: 1.0``.

Tips & Gotchas
--------------

- **Namespaces everywhere:**
  Verify all relative topics (e.g., ``scan_raw``) are correctly resolved under each robot namespace (``/r1/scan_raw``, ``/r2/scan_raw``).
- **Avoid over-remapping:**
  Only remap ``/tf`` and ``/tf_static`` to relative topics when each robot manages its own TF tree.
- **Frame references:**
  With ``tf_prefix`` set, refer to frames as ``r1/base_link``, ``r1/odom``, etc.
- **ROS Domain IDs:**
  If you want isolation or multiple networks, assign different ``ROS_DOMAIN_ID`` per fleet.
  Otherwise, keep the same domain for shared visualization.
- **Several EasyNav on one computer:** each one writes its own time statistics log, named after
  its namespace (``/tmp/easynav_r1.log``); ``ros2 easynav timestats --namespace r1`` selects one
  (see :doc:`ros2_easynav_cli`).

With this setup, each robot runs a full EasyNav navigation stack under its own namespace,
enabling **coordinated multi-robot simulation** in Gazebo and RViz2.
