.. _tutorials:

EasyNav Stack Tutorials
#######################

.. toctree::
   :maxdepth: 1

   ./easynav_plugins/index.rst
   ./easynav_simple_stack/index.rst
   ./easynav_costmap_stack/index.rst
   ./easynav_gridmap_stack/index.rst

Multi-robot with EasyNav
************************

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/BDOEr_L4mq8" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

Multi-robot navigation with EasyNav is straightforward, but it **requires discipline** from plugin developers
when naming topics and frames. In particular:

- **Always publish topics with fully-qualified names**, prefixed by the node FQN and plugin name.
  This prevents collisions between robots that run the same plugins. For example:

  .. code-block:: cpp

     path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
       node->get_fully_qualified_name() + std::string("/") + plugin_name + "/path",
       10);

- **TF management** is critical:
  
  1) Each robot typically owns its **own TF topics**. With namespacing, that means you want
     ``/r1/tf`` and ``/r1/tf_static`` for robot 1, ``/r2/tf`` and ``/r2/tf_static`` for robot 2, etc.
     To achieve this, **remap** the global TF topics to relative ones (so they get namespaced):
     ``-r /tf:=tf -r /tf_static:=tf_static``.
     Do **not** apply this remap if both robots are designed to share the same TF tree (uncommon).

  2) **Namespaced frame IDs**: if you set a TF prefix in EasyNav (e.g., ``tf_prefix: r1``),
     frames will be published as ``r1/map``, ``r1/odom``, ``r1/base_link``, etc.
     Configure this in the EasyNav **system node** parameters:

     .. code-block:: yaml

        r1/system_node:
          ros__parameters:
            use_sim_time: true
            use_real_time: true
            position_tolerance: 0.1
            angle_tolerance: 0.05
            tf_prefix: r1

Launching in Simulation
=======================

1) **Start the simulator with two robots**

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki playground_multirobot_kobuki.launch.py do_tf_remapping:=true gui:=false

   .. note::
      If your launch file uses a different name, adapt the command accordingly.

2) **Start EasyNav for each robot** in a separate terminal, using the same parameter file
   but **different namespaces** and **TF remaps**:

   .. code-block:: bash

      # Terminal 1 (robot r1)
      ros2 run easynav_system system_main \
        --ros-args \
        --params-file /home/fmrico/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap_multirobot.params.yaml \
        -r __ns:=r1 \
        -r /tf:=tf -r /tf_static:=tf_static

      # Terminal 2 (robot r2)
      ros2 run easynav_system system_main \
        --ros-args \
        --params-file /home/fmrico/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap_multirobot.params.yaml \
        -r __ns:=r2 \
        -r /tf:=tf -r /tf_static:=tf_static

Example Parameters
==================

Below is an example parameter file for **two robots**. Each section is namespaced (``r1/...``, ``r2/...``),
so the same plugins can run without topic or TF collisions. Notice the **TF prefix** under each
``system_node`` and the **filter plugins** under the maps manager (correct plugin names).

.. code-block:: yaml

    r1/controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [simple]
        simple:
          rt_freq: 30.0
          plugin: easynav_simple_controller/SimpleController
          max_linear_speed: 1.0
          max_angular_speed: 1.5
          look_ahead_dist: 0.2
          k_rot: 0.7

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
          package: easynav_indoor_testcase
          map_path_file: maps/home2.yaml
          filters: [obstacles, inflation]
          obstacles:
            plugin: easynav_costmap_maps_manager/ObstacleFilter
          inflation:
            plugin: easynav_costmap_maps_manager/InflationFilter
            inflation_radius: 1.3
            cost_scaling_factor: 3.0

    r1/planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_costmap_planner/CostmapPlanner
          cost_factor: 10.0

    r1/sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points
        camera1:
          topic: rgbd_camera/points
          type: sensor_msgs/msg/PointCloud2
          group: points

    r1/system_node:
      ros__parameters:
        use_sim_time: true
        use_real_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05
        tf_prefix: r1

    r2/controller_node:
      ros__parameters:
        use_sim_time: true
        controller_types: [simple]
        simple:
          rt_freq: 30.0
          plugin: easynav_simple_controller/SimpleController
          max_linear_speed: 1.0
          max_angular_speed: 1.5
          look_ahead_dist: 0.2
          k_rot: 0.7

    r2/localizer_node:
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
            x: 2.0
            y: 1.0
            yaw: 0.0
            std_dev_xy: 0.1
            std_dev_yaw: 0.01

    r2/maps_manager_node:
      ros__parameters:
        use_sim_time: true
        map_types: [costmap]
        costmap:
          freq: 10.0
          plugin: easynav_costmap_maps_manager/CostmapMapsManager
          package: easynav_indoor_testcase
          map_path_file: maps/home2.yaml
          filters: [obstacles, inflation]
          obstacles:
            plugin: easynav_costmap_maps_manager/ObstacleFilter
          inflation:
            plugin: easynav_costmap_maps_manager/InflationFilter
            inflation_radius: 1.3
            cost_scaling_factor: 3.0

    r2/planner_node:
      ros__parameters:
        use_sim_time: true
        planner_types: [simple]
        simple:
          freq: 0.5
          plugin: easynav_costmap_planner/CostmapPlanner
          cost_factor: 10.0

    r2/sensors_node:
      ros__parameters:
        use_sim_time: true
        forget_time: 0.5
        sensors: [laser1]
        perception_default_frame: odom
        laser1:
          topic: scan_raw
          type: sensor_msgs/msg/LaserScan
          group: points
        camera1:
          topic: rgbd_camera/points
          type: sensor_msgs/msg/PointCloud2
          group: points

    r2/system_node:
      ros__parameters:
        use_sim_time: true
        use_real_time: true
        position_tolerance: 0.1
        angle_tolerance: 0.05
        tf_prefix: r2

3) **Launch two RViz instances**, one per namespace

   .. code-block:: bash

      ros2 launch easynav_playground_kobuki rviz_namespaced.launch.py \
        namespace:=r1 use_sim_time:=true \
        rviz_config_file:=/home/fmrico/ros/ros2/easynav_ws/src/easynav_playground_kobuki/rviz/nav2_namespaced_view.rviz

      ros2 launch easynav_playground_kobuki rviz_namespaced.launch.py \
        namespace:=r2 use_sim_time:=true \
        rviz_config_file:=/home/fmrico/ros/ros2/easynav_ws/src/easynav_playground_kobuki/rviz/nav2_namespaced_view.rviz

Tips & Gotchas
==============

- **Namespaces everywhere**: ensure all relative topics in your parameters (e.g., ``scan_raw``)
  are resolved under each robot namespace (e.g., ``/r1/scan_raw``).
- **Do not over-remap**: only remap ``/tf`` and ``/tf_static`` to relative topic names if each
  robot maintains its own TF tree.
- **Frames**: with ``tf_prefix`` set, refer to frames as ``r1/base_link``, ``r1/odom``, etc.
- **Discovery**: if you have *separate* networks or want isolation, consider different
  ``ROS_DOMAIN_ID`` per fleet. Otherwise, keep the same domain to allow shared visualization.
