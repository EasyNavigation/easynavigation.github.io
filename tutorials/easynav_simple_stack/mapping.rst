.. _easynav_simple_stack/mapping:

Mapping with SLAM Toolbox and EasyNav
-------------------------------------


.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/n1vDA4ZeG6M" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>


Setup from scratch
^^^^^^^^^^^^^^^^^^

1. Create your workspace if you haven't done yet, and move to the `src` directory:

.. code-block:: bash

   mkdir -p easynav_ws/src
   cd easynav_ws/src

2. Clone, at least, these required repositories for this tutorial:

.. code-block:: bash

   git clone --recursive https://github.com/EasyNavigation/EasyNavigation.git
   git clone https://github.com/EasyNavigation/easynav_simple_stack.git
   git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git

Retrieve all third-party dependencies for simulating kobuki:

.. code-block:: bash

   vcs import . < easynav_playground_kobuki/thirdparty.repos

3. Install dependencies and build the workspace:

.. code-block:: bash

   cd ..  # go back to easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   reset && colcon build --symlink-install
   source install/setup.bash 

Mapping
^^^^^^^

This guide explains how to map an environment using **SLAM Toolbox** and then save the map for later use with **EasyNav**.

SLAM Toolbox belongs to Nav2 and can be installed via:

.. code-block:: bash

  sudo apt install ros-kilted-slam-toolbox

Or from source at: https://github.com/SteveMacenski/slam_toolbox

SLAM Toolbox publishes a `nav_msgs::msg::OccupancyGrid` on the `/map` topic. EasyNav's Simple MapsManager listens on the topic:

.. code-block:: none

  /maps_manager_node/simple/incoming_map

It can directly consume this output. EasyNav also exposes the following service:

.. code-block:: none

  /maps_manager_node/simple/savemap (std_srvs/srv/Trigger)

to save the current map (by default to `/tmp/default.map`).

Step-by-Step Instructions:


1. **Launch the simulator (with or without GUI)**

.. code-block:: bash

  ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

2. **Open RViz2**

.. code-block:: bash

  ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

> Note: The `/map` topic and `map` frame will not appear until SLAM Toolbox is started.

3. **Launch SLAM Toolbox**

By default, SLAM Toolbox subscribes to `/scan`, but in our setup, the laser publishes to `/scan_raw`. Edit or copy the launcher at:

.. code-block:: bash

  /opt/ros/kilted/share/slam_toolbox/launch/online_async_launch.py

and add the following remapping:

.. code-block:: python

  remappings=[
    ('/scan', '/scan_raw'),
  ],

Then run:

.. code-block:: bash

  ros2 launch slam_toolbox online_async_launch.py

4. **Teleoperate the robot to build the map**

.. code-block:: bash

  ros2 run teleop_twist_keyboard teleop_twist_keyboard

Observe the map in RViz2 as it builds.

5. **Launch EasyNav with MapsManager only**

Use the following configuration file (`simple.mapping.params.yaml`) with all other nodes in dummy mode:

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
      map_types: [simple]
      simple:
        freq: 10.0 
        plugin: easynav_simple_maps_manager/SimpleMapsManager

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

Run EasyNav with the following command, remapping the topic to receive the map:

.. code-block:: bash

  ros2 run easynav_system system_main --ros-args \
    --params-file ~/easynav_ws/src/easynav_indoor_testcase/robots_params/simple.mapping.params.yaml \
    -r /maps_manager_node/simple/incoming_map:=/map

6. **Save the generated map**

.. code-block:: bash

  ros2 service call /maps_manager_node/simple/savemap std_srvs/srv/Trigger {{}}

The map will be saved to `/tmp/default.map`.

7. **Rename and move the map to reuse it**

For example:

.. code-block:: bash

  mv /tmp/default.map ~/easynav_ws/src/easynav_indoor_testcase/maps/house_tests.map

Then modify your parameters to use it:

.. code-block:: yaml

  maps_manager_node:
    ros__parameters:
      use_sim_time: true
      map_types: [simple]
      simple:
        freq: 10.0 
        plugin: easynav_simple_maps_manager/SimpleMapsManager
        package: easynav_indoor_testcase
        map_path_file: maps/house_tests.map
