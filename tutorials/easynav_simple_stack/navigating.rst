.. _easynav_simple_stack/navigating:

Navigating with SimpleStack and EasyNav
---------------------------------------


.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/p4aqNA0JNhA" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>

Setup from scratch
^^^^^^^^^^^^^^^^^^

1. Create your workspace if haven't done yet, and move to the `src` directory:

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

4. Do the :doc:`mapping tutorial <./mapping>` to have a map, if you haven't done yet.

Navigating
^^^^^^^^^^

Once the environment has been mapped, save the ``.map`` file in the directory of any package in your workspace.  
This will be required because we will later reference it using the package name and relative path.  
(Optionally, you can provide the absolute path through the parameter ``map_path_file``).

Next, create a parameter file for navigation.  
In this example we will use the **SeReST controller**, **AMCL** for robot localization, and specify that the ``maps_manager`` is an instance of ``easynav_simple_maps_manager/SimpleMapsManager``.  
In the corresponding section we define where the map is located and configure the laser as the sensor to be used:

.. code-block:: yaml

  controller_node:
    ros__parameters:
      use_sim_time: true
      controller_types: [serest]
      serest:
        rt_freq: 30.0 
        plugin: easynav_serest_controller/SerestController
        allow_reverse: true
        max_linear_speed: 0.8
        max_angular_speed: 1.2
        v_progress_min: 0.08        
        k_s_share_max: 0.5         
        k_theta: 2.5                
        k_y: 1.5
        goal_pos_tol: 0.1      
        goal_yaw_tol_deg: 6.0       
        slow_radius: 0.80
        slow_min_speed: 0.02
        final_align_k: 2.5
        final_align_wmax: 0.8
        corner_guard_enable: true
        corner_gain_ey: 1.8
        corner_gain_eth: 0.7
        corner_gain_kappa: 0.4
        corner_min_alpha: 0.35
        corner_boost_omega: 1.0
        a_lat_soft: 0.9
        apex_ey_des: 0.05
  
  localizer_node:
    ros__parameters:
      use_sim_time: true
      localizer_types: [simple]
      simple:
        rt_freq: 50.0
        freq: 5.0
        reseed_freq: 1.0
        plugin: easynav_simple_localizer/AMCLLocalizer
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
  
  maps_manager_node:
    ros__parameters:
      use_sim_time: true
      map_types: [simple]
      simple:
        freq: 10.0 
        plugin: easynav_simple_maps_manager/SimpleMapsManager
        package: easynav_indoor_testcase
        map_path_file: maps/home.map
  
  planner_node:
    ros__parameters:
      use_sim_time: true
      planner_types: [simple]
      simple:
        freq: 0.5
        plugin: easynav_simple_planner/SimplePlanner
        robot_radius: 0.25
  
  sensors_node:
    ros__parameters:
      use_sim_time: true
      forget_time: 0.5
      sensors: [laser1]
      perception_default_frame: odom
      laser1:
        topic: scan_raw
        type: sensor_msgs/msg/LaserScan
        group: points
  
  system_node:
    ros__parameters:
      use_sim_time: true
      position_tolerance: 0.3
      angle_tolerance: 0.15


In one terminal, launch the simulator.  
You can disable the GUI to save resources if needed:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

In a second terminal, launch ``rviz``:

.. code-block:: bash

   rviz2

Finally, in a third terminal, start **EasyNav** and specify the parameter file.  
(Optionally, you can also create a launcher for convenience):

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file /home/fmrico/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/simple.serest_params.yaml

At this point, you can use the **“2D Goal Pose”** button in ``rviz`` to send target positions for the robot to navigate to.
