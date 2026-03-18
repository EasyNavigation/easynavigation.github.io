.. _gps_localization:

==============================
GPS Localization Configuration
==============================

This guide explains how to configure the easynav_fusion_localizer plugin for GPS localization.
Easynav is a real time navigation framework for ROS 2 developed by our university research team, designed to generalize navigation across any environment, including 3D surfaces.
It includes recommended defaults, coordinate frame setups, and practical advice for successful localization.
Once configured, you will be able to send the robot to a waypoint using RViz or perform complex tasks like patrolling using the Easynav behaviours package.


.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/Dqld9yCVcTY" frameborder="0" allowfullscreen></iframe>
    </div>




.. contents:: On this page
   :local:
   :depth: 2

Overview
========

The easynav_fusion_localizer plugin provides a robust way to fuse GPS and odometry data.
It is heavily based on the standard robot_localization package, so the general configuration approach and most of the available parameters are fully described in its official documentation at https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html

Before configuring the localizer, ensure you have created a png image and a yaml configuration file for your costmap or chosen representation.
This is necessary for the planner to work correctly.
If you have not created these files yet, you can use the provided empty file, which represents a map with no nonnavigable zones.

Quick Start Defaults
====================

Start with these default parameters extracted from the example_params_summit_dual_gazebo.yaml file.
This example uses the costmap planner and the serest controller, but you can adjust them to fit your specific robot.

.. code-block:: yaml

    localizer_node:
      ros__parameters:
        use_sim_time: true
        localizer_types: [Ukf]
        dummy:
          rt_freq: 30.0
          plugin: easynav_localizer/DummyLocalizer
          cycle_time_nort: 0.01
          cycle_time_rt: 0.001
        Ukf:
          rt_freq: 50.0
          freq: 5.0
          reseed_freq: 0.1
          plugin: easynav_fusion_localizer/FusionLocalizer
          latitude_origin: 40.284796759605115
          longitude_origin: -3.8214099132625017
          altitude_origin: 669.9730248888955
          
          local_filter:
            two_d_mode: true
            publish_tf: false
            map_frame: map
            odom_frame: odom
            base_link_frame: base_link
            world_frame: odom
            dynamic_process_noise_covariance: true
            
            odom0: /robotnik_base_control/odom
            odom0_config:
              [true,  true, false,
              false, false, true,
              true,  true, false,
              false, false, true,
              false, false, false]
            odom0_differential: false

            process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.025, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]

          global_filter:
            two_d_mode: true
            publish_tf: true
            debug: true
            debug_out_file: /tmp/ukf_global_debug.txt
            transform_timeout: 0.1
            smooth_lagged_data: true
            dynamic_process_noise_covariance: true
            
            map_frame: map
            odom_frame: odom
            base_link_frame: base_link
            world_frame: map
            
            gps0: /gps/fix
            gps0_config:
              [true,  true,  false,
              false, false, false,
              false, false, false,
              false, false, false,
              false, false, false]
            gps0_differential: false
            
            odom0: /robotnik_base_control/odom
            odom0_config:
              [false, false, false,
              false, false, false,
              true,  true,  false,
              false, false, true,
              false, false, false]
            odom0_differential: false
            
            imu0: /imu/data
            imu0_config:
              [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
            imu0_differential: false
            imu0_remove_gravitational_acceleration: true

            process_noise_covariance: [0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.025, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                                      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,   0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015]

            initial_estimate_covariance: [
              5.0, 5.0, 1e-3,   1e-3, 1e-3, 1.0,
              0.5,  0.5,  1e-3,   1e-3, 1e-3, 0.1,
              0.1,  0.1,  0.1
            ]

Core Concepts
=============

* Coordinate Frames: The standard ROS convention is map to odom to base_link. Typically the robot itself publishes the transformation from odom to base_link, while the localization algorithm publishes the transformation from map to odom.
* Filters: The plugin provides two separate filters, local and global.
* GPS Transformation: Unlike the standard robot_localization package, this plugin directly accepts navsatfix GPS messages. These messages are internally transformed into local UTM coordinates based on an origin point of :math:`(0, 0)`.

Detailed Parameter Guidance
===========================

Global Filter Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The global filter requires an absolute localization source. This can be a GPS sensor providing navsatfix messages or another source providing odometry messages.

* Origin Coordinates: You must define the latitude_origin, longitude_origin, and altitude_origin parameters.
* Variables Configuration: It is crucial that your global localization source has the :math:`x` and :math:`y` variables set to true. If you are operating in 3D, :math:`z` must also be true.
* Differential Parameter: The differential parameter for the global source must be set to false.


For the orientation, you have two different options depending on your hardware.

* Reliable absolute orientation source: Set :math:`yaw` to true. If working in 3D, also set :math:`roll` and :math:`pitch` to true. The differential parameter must be set to false.
* Unreliable orientation source: Set :math:`v\_yaw` to true or set the differential parameter to true. To prevent the filter from destabilizing when the robot is stopped, it is highly recommended to set the dynamic_process_noise_covariance parameter to true.



Simulation Demo
===============

In order to test the GPS localization plugin in gazebo simulator, you have to clone the following package:

.. code-block:: bash

    git clone https://github.com/EasyNavigation/easynav_playground_summit.git

Then, follow the installation instructions in the package documentation.

.. code-block:: bash

    cd <easynav-workspace>/src/
    vcs import . < easynav_playground_summit/thirdparty.repos
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install 


Then, you can use the following launch file:

.. code-block:: bash

    ros2 launch easynav_playground_summit playground_summit.launch.py

And run easynav loading the parameters from the fusion_localizer package:

.. code-block:: bash

    ros2 run easynav_system system_main --ros-args --params-file easynav_plugins/localizers/easynav_fusion_localizer/config/example_params_summit_dual_gazebo.yaml


After that, you can open rviz2 and check the output:

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/grEapN47KW8" frameborder="0" allowfullscreen></iframe>
    </div>


Detailed Parameter Explanations
===============================

Although the easynav_fusion_localizer is based on the standard robot_localization package, the core parameters are explained below for clarity and quick reference.

Sensor Topics: [sensor]
^^^^^^^^^^^^^^^^^^^^^^^

For each sensor, users need to define this parameter based on the message type. For example, if we define one source of Imu messages and two sources of Odometry messages, the configuration in your YAML file would look like this:

.. code-block:: yaml

    # Example defining one IMU and two Odometry sources
    imu0: robot/imu/data
    odom0: wheel_encoder/odometry
    odom1: visual_odometry/odometry

The index for each parameter name is 0-based (e.g., odom0, odom1, etc.) and must be defined sequentially. Do not use pose0 and pose2 if you have not defined pose1. The values for each parameter are the exact topic names for those sensors.

Sensor Configuration: [sensor]_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For each of the sensor messages defined above, users must specify which variables of those messages should be fused into the final state estimate. An example odometry configuration looks like this:

.. code-block:: yaml

    # Fusing X, Y, yaw, X velocity, and yaw velocity
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

The order of the boolean values is :math:`X`, :math:`Y`, :math:`Z`, :math:`roll`, :math:`pitch`, :math:`yaw`, :math:`\dot{X}`, :math:`\dot{Y}`, :math:`\dot{Z}`, :math:`\dot{roll}`, :math:`\dot{pitch}`, :math:`\dot{yaw}`, :math:`\ddot{X}`, :math:`\ddot{Y}`, :math:`\ddot{Z}`. 

*Note:* The specification is done in the frame_id of the sensor, not in the world_frame or base_link_frame. 

Queue Size: [sensor]_queue_size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Users can use these parameters to adjust the callback queue sizes for each sensor (e.g., odomN_queue_size, imuN_queue_size). This is highly useful if your filter frequency parameter value is much lower than your sensor's frequency, as it allows the algorithm to incorporate all measurements that arrived in between update cycles.

Differential Mode: [sensor]_differential
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For each of the sensor messages containing pose information, users can specify whether the pose variables should be integrated differentially. 

If a given value is set to true, then for a measurement at time :math:`t` from the sensor in question, the filter first subtracts the measurement at time :math:`t-1`, and converts the resulting value to a velocity. This setting is especially useful if your robot has two sources of absolute pose information (e.g., yaw measurements from odometry and an IMU). If the variances on the input sources are not configured correctly, these measurements may get out of sync and cause oscillations. Integrating one or both differentially avoids this scenario.

Users should take care when using this parameter for orientation data, as the conversion to velocity means that the covariance for orientation state variables will grow without bound, unless another source of absolute orientation data is being fused. 

*Note:* If you are fusing GPS information via navsat_transform_node or utm_transform_node, you should make sure that the differential setting is false. If you simply want all of your pose variables to start at :math:`0`, please use the relative parameter instead.

Relative Mode: [sensor]_relative
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If this parameter is set to true, any measurements from this sensor will be fused relative to the first measurement received from that sensor. This is useful if you want your state estimate to always start at :math:`(0,0,0)` with :math:`roll`, :math:`pitch`, and :math:`yaw` values of :math:`(0,0,0)`. It is similar to the differential parameter, but instead of removing the measurement at time :math:`t-1`, it always removes the measurement at time :math:`0`, and the measurement is not converted to a velocity.

Gravitational Acceleration
^^^^^^^^^^^^^^^^^^^^^^^^^^

* **imuN_remove_gravitational_acceleration:** If fusing accelerometer data from IMUs, this parameter determines whether or not acceleration due to gravity is removed from the acceleration measurement before fusing it. This assumes that the IMU providing the acceleration data is also producing an absolute orientation, which is strictly required to correctly remove gravitational acceleration.
* **gravitational_acceleration:** If the previous parameter is set to true, this defines the acceleration in :math:`Z` due to gravity that will be removed from the IMU's linear acceleration data. The default value is 9.80665 m/s².

Initial State
^^^^^^^^^^^^^

Starts the filter with the specified state. The state is given as a 15-D vector of doubles, following the same variable order as the sensor configurations. For example, to start your robot at a position of :math:`(5.0, 4.0, 3.0)`, a :math:`yaw` of 1.57, and a linear velocity of :math:`(0.1, 0.2, 0.3)`, you would use:

.. code-block:: yaml

    # Initial state configuration vector
    initial_state: [5.0,  4.0,  3.0,
                    0.0,  0.0,  1.57,
                    0.1,  0.2,  0.3,
                    0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0]

General System Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^

* **publish_tf:** If true, the state estimation node will publish the transform from the frame specified by the world_frame parameter to the frame specified by the base_link_frame parameter. Defaults to true.
* **publish_acceleration:** If true, the state estimation node will publish the linear acceleration state. Defaults to false.
* **permit_corrected_publication:** When the state estimation nodes publish the state at time :math:`t`, but then receive a measurement with a timestamp :math:`< t`, they re-publish the corrected state with the same time stamp as the previous publication. Setting this parameter to false disables that behavior. Defaults to false.
* **print_diagnostics:** If true, the state estimation node will publish diagnostic messages to the /diagnostics topic. This is useful for debugging your configuration and sensor data.
* **two_d_mode:** If your robot is operating in a planar environment and you are comfortable with ignoring the subtle variations in the ground reported by an IMU, set this to true. It will fuse :math:`0` values for all 3D variables (:math:`Z`, :math:`roll`, :math:`pitch`, and their respective velocities and accelerations). This keeps the covariances for those values from exploding while ensuring that your robot's state estimate remains affixed to the X-Y plane.



Recommended Setup Order
=======================

1. Verify prerequisites: Ensure you have a valid map image and configuration file.
2. Check coordinate frames: Determine if your robot publishes the odom to base_link transformation to decide whether to use only the global filter or both.
3. Set GPS origin: Accurately configure your starting latitude, longitude, and altitude.
4. Configure each sensor you want to use in the filter


Satellite Map Visualization in RViz (Optional)
==============================================

Visualizing the satellite map directly in RViz provides an excellent way to verify that your GPS localization is working correctly within the Easynav framework. 

Installation
^^^^^^^^^^^^

First, you need to install the satellite plugin for RViz. Since Easynav operates on ROS 2, make sure to install the package corresponding to your distribution. For this example, we will use the Jazzy release.

.. code-block:: bash

    sudo apt install ros-jazzy-rviz-satellite

Map Server Setup
^^^^^^^^^^^^^^^^

Next, launch a Docker container that will act as a local map server using MapProxy. This server will provide the satellite map tiles directly to your RViz session.

.. code-block:: bash

    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

RViz Configuration
^^^^^^^^^^^^^^^^^^

Once the plugin is installed and the map server is actively running, open your RViz interface and add the AerialMap display type. Configure the properties exactly as follows to see the map:

* Topic: Set this field to /gps/filtered
* Object URI: Enter http://localhost:8080/wmts/gm_layer/gm_grid/{z}/{x}/{y}.png
* Zoom: A value of 19 is highly recommended for optimal clarity.




Troubleshooting by Symptom
==========================

Filter does not work or destabilizes
^^^^^^^^^^^^^^^^^^^^^^^^^^^
Verify that all the frames appearing in the headers of the filtered messages have a valid transformation to the robot base frame.

Ensure that you have at least one source of absolut localization.

Simulator or bag playback timing issues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Ensure that the use_sim_time parameter is set to false when running on a real robot and to true when running in a simulator. When playing a recorded bag, you must also set use_sim_time to true and pass the clock argument to the playback command.

Filter destabilizes when the robot is stopped
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you are using an unreliable orientation source, make sure the dynamic_process_noise_covariance parameter is set to true to avoid unpredictable behavior when stationary.