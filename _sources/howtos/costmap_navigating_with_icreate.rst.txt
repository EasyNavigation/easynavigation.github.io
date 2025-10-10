.. _costmap_navigating_with_icreate:

==========================================
Deploying EasyNav on a Real iCreate3 Robot
==========================================

This HowTo explains how to deploy the **Costmap Stack** of EasyNavigation (EasyNav) on a real **iRobot iCreate3** robot,
using a Raspberry Pi 4 as on-board computer and ROS 2 Kilted.  
It is based on the same workflow as the *Simple Stack* tutorials but adapted for **real hardware** with a graded
**Costmap2D** environment representation.

.. contents:: On this page
   :local:
   :depth: 2

Setup
------

Before starting, ensure that:

1. You have completed the EasyNav installation steps in :doc:`../build_install/index`.  
2. You have a workspace on the Raspberry Pi (for example ``~/easynav_ws``) and it is sourced correctly.
3. The following repositories are cloned inside your ``src/`` folder:

   - ``EasyNavigation``
   - ``easynav_plugins``
   - ``easynav_indoor_testcase`` *(for maps and configuration examples)*
   - ``easynav_costmap_stack`` *(for the costmap-based stack)*
   - ``sllidar_ros2`` *(LIDAR driver)*

4. Your robot and laptop can communicate over the same Wi-Fi network.

---

Hardware Setup
--------------

.. image:: ../images/base_icreate.jpg
   :alt: iCreate3 base
   :align: center
   :class: shadow rounded

.. note::
   The overall workflow mirrors the one in the Simple Stack tutorials, but here you will run on **real hardware**.
   The navigation stack represents the environment as a graded **Costmap2D**.

- **Base:** `iRobot iCreate3 <https://edu.irobot.com/create3-setup>`_  
  Follow the vendor instructions to connect to Wi-Fi and update firmware.  
  We used the latest ROS 2 Iron firmware with FastDDS:  
  https://edu.irobot.com/create3-latest-fw

- **Laser:** `RPLidar S2 <https://www.slamtec.com/en/S2>`_ mounted on top of the base.

.. image:: ../images/rpi_icreate.jpg
   :alt: Raspberry Pi 4 on iCreate3
   :align: center
   :class: shadow rounded

- **On-board computer:** Raspberry Pi 4 Model B powered from the base via USB-C.  
  Ethernet-over-USB-C was not available, so communication occurs over Wi-Fi.  
  The LIDAR connects via USB.  
  The Raspberry Pi runs **Ubuntu 24.04 Desktop** and **ROS 2 Kilted**.

- **Operator laptop:** Runs ``rviz2`` and connects to the Raspberry Pi via SSH to launch processes.

---

ROS 2 Setup on the Raspberry Pi
-------------------------------

Follow the official ROS 2 Kilted installation steps:  
https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html

**Quick summary:**

1. Enable the *universe* repository:

   .. code-block:: bash

      sudo apt install -y software-properties-common
      sudo add-apt-repository -y universe

2. Configure ROS 2 APT repositories:

   .. code-block:: bash

      sudo apt update && sudo apt install -y curl
      export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
      curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
      sudo dpkg -i /tmp/ros2-apt-source.deb

3. Install development tools and ROS 2 Desktop:

   .. code-block:: bash

      sudo apt update && sudo apt install -y ros-dev-tools
      sudo apt upgrade -y
      sudo apt install -y ros-kilted-desktop

4. Install additional packages:

   .. code-block:: bash

      sudo apt install -y openssh-server \
          ros-kilted-rplidar-ros \
          ros-kilted-depthai ros-kilted-depthai-ros ros-kilted-depthai-ros-driver ros-kilted-depthai-bridge \
          ros-kilted-slam-toolbox \
          ros-kilted-rmw-zenoh-cpp ros-kilted-zenoh-cpp-vendor \
          ros-kilted-tf2-ros ros-kilted-tf2-py ros-kilted-tf2-tools \
          ros-kilted-irobot-create-msgs

---

EasyNav Setup on the Raspberry Pi
---------------------------------

1. **Create the workspace:**

   .. code-block:: bash

      mkdir -p ~/easynav_ws/src
      cd ~/easynav_ws/src

2. **Clone the required repositories:**

   .. code-block:: bash

      git clone https://github.com/EasyNavigation/easynav_plugins.git
      git clone https://github.com/EasyNavigation/easynav_costmap_stack.git
      git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git
      git clone --recursive https://github.com/EasyNavigation/EasyNavigation.git
      git clone https://github.com/Slamtec/sllidar_ros2.git

3. **Install dependencies:**

   .. code-block:: bash

      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace:**

   .. code-block:: bash

      colcon build --symlink-install

5. **Source automatically in ``~/.bashrc``:**

   .. code-block:: bash

      echo 'source /opt/ros/kilted/setup.bash' >> ~/.bashrc
      echo 'source ~/easynav_ws/install/setup.bash' >> ~/.bashrc
      source ~/.bashrc

---

Mapping
-------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/2XD2wkrfFR8" frameborder="0" allowfullscreen></iframe>
    </div>

1. Verify that both the **laptop** and the **Raspberry Pi** can see the iCreate3 topics:

   .. code-block:: bash

      ros2 topic list

   You should see topics such as ``/odom``, ``/tf``, ``/tf_static``, ``/battery_state`` and others published by the base.

2. Publish a static transform from the base to the laser:

   .. code-block:: bash

      ros2 run tf2_ros static_transform_publisher \
        --x 0.02 --z 0.22 --yaw 3.14 \
        --frame-id base_link --child-frame-id laser

   Keep this process running during mapping and navigation.

3. Start the laser driver:

   .. code-block:: bash

      ros2 launch sllidar_ros2 sllidar_s2_launch.py

4. Place the robot at the desired origin (``0, 0``) and start **SLAM Toolbox**:

   .. code-block:: bash

      ros2 launch slam_toolbox online_async_launch.py

5. On the **laptop**, open **RViz2**:

   .. code-block:: bash

      rviz2

6. Start teleoperation to drive the robot while mapping:

   .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard

7. When the map is complete, save it:

   .. code-block:: bash

      ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap

8. Store the generated ``.yaml`` and image file (``.pgm``/``.png``) under  
   ``~/easynav_ws/src/easynav_indoor_testcase/maps``.  
   If you rename the map, ensure the YAML’s image field matches.

.. tip::
   To reset the ``odom → base_footprint`` transform on iCreate3, call:

   .. code-block:: bash

      ros2 service call /reset_pose irobot_create_msgs/srv/ResetPose

---

Navigation
----------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/MssaLixuv2g" frameborder="0" allowfullscreen></iframe>
    </div>

Repeat steps **(2)** and **(3)** from *Mapping* if the transform publisher or laser driver were closed.

Verify the parameter file at:  
``~/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap.serest.params.yaml``

Ensure that:
- ``map_path_file`` points to your saved map (e.g., ``maps/casa.yaml``)
- The **filter plugin names** are correct (see *Notes* below)

Start EasyNav on the Raspberry Pi:

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file ~/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap.serest.params.yaml

---

Notes
-----

- In the **maps manager**, filter plugin types must match the full class names:  
  ``easynav_costmap_maps_manager/ObstacleFilter`` and  
  ``easynav_costmap_maps_manager/InflationFilter``.
- The **Costmap Planner** supports parameters such as  
  ``cost_factor``, ``cost_axial``, ``cost_diagonal``, ``inflation_penalty`` and ``continuous_replan``.  
  Adjust these values to fine-tune planning behavior for your environment.
