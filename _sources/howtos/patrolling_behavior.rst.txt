.. _patrolling_behavior:

===================
Patrolling Behavior
===================

This HowTo demonstrates how to create a **patrolling behavior** using EasyNav’s Costmap-based navigation stack.  
The robot repeatedly navigates through a predefined list of waypoints, automatically looping back to the first point when the sequence finishes.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/yN80Dqan5rE" frameborder="0" allowfullscreen></iframe>
    </div>

The **Patrolling Behavior** shows how to send navigation commands programmatically,  
monitor their completion, and reset the navigation state to repeat missions.  
Both **C++** and **Python** versions are available in the `easynav_behaviors` repository.

---

Setup
-----

Before starting, make sure you have completed the installation instructions in :doc:`../build_install/index`.

Then clone the following repositories into your workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git
   git clone https://github.com/EasyNavigation/easynav_behaviors.git

Build and source the workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

---

Waypoint Configuration
----------------------

Waypoints are defined in a YAML file under the ``config`` directory of the behavior package.  
Each waypoint specifies its position (``x``, ``y``) and orientation (``yaw`` in radians).

Example (``config/patrol_points.yaml``):

.. code-block:: yaml

   waypoints:
     - { x: 1.5, y: 0.0, yaw: 0.0 }
     - { x: 3.0, y: 1.2, yaw: 1.57 }
     - { x: 2.0, y: 3.5, yaw: 3.14 }
     - { x: 0.5, y: 1.5, yaw: -1.57 }

You can modify this list to create your own patrol routes.

---

Launching Navigation
--------------------

Before starting the patrol, launch the Costmap-based navigation stack.

**Terminal 1 – Simulator:**

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

**Terminal 2 – Visualization:**

.. code-block:: bash

   ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

**Terminal 3 – EasyNav system:**

.. code-block:: bash

   ros2 run easynav_system system_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap.serest.params.yaml

---

Running the Patrolling Behavior
-------------------------------

You can run the patrolling behavior in **C++** or **Python**. Both implementations use the same configuration.

### 🧩 C++ Version

.. code-block:: bash

   ros2 run easynav_patrolling_behavior patrolling_main \
     --ros-args --params-file ~/ros/ros2/easynav_ws/src/easynav_behaviors/config/patrol_points.yaml

### 🐍 Python Version

.. code-block:: bash

   ros2 launch easynav_patrolling_behavior_py patrolling.launch.py

Both versions will read the YAML file, create navigation goals for each waypoint, and start cyclic navigation.

---

Code Explanation (C++ Version)
------------------------------

Below is an overview of the core logic of the **C++ implementation** in `easynav_patrolling_behavior`.

### 1. Creating the GoalManagerClient

The `GoalManagerClient` provides an interface to send and monitor navigation goals managed by the EasyNav system.

.. code-block:: cpp

   gm_client_ = GoalManagerClient::make_shared(shared_from_this());

This initializes the client and associates it with the current ROS 2 node.

---

### 2. Creating and Sending Goals

In the `initialize()` method, the YAML file is parsed and each waypoint is converted into a `geometry_msgs::msg::PoseStamped`.  
These poses are stored inside a `nav_msgs::msg::Goals` message, which represents the full navigation sequence.

.. code-block:: cpp

   goals_.goals.clear();

   for (const auto & wp : waypoints_) {
     geometry_msgs::msg::PoseStamped pose;
     pose.header.frame_id = "map";
     pose.pose.position.x = wp.x;
     pose.pose.position.y = wp.y;
     pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, wp.yaw));
     goals_.goals.push_back(pose);
   }

   gm_client_->send_goals(goals_);

The call to `send_goals()` sends the list of waypoints to the navigation system.  
The robot will automatically move through each waypoint in order.

---

### 3. Monitoring Navigation State

The navigation state can be checked at any time using:

.. code-block:: cpp

   auto nav_state = gm_client_->get_state();

The returned value indicates the current status of navigation.  
A typical `switch` block may look like this:

.. code-block:: cpp

   switch (nav_state)
   {
     case GoalManagerClient::State::IDLE:
       // No active navigation
       break;

     case GoalManagerClient::State::RUNNING:
       // Currently navigating toward a goal
       break;

     case GoalManagerClient::State::SUCCESS:
       // The goal was reached successfully
       break;

     case GoalManagerClient::State::FAILED:
       // The navigation failed or was aborted
       break;
   }

This allows the behavior to detect when all waypoints are completed, or when to retry a goal if it fails.

---

### 4. Resetting the Navigation State

Before sending new goals, it is recommended to reset the GoalManager to clear any previous navigation state:

.. code-block:: cpp

   gm_client_->reset();

This ensures the next sequence starts from a clean state.

---

Notes
-----

- The **patrolling behavior** is a simple example of commanding navigation goals programmatically.  
  It can be extended to perform inspection, delivery, or monitoring tasks.
- Both the C++ and Python implementations use the same `GoalManager` interface, so they behave identically.
- The YAML file defines the patrol route; it can be edited live or generated from recorded positions.
- Ensure that all waypoints are reachable within the current map and costmap configuration.

---

With this setup, the robot will continuously patrol between the defined waypoints, showcasing how EasyNav behaviors can coordinate higher-level missions on top of the navigation stack.
