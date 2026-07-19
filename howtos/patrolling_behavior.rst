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
monitor their completion, and reset the navigation state to repeat missions, using the
``GoalManagerClient`` interface exposed by ``easynav_system``.

.. warning::

   There is currently **no ready-made "patrolling behavior" package** (``easynav_behaviors``,
   ``easynav_patrolling_behavior``, ``easynav_patrolling_behavior_py``) shipped in this workspace.
   What *does* exist and is real is the client-side goal API:

   - C++: ``GoalManagerClient`` in ``easynav_system``
     (``easynav_system/include/easynav_system/GoalManagerClient.hpp``).
   - Python: ``GoalManagerClient`` / ``ClientState`` in the ``easynav_goalmanager_py`` module,
     part of the ``easynav_support_py`` package.

   This page describes how to use that real API to build a patrolling node yourself. Treat the
   code below as a pattern to implement in your own package, not as an existing executable you can
   ``ros2 run``.

---

Setup
-----

Before starting, make sure you have completed the installation instructions in :doc:`../build_install/index`.

Make sure ``easynav_indoor_testcase`` (for the simulation/config used below) and the core
``EasyNavigation`` packages (which provide ``easynav_system`` and ``easynav_support_py``) are
present in your workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git

Build and source the workspace:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash

---

Waypoint Configuration
----------------------

A common pattern is to define waypoints in a YAML file under the ``config`` directory of your own
patrolling package. Each waypoint specifies its position (``x``, ``y``) and orientation
(``yaw`` in radians).

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

Building the Patrolling Behavior
---------------------------------

As noted above, there is no bundled ``patrolling_main`` executable or launch file in the current
codebase. Instead, you write a small ROS 2 node (C++ or Python) in your own package that:

1. Loads the waypoints YAML shown above.
2. Wraps a ``GoalManagerClient`` around your node.
3. Converts each waypoint into a ``geometry_msgs/msg/PoseStamped`` and sends them all as a
   ``nav_msgs/msg/Goals`` via ``send_goals()``.
4. Polls ``get_state()`` until the sequence finishes, then ``reset()``\ s and repeats.

You would then run your own node/launch file, e.g.:

.. code-block:: bash

   ros2 run my_patrol_pkg patrolling_main --ros-args --params-file config/patrol_points.yaml

or, in Python, build on the ``GoalManagerClient``/``ClientState`` classes importable from
``easynav_goalmanager_py`` (part of ``easynav_support_py``).

---

Code Explanation (C++ Version)
------------------------------

Below is an overview of the core logic you would write against the real
``easynav_system::GoalManagerClient`` API (``easynav_system/include/easynav_system/GoalManagerClient.hpp``).

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

The returned value indicates the current status of navigation. The real ``GoalManagerClient::State``
enum (see ``GoalManagerClient.hpp``) has more granularity than a simple idle/running/success/failed
set — it tracks the request/accept handshake explicitly:

.. code-block:: cpp

   switch (nav_state)
   {
     case GoalManagerClient::State::IDLE:
       // No active navigation
       break;

     case GoalManagerClient::State::SENT_GOAL:
     case GoalManagerClient::State::SENT_PREEMPT:
       // Goal(s) sent, waiting for the system to accept/reject them
       break;

     case GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
       // Currently navigating toward the goal(s)
       break;

     case GoalManagerClient::State::NAVIGATION_FINISHED:
       // The goal sequence was reached successfully
       break;

     case GoalManagerClient::State::NAVIGATION_REJECTED:
     case GoalManagerClient::State::NAVIGATION_FAILED:
     case GoalManagerClient::State::NAVIGATION_CANCELLED:
     case GoalManagerClient::State::ERROR:
       // The navigation failed, was rejected, was cancelled, or errored out
       break;
   }

This allows the behavior to detect when all waypoints are completed, or when to retry a goal if it fails.
``reset()`` may only be called from one of the terminal states (``NAVIGATION_FINISHED``,
``NAVIGATION_REJECTED``, ``NAVIGATION_FAILED``, ``NAVIGATION_CANCELLED``, ``ERROR``); calling it from
``IDLE``/``SENT_GOAL``/``SENT_PREEMPT``/``ACCEPTED_AND_NAVIGATING`` is logged as an error and ignored.

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
- The C++ (``easynav_system::GoalManagerClient``) and Python (``easynav_goalmanager_py``) client
  APIs mirror the same request/accept/feedback/result protocol, so behavior nodes written against
  either one follow the same state machine.
- The YAML file defines the patrol route; it can be edited live or generated from recorded positions.
- Ensure that all waypoints are reachable within the current map and costmap configuration.

---

With this setup, the robot will continuously patrol between the defined waypoints, showcasing how EasyNav behaviors can coordinate higher-level missions on top of the navigation stack.
