.. _patrolling_behavior:

===================
Patrolling Behavior
===================

This HowTo demonstrates how to run a **patrolling behavior** on top of EasyNav's navigation stack.
The robot repeatedly navigates through a predefined list of waypoints, automatically looping back
to the first point when the sequence finishes.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

.. raw:: html

    <div align="center">
      <iframe width="450" height="300" src="https://www.youtube.com/embed/yN80Dqan5rE" frameborder="0" allowfullscreen></iframe>
    </div>

The **Patrolling Behavior** shows how to send navigation goals programmatically, monitor their
completion, and reset the navigation state to repeat missions, using the ``GoalManagerClient``
interface exposed by ``easynav_system``. It ships as a real, ready-to-run package, in two
equivalent flavors, both under the ``easynav_behaviors`` repository:

- **C++**: ``easynav_patrolling_behavior`` — a plain ``rclcpp::Node`` and a ``patrolling_main``
  executable.
- **Python**: ``easynav_patrolling_behavior_py`` — the same behavior built on ``rclpy`` and the
  ``easynav_goalmanager_py`` client module.

Both implementations poll the same underlying client API and follow the same state machine; pick
whichever language fits your own application.

---

Setup
-----

Before starting, complete the installation steps in :doc:`../build_install/index`
(any of APT, Pixi or source). The navigation stack launched below
(``costmap.serest.params.yaml``) uses the **SeReST Controller**, **Costmap
Localizer**, **Costmap Maps Manager** and **Costmap Planner** plugins, which the
core ``easynav`` package does not include:

- **APT**:

  .. code-block:: bash

     sudo apt install \
       ros-<distro>-easynav-serest-controller \
       ros-<distro>-easynav-costmap-localizer \
       ros-<distro>-easynav-costmap-maps-manager \
       ros-<distro>-easynav-costmap-planner

- **Pixi**:

  .. code-block:: bash

     pixi add \
       ros-<distro>-easynav-serest-controller \
       ros-<distro>-easynav-costmap-localizer \
       ros-<distro>-easynav-costmap-maps-manager \
       ros-<distro>-easynav-costmap-planner

- **Source**: already built if you cloned ``easynav_plugins`` as described in
  :ref:`build_from_source`.

``easynav_behaviors`` (which provides both patrolling packages), ``easynav_indoor_testcase``
(simulation/config used below) and ``easynav_playground_kobuki`` (the simulator launched
below) are demo/example content and only distributed as source — clone them into
``~/easynav_ws/src`` regardless of install method:

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone https://github.com/EasyNavigation/easynav_behaviors.git
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git
   git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git

Build and source the workspace as described in :ref:`gs_source_workspace`:

.. code-block:: bash

   cd ~/easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install

---

Waypoint Configuration
----------------------

Both packages read the same parameter shape. Waypoints are declared as a list of **names** under
``waypoints:``, and each name is itself a parameter holding a 3-element ``[x, y, yaw]`` array
(``yaw`` in radians) — not a list of ``{x, y, yaw}`` maps.

Real, shipped example (``easynav_patrolling_behavior/config/patrolling_params.yaml``):

.. code-block:: yaml

   patrolling_node:
     ros__parameters:
       use_sim_time: true
       frame_id: map
       waypoints: [wp1, wp2, wp3, wp4, wp5]
       wp1: [4.23, -3.0, 0.0]
       wp2: [-1.47, -4.08, 1.57]
       wp3: [-6.76, -3.01, 3.14]
       wp4: [-7.36, -0.38, 0.0]
       wp5: [0.0, 0.5, 0.0]

The Python package ships its own, slightly different example under
``easynav_patrolling_behavior_py/config/patrolling_params.yaml`` (same shape, different waypoints,
and ``use_sim_time: false`` by default). Edit either file's ``waypoints:`` list and the
corresponding ``wpN:`` entries to define your own route.

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
     --ros-args --params-file ~/easynav_ws/src/easynav_indoor_testcase/robots_params/costmap.serest.params.yaml

---

Running the Patrolling Behavior
---------------------------------

**C++ version.** ``easynav_patrolling_behavior`` builds a ``patrolling_main`` executable, but does
**not** currently ship a working launch file (its ``launch/`` directory only contains an empty
placeholder file). Run it directly with its own parameter file:

.. code-block:: bash

   ros2 run easynav_patrolling_behavior patrolling_main \
     --ros-args --params-file ~/easynav_ws/src/easynav_behaviors/easynav_patrolling_behavior/config/patrolling_params.yaml

**Python version.** ``easynav_patrolling_behavior_py`` does ship a launch file that loads its own
config automatically:

.. code-block:: bash

   ros2 launch easynav_patrolling_behavior_py patrolling.launch.py

---

Code Explanation (C++ Version)
------------------------------

The C++ node (``easynav_patrolling_behavior/src/easynav_patrolling_behavior/PatrollingNode.cpp``)
runs its own simple state machine — ``PatrolState {IDLE, PATROLLING, FINISHED, ERROR}`` — on a
100 ms timer, layered on top of the real ``easynav_system::GoalManagerClient`` API
(``easynav_system/include/easynav_system/GoalManagerClient.hpp``).

1. Loading waypoints
~~~~~~~~~~~~~~~~~~~~~

On the first tick, ``initialize()`` reads ``waypoints`` and ``frame_id``, then reads each named
waypoint's ``[x, y, yaw]`` array and converts it into a ``geometry_msgs::msg::PoseStamped``,
appended to a ``nav_msgs::msg::Goals``:

.. code-block:: cpp

   for (const auto & wp : waypoints) {
     std::vector<double> wp_coord;
     declare_parameter(wp, wp_coord);
     get_parameter(wp, wp_coord);

     geometry_msgs::msg::PoseStamped wp_pose;
     wp_pose.header.frame_id = frame_id_;
     wp_pose.pose.position.x = wp_coord[0];
     wp_pose.pose.position.y = wp_coord[1];
     wp_pose.pose.orientation = orientationAroundZAxis(wp_coord[2]);
     goals_.goals.push_back(wp_pose);
   }

2. The state machine
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   switch (state_) {
     case PatrolState::IDLE:
       if (!initialized_) {
         gm_client_ = GoalManagerClient::make_shared(shared_from_this());
         initialize();
         initialized_ = true;
       }
       gm_client_->send_goals(goals_);
       state_ = PatrolState::PATROLLING;
       break;

     case PatrolState::PATROLLING: {
       auto nav_state = gm_client_->get_state();
       switch (nav_state) {
         case GoalManagerClient::State::NAVIGATION_REJECTED:
         case GoalManagerClient::State::NAVIGATION_FAILED:
         case GoalManagerClient::State::NAVIGATION_CANCELLED:
         case GoalManagerClient::State::ERROR:
           state_ = PatrolState::ERROR;
           break;
         case GoalManagerClient::State::NAVIGATION_FINISHED:
           state_ = PatrolState::FINISHED;
           break;
         default:
           break;  // still SENT_GOAL / ACCEPTED_AND_NAVIGATING: keep waiting
       }
       break;
     }

     case PatrolState::FINISHED:
       gm_client_->reset();
       state_ = PatrolState::IDLE;  // loops back: goals are re-sent next tick
       break;

     case PatrolState::ERROR:
       break;  // terminal: no automatic recovery
   }

Note the node's own ``PatrolState`` (a simple IDLE/PATROLLING/FINISHED/ERROR mission-level status)
is distinct from ``GoalManagerClient::State``, the finer-grained request/accept/feedback protocol
described in :ref:`commanding`. ``PatrollingNode`` polls the latter to drive the former. Also note
that reaching ``PatrolState::ERROR`` is a dead end in the current implementation — there is no
automatic retry; you would need to extend ``cycle()`` yourself (e.g. transition back to ``IDLE``)
if you want the patrol to recover from a failed/rejected/cancelled goal instead of stopping.

---

Code Explanation (Python Version)
------------------------------------

``easynav_patrolling_behavior_py/patrolling_node.py`` mirrors the same four-state machine using
``easynav_goalmanager_py``'s ``GoalManagerClient``/``ClientState``, on a 0.2 s timer. Two details
differ from the C++ version:

- In ``PatrolState.IDLE``, it keeps **resending** ``send_goals()`` on every tick until the state
  becomes ``ClientState.ACCEPTED_AND_NAVIGATING``, rather than sending once and immediately moving
  to ``PATROLLING``.
- In ``PatrolState.FINISHED``, it calls ``reset()`` repeatedly until ``get_state()`` reports
  ``ClientState.IDLE``, only then moving back to ``IDLE`` itself — instead of resetting once and
  assuming it succeeded.

.. code-block:: python

   match self._state:
       case PatrolState.IDLE:
           nav_state = self._gm.get_state()
           if nav_state != ClientState.ACCEPTED_AND_NAVIGATING:
               self._goals.header.stamp = self.get_clock().now().to_msg()
               self._gm.send_goals(self._goals)
           else:
               self._state = PatrolState.PATROLLING
       case PatrolState.PATROLLING:
           nav_state = self._gm.get_state()
           if nav_state in (ClientState.NAVIGATION_REJECTED, ClientState.NAVIGATION_FAILED,
                            ClientState.NAVIGATION_CANCELLED, ClientState.ERROR):
               self._state = PatrolState.ERROR
           elif nav_state == ClientState.NAVIGATION_FINISHED:
               self._state = PatrolState.FINISHED
       case PatrolState.FINISHED:
           if self._gm.get_state() != ClientState.IDLE:
               self._gm.reset()
           else:
               self._state = PatrolState.IDLE
       case PatrolState.ERROR:
           pass  # terminal here too

---

Notes
-----

- The **patrolling behavior** is a simple example of commanding navigation goals
  programmatically. It can be extended to perform inspection, delivery, or monitoring tasks.
- The C++ (``easynav_system::GoalManagerClient``) and Python (``easynav_goalmanager_py``) client
  APIs mirror the same request/accept/feedback/result protocol, so both behavior nodes follow the
  same underlying state machine even though their own IDLE/FINISHED handling loops differ
  slightly (see above).
- ``reset()`` may only be called from one of the terminal ``GoalManagerClient`` states
  (``NAVIGATION_FINISHED``, ``NAVIGATION_REJECTED``, ``NAVIGATION_FAILED``,
  ``NAVIGATION_CANCELLED``, ``ERROR``); calling it earlier is logged as an error and ignored.
- The YAML file defines the patrol route; it can be edited live or generated from recorded
  positions.
- Ensure that all waypoints are reachable within the current map and costmap configuration.
- The Python package's ``package.xml`` declares an ``exec_depend`` on ``easynav_goalmanager_py``,
  but that is the *Python import name*, not a separate ROS package — the actual package providing
  it is ``easynav_support_py``. If ``rosdep``/``colcon`` complain about a missing
  ``easynav_goalmanager_py`` package, make sure ``easynav_support_py`` is built and sourced.

---

With this setup, the robot will continuously patrol between the defined waypoints, showcasing how EasyNav behaviors can coordinate higher-level missions on top of the navigation stack.
