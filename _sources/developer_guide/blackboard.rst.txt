.. _blackboard:

How to use the NavState BlackBoard
==================================

The `NavState` class is a central component in the EasyNav architecture. It implements a **shared blackboard**, which allows all modules (localizers, planners, controllers, etc.) to **read and write shared data without using ROS 2 communication mechanisms** internally. This design improves **determinism**, simplifies debugging, and avoids unnecessary overhead.

Overview
--------

All information needed or produced by EasyNav modules flows through the `NavState`. Examples include:

- the robot's estimated pose (`robot_pose`),
- a list of navigation goals (`goals`),
- a path to follow (`path`),
- environment maps (e.g., `map`, `map.dynamic`, `map.static`),
- control commands (`cmd_vel`),
- perception data (`points`, `image`, etc.).

The `NavState` provides a simple API to check for, retrieve, and update entries using keys.

Basic API
---------

To **check whether a value is available**, use:

.. code-block:: cpp

   if (nav_state.has("robot_pose")) {
     // do something
   }

To **read a value**, use the templated `get` method. It is recommended to use
``const auto &`` to avoid unnecessary copies:

.. code-block:: cpp

   const auto & odom = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

To **write a value**, use:

.. code-block:: cpp

   nav_state.set("cmd_vel", computed_twist);

Values are stored under a string key and must be copyable. You can store standard ROS messages, custom types, or even nested structures. Keys can use dot notation (e.g., `"map.static"`, `"map.dynamic"`).

Examples from Plugins
---------------------

**Planner Example**

A planner typically requires the robot’s pose, a goal, and a map. It writes back a path:

.. code-block:: cpp

   if (!nav_state.has("robot_pose") || !nav_state.has("goals") || !nav_state.has("map")) return;

   const auto & pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
   const auto & goal = nav_state.get<nav_msgs::msg::Goals>("goals").goals.front().pose;
   const auto & map = nav_state.get<grid_map::GridMap>("map");

   auto path = compute_path(map, pose.pose.pose, goal);

   nav_state.set("path", path);

**Controller Example**

A controller reads the current pose and planned path, and outputs a velocity command:

.. code-block:: cpp

   if (!nav_state.has("path") || !nav_state.has("robot_pose")) return;

   const auto & path = nav_state.get<nav_msgs::msg::Path>("path");
   const auto & pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose").pose.pose;

   geometry_msgs::msg::TwistStamped cmd_vel = compute_control(path, pose);

   nav_state.set("cmd_vel", cmd_vel);

**Localization Example**

A localizer typically updates the estimated pose:

.. code-block:: cpp

   nav_state.set("robot_pose", latest_odom);

Advanced Features
-----------------

**Namespacing**

You can organize data hierarchically by using dots in keys:

- `"map.static"` vs `"map.dynamic"`
- `"perception.lidar"` vs `"perception.camera"`

**Debugging**

You can inspect the contents of the blackboard as a string:

.. code-block:: cpp

   std::cout << nav_state.debug_string();

**Custom Printers**

You can register pretty-printers for your own types using:

.. code-block:: cpp

   NavState::register_printer<MyType>(
     [](const MyType & value) {
       std::ostringstream out;
       out << "MyType: " << value.to_string();
       return out.str();
     });

This improves the output of `debug_string()` for non-standard types.

Best Practices
--------------

- Always check `has(key)` before `get<T>(key)` to avoid exceptions.
- Use descriptive keys, preferably documented.
- When designing a plugin, clearly define which `NavState` keys it consumes and which ones it produces.
- Avoid storing large or non-copyable data unless necessary.

Conclusion
----------

The `NavState` blackboard is a powerful and lightweight alternative to traditional ROS 2 message passing. It simplifies integration between modules and improves runtime performance by avoiding queues and callback latencies.

It is the recommended mechanism for **all internal data exchange** in EasyNav.


