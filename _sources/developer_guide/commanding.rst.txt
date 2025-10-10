.. _commanding:


Sending Navigation Commands to EasyNav
**************************************

EasyNav is designed to be easy to control from user applications or higher-level decision-making modules.
Instead of relying on ROS 2 Actions, which can be complex and hard to integrate across systems, EasyNav follows a simpler and more transparent communication model using topics and message protocols.

Primary Method: `/easynav_control` Topic
========================================

The main interface to control navigation is the topic `/easynav_control`, which uses the message type `easynav_interfaces/msg/NavigationControl`.
This message encapsulates both commands and feedback for the navigation process.

Here is the full definition of the message:

.. code-block:: cpp

  uint8 REQUEST=0
  uint8 REJECT=1
  uint8 ACCEPT=2
  uint8 FEEDBACK=3
  uint8 FINISHED=4
  uint8 FAILED=5
  uint8 CANCEL=6
  uint8 CANCELLED=7
  uint8 ERROR=8

  uint8 type
  std_msgs/Header header
  int64 seq
  string user_id
  string nav_current_user_id
  nav_msgs/Goals goals
  string status_message
  geometry_msgs/PoseStamped current_pose
  builtin_interfaces/Duration navigation_time
  builtin_interfaces/Duration estimated_time_remaining
  float32 distance_covered
  float32 distance_to_goal

Protocol Description
--------------------

The communication protocol over `/easynav_control` is designed around a type-based message system. Each message indicates a specific type of interaction or feedback:

- **REQUEST (0)**: A new navigation goal is requested. The `goals` field must be filled.
- **REJECT (1)**: The system rejects the navigation goal.
- **ACCEPT (2)**: The system has accepted the navigation goal and has started processing.
- **FEEDBACK (3)**: Periodic updates including the current pose, distance covered, and estimated time remaining.
- **FINISHED (4)**: Navigation completed successfully.
- **FAILED (5)**: Navigation failed (e.g., due to unreachable goal or obstacle).
- **CANCEL (6)**: The user requests to cancel the current navigation.
- **CANCELLED (7)**: Navigation has been cancelled.
- **ERROR (8)**: An error occurred during navigation.

Each message has a sequence number (`seq`), user ID (`user_id`), and optionally contains feedback fields if it is of a type that requires them.

To issue a navigation goal, a client should publish a message like:

.. code-block:: cpp

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header = ...
  goal_pose.pose.position = ....
  goal_pose.pose.orientation = ....
  
  auto command = std::make_unique<easynav_interfaces::msg::NavigationControl>();
  command->header = goal_pose.header;
  command->seq = 32;
  command->user_id = "nav_user_1";
  command->type = easynav_interfaces::msg::NavigationControl::REQUEST;
  command->goals.header = goal_pose.header;
  command->goals.goals.push_back(goal_pose);

The system will then respond with messages of type ACCEPT, FEEDBACK, FINISHED, etc., according to the internal state.

Using `GoalManagerClient`
-------------------------

Instead of manually implementing the communication protocol, developers are encouraged to use the `GoalManagerClient` class. It abstracts the handling of the `/easynav_control` topic and maintains the internal state machine automatically.

- `send_goal(PoseStamped goal)` issues a goal with proper message formatting.
- `cancel()` sends a CANCEL message.
- `get_state()` returns the current state (e.g., `State::NAVIGATING`, `State::NAVIGATION_FINISHED`, etc.).
- After receiving a terminal state (`FINISHED`, `FAILED`, `CANCELLED`, `REJECTED`, or `ERROR`), you **must** call `reset()` to clear the state before issuing a new command.

Secondary Method: `/goal_pose` Topic
====================================

For simplicity, EasyNav also listens to the topic `/goal_pose` of type `geometry_msgs/msg/PoseStamped`.
Publishing a pose here is equivalent to publishing a `REQUEST` command on `/easynav_control`.

This interface is ideal for manual tools (such as RViz2 "2D Goal Pose") or simple applications where feedback tracking is not required.
However, users may still subscribe to `/easynav_control` to monitor navigation status and transitions.

Conclusion
==========

- For complete command and feedback control, use the `/easynav_control` topic with the `GoalManagerClient`.
- For basic use cases, publishing to `/goal_pose` is sufficient.
- After receiving any terminal state (FINISHED, FAILED, etc.), a RESET command is mandatory before a new REQUEST.
