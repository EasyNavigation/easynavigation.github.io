.. _recovery:

Recovery System
***************

Navigation fails in many ways: localization diverges, a sensor dies, the robot gets stuck, an
obstacle appears right ahead. The **recovery system** is the part of EasyNav that detects these
problems and reacts to them: it can drive the robot, stop it, hold or abort the mission, change
parameters, or terminate EasyNav.

EasyNav does not impose a recovery strategy. The whole recovery system is **one plugin**, and EasyNav
only hosts it: it gives the plugin its cycles and a narrow set of means to act. Replacing the
recovery system is a configuration change.

.. contents:: On this page
   :local:
   :depth: 2


The Recovery Node
=================

``recovery_node`` (``RecoveryManagerNode``, package ``easynav_recovery``) is one more EasyNav
lifecycle node, owned by ``SystemNode`` like the sensors, maps manager, localizer, planner and
controller nodes. It loads the ``RecoveryManagerBase`` plugin named by ``recovery_manager.plugin``
on every configure, and releases it on cleanup, shutdown or error.

.. code-block:: yaml

   recovery_node:
     ros__parameters:
       use_sim_time: true
       recovery_manager:
         plugin: easynav_simple_recovery/SimpleRecoveryManager
         # ...the recovery system's own parameters, under recovery_manager.*

Without ``recovery_manager.plugin``, ``easynav_recovery/DummyRecoveryManager`` is loaded: it does
nothing, so EasyNav behaves as if there were no recovery system.

``SystemNode`` cycles the recovery system in both loops:

- **RT cycle**: right after the controller has proposed its command, and before the velocity command
  is published. This is where fast reactions live (e.g. braking before an obstacle).
- **Non-RT cycle**: after the rest of EasyNav (sensors, maps, localization, goals and planning) has
  run. This is where problems are diagnosed and decided upon.


The Base Class: ``RecoveryManagerBase``
=======================================

A recovery system derives from ``easynav::RecoveryManagerBase`` (``easynav_core``), a
``MethodBase`` like any other EasyNav plugin:

.. code-block:: cpp

   class RecoveryManagerBase : public MethodBase
   {
   public:
     virtual void on_activate() {}    // EasyNav activated
     virtual void on_deactivate() {}  // EasyNav deactivated

   protected:
     // Every non-RT cycle: diagnose, decide, act. Rate-limit yourself if needed.
     virtual void update(NavState & nav_state) = 0;

     // Every RT cycle, before publishing. True if it commanded the robot.
     virtual bool update_rt(NavState & nav_state) {return false;}

     // Moving the robot
     void command_velocity(NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd);
     void override_velocity(NavState & nav_state, const geometry_msgs::msg::TwistStamped & cmd);

     // Acting on the mission and on EasyNav (see SystemActions)
     void abort_mission(const std::string & reason);
     void hold_mission_progress(bool hold);
     void request_shutdown(const std::string & reason);
     void request_reconfigure(const std::vector<ParameterChange> & changes, const std::string & reason);
     void request_restore_parameters(const std::string & reason);
   };

A faulty recovery system cannot bring EasyNav down: ``update()`` and ``update_rt()`` run inside
exception-safe wrappers. If ``update_rt()`` throws, the robot is stopped (an override of zero
velocity) as a fail-safe.


Moving the Robot
================

The recovery system never overwrites the controller's command. Every source of velocity proposes its
command in its own slot in NavState, and ``ControllerNode``, the single velocity output of EasyNav,
selects one of them every RT cycle (``VelocityMux``), in this order:

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Source
     - Proposed with
     - Meaning
   * - ``OVERRIDE``
     - ``override_velocity()``
     - Emergency (e.g. braking before a collision). Highest priority, published **as is**, without
       smoothing.
   * - ``TAKEOVER``
     - ``command_velocity()``
     - The recovery system drives the robot (e.g. backing up). Preferred over the controller, and
       smoothed within the robot limits.
   * - pause
     - (``easynav_control``)
     - A paused navigation commands zero velocity.
   * - ``CONTROLLER``
     - (the controller plugin)
     - Nominal motion.

A proposal lasts **one RT cycle**: the mux consumes every proposal, so a recovery that drives the
robot must command it on every RT cycle while it lasts. As soon as it stops proposing, control goes
back to the controller, which has kept computing its command undisturbed.

The selected command goes through the ``VelocitySmoother``, which enforces the robot's velocity and
acceleration limits (``controller_node.robot_limits.*``) and stops at zero before a change of
direction. An override bypasses it, and the smoother continues from the overridden command.


System Actions
==============

Beyond velocity, a recovery system acts on the mission and on EasyNav through ``SystemActions``
(``easynav_core``), an interface implemented by ``SystemNode``. ``RecoveryManagerBase`` exposes each
action as a protected method with the same name, which forwards it to ``SystemNode`` (and ignores it,
with a warning, when there is no system, e.g. in a unit test).

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Action
     - Effect
   * - ``abort_mission(reason)``
     - Fails the active mission: the client receives ``ERROR`` on ``/easynav_control``, with
       ``reason`` as its ``status_message``. Does nothing without a mission.
   * - ``hold_mission_progress(hold)``
     - While held, the mission stays active and its feedback keeps flowing, but **no goal is taken
       as reached**: the robot pose cannot be trusted (e.g. while relocalizing). The hold lasts
       until released, across missions; unloading the recovery system releases it.
   * - ``request_shutdown(reason)``
     - Terminates EasyNav because of an unrecoverable problem (see below). Latched: the first
       reason is kept.
   * - ``request_reconfigure(changes, reason)``
     - Changes parameters of any EasyNav node and reconfigures EasyNav to apply them (see below).
   * - ``request_restore_parameters(reason)``
     - Restores every parameter changed by ``request_reconfigure()`` to its value before the first
       change, and reconfigures.

Controlled shutdown
-------------------

After ``request_shutdown()``, EasyNav stops the robot and leaves ``Active`` through the lifecycle's
error path: deactivation returns ``ERROR``, ``on_error()`` shuts down every EasyNav node, and they
all end in ``Finalized``. ``system_main`` acts as the supervisor: it leaves its loops, prints the
reason and exits with code 1. The launch files of ``easynav_indoor_testcase`` end the whole launch
(RViz included) when ``system_main`` exits.

Reconfiguration as a mitigation
-------------------------------

A recovery can change parameters and have EasyNav apply them, for example to slow the robot down
(``controller_node.robot_limits.max_linear_vel``) or to switch a plugin:

.. code-block:: cpp

   request_reconfigure(
     {{"controller_node", rclcpp::Parameter("robot_limits.max_linear_vel", 0.1)}},
     "stuck: slowing down");

The request is **not applied during the call**: the recovery system is itself reloaded. ``SystemNode``
keeps the request (a newer one replaces it), and the supervisor applies it between cycles: EasyNav
goes active → inactive → unconfigured, sets the parameters, and goes back to active. The mission goes
on, localizers continue from the last known pose, and the robot only stops during the transitions.

- An unknown node or parameter rejects the request. If a value is not accepted or configure fails,
  the previous values are restored; if that fails too, EasyNav shuts down.
- The recovery system is a **new instance** after a reconfiguration: anything it must remember goes
  in NavState. ``SystemNode`` lists the parameters changed so far in NavState's
  ``reconfigured_parameters`` (``"node/parameter"``), so the new instance knows its state.


Available Recovery Systems
==========================

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Plugin
     - Description
   * - ``easynav_recovery/DummyRecoveryManager``
     - The default: does nothing. EasyNav runs without recovery.
   * - ``easynav_simple_recovery/SimpleRecoveryManager``
     - A small recovery system written as a tutorial. See
       `easynav_simple_recovery README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_simple_recovery/README.md>`_.
   * - ``easynav_diagnostic_recovery/DiagnosticRecoveryManager``
     - A diagnosis-driven recovery system, made of plugins. See
       `easynav_diagnostic_recovery README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_diagnostic_recovery/README.md>`_.

DummyRecoveryManager
--------------------

Loaded when ``recovery_manager.plugin`` is not set. It never diagnoses, commands or asks anything. It
is also the minimal example of a ``RecoveryManagerBase`` implementation.

SimpleRecoveryManager
---------------------

A readable starting point for writing your own recovery system: one simple case for each output the
framework offers.

- **RT, fast recovery**: an obstacle right ahead while moving forward → brake dead
  (``override_velocity()``). The area checked ahead is the robot's radius and height
  (``system_node.robot_geometry``).
- **RT, mitigation**: executes the mitigation chosen by ``update()`` with ``command_velocity()``, on
  every RT cycle.
- **Non-RT**, one case at a time, the most severe first:

  .. list-table::
     :header-rows: 1
     :widths: 40 60

     * - Case
       - Action
     * - No new sensor data for ``sensors_timeout``
       - ``request_shutdown()``
     * - Localization lost (position variance too high)
       - ``hold_mission_progress(true)`` and rotate in place; ``abort_mission()`` if it takes too
         long
     * - Stuck (commanded but not moving)
       - Back up; after ``max_backup_attempts``, slow down with ``request_reconfigure()``; still
         stuck, ``abort_mission()``. The speed is restored when the mission ends.

See the `easynav_simple_recovery README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_simple_recovery/README.md>`_
for its parameters.

DiagnosticRecoveryManager
-------------------------

A complete recovery system driven by diagnostics, itself made of plugins at two levels:

- **Level 0, RT: safety reflexes** (``SafetyReflexBase``) check the command about to be sent, whoever
  produced it, and override it on imminent danger. ``CollisionSafetyReflex`` brakes when the commanded
  motion would hit an obstacle within its stopping distance.
- **Level 1, non-RT: evaluators and mitigations.** Evaluators (``RecoveryEvaluatorBase``) write
  standard ``diagnostic_msgs/DiagnosticStatus`` to NavState. Mitigations (``RecoveryMitigationBase``)
  handle diagnostics in ``ERROR``: at most one is active at a time, chosen by priority, and a
  mitigation that fails is excluded for that diagnostic, so the next one is tried (escalation). While
  a mitigation is active or a diagnostic is in ``ERROR``, the mission's progress is held.

Besides its own evaluators, it sees the diagnostics other components write to NavState's
``diagnostics`` group, e.g. ``ControllerNode``'s ``diagnostics.cmd_vel`` when no new velocity command
arrives or one is discarded (``hardware_id: controller_node``, see :ref:`velocity_output`).

It ships evaluators (no path, obstacle too close, controller stuck, miswired ROS graph) and
mitigations (safe retreat, advance, shutdown, wait for a human, cancel the mission). A component can
ship recovery for its own failures: ``easynav_costmap_localizer`` provides
``AmclConvergenceEvaluator`` and ``AmclRelocalizeMitigation``. Its plugin interfaces live in the
``easynav_diagnostic_recovery`` namespace, so a plugin says which recovery system it belongs to.

It publishes its diagnostics on ``diagnostics`` and what the active mitigation does on
``mitigation``; the EasyNav TUI shows both in its *Diagnostics* and *Mitigation* panels.

See the `easynav_diagnostic_recovery README <https://github.com/EasyNavigation/easynav_plugins/blob/rolling/recoveries/easynav_diagnostic_recovery/README.md>`_
for its plugins and parameters, and the
`recovery system design document <https://github.com/EasyNavigation/EasyNavigation/blob/rolling/docs/recoveries_easynav.md>`_
for the full rationale.


Writing a Recovery System
=========================

Derive from ``easynav::RecoveryManagerBase`` and implement ``update()`` and, if it acts in real time,
``update_rt()``:

.. code-block:: cpp

   #include "easynav_core/RecoveryManagerBase.hpp"

   namespace my_recovery
   {

   class MyRecoveryManager : public easynav::RecoveryManagerBase
   {
   protected:
     void update(easynav::NavState & nav_state) override
     {
       if (something_is_wrong(nav_state)) {
         abort_mission("something is wrong");
       }
     }

     bool update_rt(easynav::NavState & nav_state) override
     {
       if (danger_ahead(nav_state)) {
         override_velocity(nav_state, geometry_msgs::msg::TwistStamped());  // Brake
         return true;
       }
       return false;
     }
   };

   }  // namespace my_recovery

   #include <pluginlib/class_list_macros.hpp>
   PLUGINLIB_EXPORT_CLASS(my_recovery::MyRecoveryManager, easynav::RecoveryManagerBase)

Register it against ``easynav_core``:

.. code-block:: xml

   <class name="my_recovery/MyRecoveryManager" type="my_recovery::MyRecoveryManager"
          base_class_type="easynav::RecoveryManagerBase">
     <description>My recovery system.</description>
   </class>

.. code-block:: cmake

   pluginlib_export_plugin_description_file(easynav_core my_recovery_plugins.xml)

Keep in mind:

- ``update()`` and ``update_rt()`` run in different threads: share state through atomics, or keep it
  in one of them.
- ``update_rt()`` must be fast and bounded.
- A velocity proposal lasts one RT cycle.
- After a reconfiguration, your recovery system is a new instance: keep what must survive in
  NavState.
