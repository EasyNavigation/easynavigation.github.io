.. _safety:

Safety
******

**EasyNav is not a certified safety component.** It runs on Linux and ROS 2, for which no
certification evidence exists. It is designed to be the **non-safety** part of a robot whose safety
functions (protective stop, safely limited speed, emergency stop) are implemented by an independent,
certified **safety channel**: a safety laser scanner, a safety PLC and STO (safe torque off) on the
drives, typically PL d / SIL 2 under ISO 3691-4 or ISO 13482. EasyNav makes integrating with that
channel easier, keeps it from being triggered needlessly, and never stands in its way.

.. _safety_summary:

Summary
=======

Status: **Provided** — available today; **Partial** — part of it; **Planned** — on the roadmap;
**Integrator** — the integrator's responsibility, outside EasyNav.

.. list-table::
   :header-rows: 1
   :widths: 22 24 40 14

   * - Topic
     - Reference
     - What EasyNav does
     - Status
   * - Safety functions (protective stop, safely limited speed, emergency stop, STO)
     - ISO 3691-4, ISO 13482; ISO 13849-1 PL d / IEC 61508 SIL 2
     - None: they must be implemented by the certified safety channel. EasyNav's own collision
       braking (``CollisionSafetyReflex``) reduces how often that channel has to act, but it is not
       a safety function.
     - Integrator
   * - Certification of EasyNav (SIL / PL)
     - IEC 61508-3, ISO 13849-1
     - Not certified, and not designed to be: EasyNav is the non-safety element next to the safety
       channel.
     - Integrator
   * - Freedom from interference: EasyNav cannot prevent the safety channel from acting
     - IEC 61508-3 annex F (non-interference)
     - By architecture: EasyNav only requests velocities, and has no path to the STO chain. The
       safety channel must supervise the measured speed, not trust EasyNav's commands.
     - Integrator
   * - No stale velocity commands
     - IEC 61508-3 (defensive programming)
     - Only new commands are sent; with none for ``cmd_timeout`` (0.5 s by default) the robot brakes
       to zero; non-finite commands are discarded. See :ref:`safety_commands`.
     - Provided
   * - Detecting that EasyNav stopped commanding (dead process, blocked cycle)
     - IEC 61784-3 (timeout of the safety communication)
     - Periodic republication of the command and ``deadline``/``liveliness`` QoS on ``cmd_vel``, so
       the receiver can detect the loss. An explicit heartbeat with sequence counter and
       configuration hash is planned. See :ref:`safety_commands`.
     - Partial
   * - Velocity and acceleration limits
     - ISO 3691-4 (speed control)
     - ``robot_limits`` are enforced on every command, in any mode; they are not safety-rated.
       See :ref:`velocity_output`.
     - Provided
   * - Consistency with the safety channel's limits
     - ISO 3691-4 (speed and protective fields)
     - Configuring fails if ``robot_limits`` exceed the declared ``safety.plc_limits``. Reading the
       channel's live state (active field, current speed limit, protective stop) is planned.
       See :ref:`safety_mode`.
     - Partial
   * - Configuration data: validated, identified, unchanged while running
     - IEC 61508-3 (configuration data)
     - Invalid values fail to configure; a SHA-256 fingerprint of every parameter is logged, and the
       parameters saved and published; in safety mode the configuration is frozen. See
       :ref:`safety_checks`, :ref:`safety_fingerprint`.
     - Provided
   * - Timing: real-time scheduling, no page faults in the RT cycle
     - IEC 61508-3 (timing behavior)
     - ``SCHED_FIFO`` priority 80, required in safety mode; optional memory locking. Monitoring of
       RT-cycle overruns is planned. See :ref:`safety_mode`, :ref:`safety_memory`.
     - Partial
   * - Faulty software components
     - IEC 61508-3 (fault detection, fail-safe behavior)
     - Exceptions in any plugin are contained; a failing controller or safety reflex stops the robot.
     - Provided
   * - Fault injection to verify the behavior on failures
     - IEC 61508-3 (fault insertion testing)
     - ``FaultyController`` (throw, hang, stop proposing, freeze, huge or NaN commands), used by the
       tests. Fault plugins for the other components are planned. See :ref:`safety_faults`.
     - Partial
   * - Stale sensor data and transforms
     - IEC 61508-3 (defensive programming)
     - A maximum age for perceptions and TF is planned.
     - Planned
   * - Controlled stop on shutdown and on unrecoverable errors
     - IEC 60204-1 (stop categories)
     - Deactivating or shutting down brakes within the deceleration limits and ends with an exact
       zero; an unrecoverable error ends EasyNav through the lifecycle's error path.
     - Provided
   * - Diagnostics and recovery
     - ISO 3691-4 (fault handling)
     - Standard diagnostics in NavState and on ``diagnostics``; a diagnostic-driven recovery system
       (see :ref:`recovery`).
     - Provided
   * - Development process: tests, CI, coverage
     - IEC 61508-3 (verification techniques, annexes A and B)
     - Unit and integration tests in CI on every supported ROS 2 distribution, coverage published.
       Not yet: a coding standard (MISRA C++), mandatory static analysis, 100 % statement coverage,
       requirements traceability.
     - Partial
   * - Information for integrators
     - IEC 61508-3 annex D (safety manual)
     - This page: assumptions of use, safety-related parameters and interfaces.
     - Partial

Overview
========

What EasyNav provides, in short:

- the velocity command sent to the robot is never stale or invalid;
- the configuration is checked, and fingerprinted, every time EasyNav is configured;
- an optional **safety mode** makes EasyNav stricter, and the process memory can be locked.

The robot limits (``controller_node.robot_limits.*``, see :ref:`velocity_output`) **always apply**,
whatever the safety parameters below say.

The code lives in ``safety`` modules (namespace ``easynav::safety``), so the nodes only delegate to
it: ``safety::CommandGuard`` in ``easynav_controller`` and ``safety::SafetySupervisor`` in
``easynav_system``.


.. _safety_commands:

Stale and invalid commands
==========================

Drivers keep executing the last command they received, so ``ControllerNode`` makes sure it is never
a stale or invalid one:

.. code-block:: yaml

   controller_node:
     ros__parameters:
       cmd_timeout: 0.5               # s, 0 disables it
       cmd_vel_keepalive_period: 0.0  # s, 0 disables it

- **Only new controller commands** are proposed: a command whose stamp and value are the same as the
  previous one is not proposed again. A controller plugin must therefore stamp each new command
  (``header.stamp``).
- **Non-finite commands** (NaN or inf in any axis, from any source) are discarded; the mux uses the
  next valid source.
- ``cmd_timeout``: if no source proposes a new command for this long, the robot brakes to zero
  within the deceleration limits. This covers a controller that stops writing ``cmd_vel``, keeps
  writing the same one, or only produces non-finite values. It must be longer than the controller's
  period (``<controller>.rt_freq``), or configuring fails. If the clock jumps back (e.g. a simulation
  restarts), the timeout counts from the jump.
- ``cmd_vel_keepalive_period``: the current command is republished at least this often, even if it
  does not change. The publisher then also offers ``deadline`` and ``liveliness`` QoS of twice this
  period, so a driver or a safety controller that requests a deadline is notified when EasyNav stops
  commanding (e.g. a blocked RT cycle or a dead process). It is off by default because publishing
  zeros at rest would block a lower-priority teleoperation in a ``twist_mux``.
- The velocity publisher keeps only the latest command (depth 1).

A timed-out or discarded command is reported in NavState as ``diagnostics.cmd_vel``
(``diagnostic_msgs/DiagnosticStatus``, ``hardware_id: controller_node``), in ``ERROR`` while the
problem lasts and back to ``OK`` when commands flow again. It is written only after the first
problem, so a recovery system can handle it (see :ref:`recovery`).


.. _safety_checks:

Configuration checks
====================

In any mode, configuring fails, naming the parameter, when:

- a robot limit is out of range or not finite: velocities ``max_* >= 0``, ``min_linear_vel <= 0``,
  accelerations and decelerations ``> 0``;
- a frequency (``system_node.rt_freq``, ``freq``) is not ``> 0``, a spin time or a robot geometry
  field is negative, or any of them is not finite;
- ``cmd_timeout`` or ``cmd_vel_keepalive_period`` is negative or not finite;
- the robot limits exceed ``safety.plc_limits``, when given (see below).

A failed configure leaves every EasyNav node unconfigured.

Every configure also fingerprints the configuration (see :ref:`safety_fingerprint`).


.. _safety_mode:

Safety parameters and safety mode
=================================

The safety parameters are grouped in ``system_node``:

.. code-block:: yaml

   system_node:
     ros__parameters:
       safety:
         mode: false            # stricter, see below
         lock_memory: false     # lock the process memory, in any mode
         plc_limits:            # limits configured in the safety channel (0: not given)
           max_linear_vel: 1.0  # m/s
           max_angular_vel: 2.0 # rad/s

``safety.plc_limits``
   Not applied to the commands (``robot_limits`` are): they declare the limits the safety channel
   enforces, e.g. the safety PLC's safely limited speed. ``robot_limits.max_linear_vel``,
   ``|min_linear_vel|`` and ``max_angular_vel`` may not exceed them, so EasyNav is never configured
   to command more than the channel allows, which would only end in safety stops. Checked whenever
   given; required in safety mode.

``safety.lock_memory``
   ``system_main`` locks the process memory before activating (see :ref:`safety_memory`). If it
   cannot, EasyNav does not start.

``safety.mode``
   Makes EasyNav stricter. Whatever is unsafe makes configuring fail, instead of a warning or a
   default:

   - ``safety.plc_limits`` are required;
   - ``controller_node.cmd_vel_keepalive_period`` and ``cmd_timeout`` must be ``> 0``;
   - real-time scheduling is required: ``use_real_time`` must be ``true``, and ``system_main``
     terminates EasyNav if it cannot get ``SCHED_FIFO`` (see :ref:`realtime_setup`);
   - once configured, **the configuration is frozen**: any change to a parameter of any EasyNav node
     is rejected (so plugins cannot be switched either), and so are the recovery system's
     ``request_reconfigure()`` and ``request_restore_parameters()``, which return ``false``.
     Setting a parameter to the value it already has, or declaring a new one, is still accepted, so
     EasyNav can still go through its lifecycle.

``safety.mode`` and ``safety.lock_memory`` are independent: whether the memory can be locked depends
on the hardware and the deployment, not on wanting the rest of the safety mode.

.. list-table::
   :header-rows: 1
   :widths: 15 20 65

   * - ``mode``
     - ``lock_memory``
     - Result
   * - ``false``
     - ``false``
     - Default behavior.
   * - ``false``
     - ``true``
     - Memory locked (or EasyNav does not start); nothing else required.
   * - ``true``
     - ``false``
     - Full safety mode, without locking the memory.
   * - ``true``
     - ``true``
     - Full safety mode and memory locked (or EasyNav does not start).


.. _safety_memory:

Memory locking
==============

**The problem.** Linux gives each process virtual memory: which of its pages (4 KB each) are really in
RAM at a given moment is up to the kernel. A page may not be there because it was never touched yet
(memory is only given when first written), because it was moved to swap, or because it holds code of
a library (e.g. a plugin's ``.so``), which the kernel can drop even without swap since it can read it
again from disk. Touching such a page is a *page fault*: the process stops while the kernel brings the
page in, which takes microseconds, or milliseconds if it has to read the disk.

``SCHED_FIFO`` keeps other processes from taking the CPU away from the RT cycle, but a page fault is
the RT cycle itself waiting for the disk: its priority does not help. For example, after the robot
has been idle for a while, the kernel may have dropped the pages of the controller; the first cycle
when it moves again then takes tens of milliseconds instead of one. It is rare, but a bounded response
time has to hold always, not almost always.

**The solution.** With ``safety.lock_memory``, ``system_main`` calls
``mlockall(MCL_CURRENT | MCL_FUTURE)`` before activating: every page of the process, those it has
and those it gets later, stays in RAM and is never dropped. This is the usual practice in real-time
ROS 2 (e.g. ``ros2_control``).

**Why the limit must be unlimited.** The memory a process may lock is limited by ``RLIMIT_MEMLOCK``
(``ulimit -l``), often a few MB. With ``MCL_FUTURE`` and a finite limit, locking may work at first,
but once EasyNav needs more locked memory than allowed, **new allocations fail**: a ``new`` throws
anywhere, e.g. when a large map or point cloud arrives. EasyNav with ROS 2, DDS and its plugins uses
hundreds of MB, so that would happen almost at once, which is far worse than not locking at all.
That is why EasyNav checks the limit first and refuses to start instead. To allow it, see
:ref:`realtime_setup`, for user sessions, systemd services and Docker.

**The cost.** All of EasyNav's memory stays in RAM: it cannot go to swap and the system cannot reclaim
it when short of memory. On a robot with enough RAM it does not matter; on one that is short of it,
other processes may fail instead of slowing down. That is why it is optional, and independent of
``safety.mode``: whether it can be done depends on the hardware and the deployment.

**How much it helps.** On a robot without swap and with plenty of RAM the gain is moderate, but code
pages can still be dropped without swap. In a safety argument it matters to be able to state that
there are no page faults from evicted pages, rather than to measure it and trust it does not happen.


.. _safety_fingerprint:

Configuration fingerprint
=========================

Every time EasyNav configures successfully, it computes a fingerprint of its configuration: a
SHA-256, 64 hex characters. The same fingerprint means the same configuration; changing a single
parameter gives a different one.

**How it is computed.**

1. Every parameter of every EasyNav node (``system_node`` and its subnodes) is written, one per line,
   sorted by node and name::

      controller_node/cmd_timeout=0.500000
      controller_node/controller_types=[ctrl]
      controller_node/ctrl.plugin=easynav_simple_controller/SimpleController
      ...
      system_node/safety.mode=false

2. The SHA-256 of that text is computed.
3. It is logged, with the plugins loaded and where the parameters were saved, and left in NavState
   as ``configuration_hash``::

      [INFO] [system_node]: Configuration SHA-256: 1c161c031f71...65a
        (/home/user/.ros/log/easynav_configuration_1c161c031f71...65a.txt). Plugins:
        controller_node: ctrl [easynav_simple_controller/SimpleController]

**Where the parameters are.** The fingerprint tells whether two configurations differ; the parameters
behind it tell how:

- They are saved in the ROS log directory (``$ROS_LOG_DIR``, else ``$ROS_HOME/log``, else
  ``~/.ros/log``), in ``easynav_configuration_<hash>.txt``, or
  ``easynav_configuration_<namespace>_<hash>.txt`` for an EasyNav in a namespace (e.g.
  ``easynav_configuration_robot1_<hash>.txt`` for ``/robot1``). A configuration seen before is not
  saved again: same hash, same contents. Saving never stops EasyNav; if it fails, a warning says why.
- They are also published on the ``easynav_configuration`` topic (``/<namespace>/easynav_configuration``
  in a namespace; ``std_msgs/String``, latched: a subscriber that comes later still gets the last
  one), with a first line ``# SHA-256: <hash>``:

  .. code-block:: bash

     ros2 topic echo --once --qos-durability transient_local /easynav_configuration

  This is handy for a quick look at a remote robot; ``ros2 topic echo`` prints it quoted, as YAML.

To see what differs between two robots, or between a robot and the validated configuration, compare
their saved files; they are sorted text, one parameter per line:

.. code-block:: bash

   diff easynav_configuration_<validated_hash>.txt easynav_configuration_<robot_hash>.txt

**Why this way.**

- The **effective parameters**, not the YAML file: launch arguments, several files and default values
  all end up in what runs. Comments, formatting and order in the YAML do not matter.
- **Default values are included**: if a new version of a plugin changes a default, the fingerprint
  changes even though your YAML did not, because the behavior did.
- **Sorted**: plugins declare their parameters in their own order; sorting gives the same text, and
  fingerprint, for the same configuration.
- **SHA-256**, not ``std::hash``: it is a standard, the same on any machine and compiler, can be
  checked with external tools (``sha256sum``), and two different configurations will in practice
  never share it.

**What it is for.**

- Knowing which configuration a robot runs: compare one log line between two robots, or with a robot
  someone has "tuned" in the field.
- Tying the validated configuration to the deployed one: record the fingerprint of the configuration
  that passed the tests, and check the robot runs the same. For functional safety, parameters are
  configuration data, and it must be shown that what runs is what was validated.
- In safety mode the configuration is frozen once configured, so the fingerprint stays valid while
  EasyNav runs. Outside it, a parameter changed at run time is only reflected at the next configure.

**What it does not cover.** It fingerprints **parameters**, not the whole system:

- **Software**: two robots with the same fingerprint may run different versions of EasyNav or of the
  plugins, if those versions do not change any parameter.
- **Contents of external files**: a map or a calibration file only counts through its path; changing
  the map but not its name leaves the fingerprint unchanged.


.. _safety_faults:

Fault injection
===============

To test how EasyNav copes with components that misbehave, fault-injection plugins live next to the
``Dummy*`` plugins of each component. ``easynav_controller/FaultyController`` commands a constant
velocity (``<name>.linear_vel``, ``<name>.angular_vel``) and, after ``<name>.fault_after`` updates,
injects the fault in ``<name>.fault``: ``throw``, ``hang`` (for ``<name>.hang_time`` s),
``stop_proposing``, ``freeze``, ``max_velocity`` or ``nan``.
