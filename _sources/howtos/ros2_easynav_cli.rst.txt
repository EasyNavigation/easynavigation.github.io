.. _ros2_easynav_cli:

=====================================
ros2 easynav — EasyNav CLI Extensions
=====================================

NAME
====

**ros2 easynav** — top-level command group providing EasyNav developer/operator utilities.

This CLI is part of ``easynav_tools``, included in the core ``easynav`` package — no
extra plugins needed. See :doc:`../build_install/index` if you have not installed
EasyNav yet (APT, Pixi or source all work).


SYNOPSIS
========

.. code-block:: bash

   ros2 easynav <command> [options]

Available commands:

- **plugins**              List pluginlib plugins known to EasyNav.
- **nav_state**            Print live NavState updates for a duration.
- **goal_info**            Print live Goal Manager info for a duration.
- **navigation_control**   Print live Navigation Control info for a duration.
- **twist**                Print `geometry_msgs/Twist` and `TwistStamped` for a duration.
- **timestats**            Print periodic time-stats (TUI-like refresh) for a duration.

Note that these verb names use underscores (e.g. ``nav_state``, not ``nav-state``), since
each is registered as a ``ros2cli`` verb entry point spelled exactly that way in
``easynav_tools/setup.py``.


DESCRIPTION
===========

The **ros2 easynav** command group bundles several diagnostic and discovery utilities for the
EasyNav framework. Most “live” commands subscribe to EasyNav topics and refresh their output
for a configurable time window (default: 5000 seconds).

Run ``-h`` on the group or any subcommand for inline help:

.. code-block:: bash

   ros2 easynav -h
   ros2 easynav plugins -h
   ros2 easynav nav_state -h
   # ... etc.


COMMANDS
========

plugins
-------

List EasyNav `pluginlib` plugins grouped by category (maps managers, planners, controllers, localizers, and filters).

**Usage**

.. code-block:: bash

   ros2 easynav plugins [--mapsmanager] [--localizer] [--planner] [--controller]
                        [--costmap-filters] [--navmap-filters] [--sensors]
                        [--grep SUBSTR] [--show-lib] [--show-xml]
                        [--json] [--pretty] [--debug]

**Options**

- ``--mapsmanager``        Only show mapsmanager plugins.
- ``--localizer``          Only show localizer plugins.
- ``--planner``            Only show planner plugins.
- ``--controller``         Only show controller plugins.
- ``--costmap-filters``    Only show Costmap2D filter plugins.
- ``--navmap-filters``     Only show NavMap filter plugins.
- ``--sensors``            Only show sensor perception handler plugins.
- ``--grep SUBSTR``        Filter by substring in plugin *name* or *type*.
- ``--show-lib``           Print the shared library path for each plugin (if available).
- ``--show-xml``           Print the plugin XML descriptor path.
- ``--json``               Output machine-readable JSON instead of text.
- ``--pretty``             Pretty-print JSON (only applies with ``--json``).
- ``--debug``              Print ament-index scanning details for troubleshooting.

**Notes**

This command scans **all ament index roots** across your overlays and system install.
Output can be filtered by category and/or by substring match.

**Examples**

.. code-block:: bash

   # List everything, human-readable:
   ros2 easynav plugins

   # Only planners, showing their library and XML descriptor:
   ros2 easynav plugins --planner --show-lib --show-xml

   # JSON output filtered by "serest":
   ros2 easynav plugins --grep serest --json --pretty


nav_state
---------

Print live **NavState** updates (robot pose, velocities, goal status, etc.) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav nav_state [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav nav_state --duration 30


goal_info
---------

Print live **Goal Manager** information (pending goals, active goal, completion/failure events) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav goal_info [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav goal_info --duration 20


navigation_control
-------------------

Print live **Navigation Control** status (control loop metrics, setpoints, internal flags) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav navigation_control [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav navigation_control --duration 60


twist
-----

Print live **Twist** and **TwistStamped** messages for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav twist [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav twist --duration 10


timestats
---------

Print live **time-stats** in a TUI-like loop for the given duration.
(Uses screen refresh to render a table; the verb is registered as **``timestats``**
in ``easynav_tools/setup.py``, even though its implementation lives in
``easynav_tools/cli/timetats.py``.)

**Usage**

.. code-block:: bash

   ros2 easynav timestats [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav timestats --duration 15


OPTIONS (Common)
================

The live-print commands (``nav_state``, ``goal_info``, ``navigation_control``, ``twist``,
``timestats``) accept:

- ``--duration SECONDS`` — time window to keep printing; default is long (``5000.0``) for continuous sessions.
- ``--spin-time SPIN_TIME`` — discovery spin time in seconds (only applies when not using an already running daemon).
- ``-s, --use-sim-time`` — enable ROS simulation time.
- ``--no-daemon`` — do not spawn nor use an already running ``ros2cli`` daemon.

``plugins`` does not accept these node-discovery options; it only scans the ament index and
takes the category/filter/output flags documented above.


EXIT STATUS
===========

Returns **0** on success. Non-zero on failures (e.g., transport errors, ROS graph unavailable).


EXAMPLES
========

.. code-block:: bash

   # Quick inventory of installed EasyNav plugins
   ros2 easynav plugins

   # Inspect planner plugins and search for "astar"
   ros2 easynav plugins --planner --grep astar

   # Monitor NavState for 45 seconds
   ros2 easynav nav_state --duration 45

   # Watch goal manager events while sending goals from RViz
   ros2 easynav goal_info --duration 60

   # Observe control loop values
   ros2 easynav navigation_control --duration 30

   # Tail robot velocities
   ros2 easynav twist --duration 10

   # Render periodic time-stats with screen refresh
   ros2 easynav timestats --duration 20


SEE ALSO
========

- :doc:`../howtos/index`
- :doc:`../developer_guide/index`
- `ROS 2 CLI <https://docs.ros.org>`_ (ros2cli)
