.. _ros2_easynav_cli:

=====================================
ros2 easynav — EasyNav CLI Extensions
=====================================

NAME
====

**ros2 easynav** — top-level command group providing EasyNav developer/operator utilities.


SYNOPSIS
========

.. code-block:: bash

   ros2 easynav <command> [options]

Available commands:

- **plugins**              List pluginlib plugins known to EasyNav.
- **nav-state**            Print live NavState updates for a duration.
- **goal-info**            Print live Goal Manager info for a duration.
- **navigation-control**   Print live Navigation Control info for a duration.
- **twist**                Print `geometry_msgs/Twist` and `TwistStamped` for a duration.
- **timetats**             Print periodic time-stats (TUI-like refresh) for a duration.  (*command name as shipped*)


DESCRIPTION
===========

The **ros2 easynav** command group bundles several diagnostic and discovery utilities for the
EasyNav framework. Most “live” commands subscribe to EasyNav topics and refresh their output
for a configurable time window (default: 5000 seconds).

Run ``-h`` on the group or any subcommand for inline help:

.. code-block:: bash

   ros2 easynav -h
   ros2 easynav plugins -h
   ros2 easynav nav-state -h
   # ... etc.


COMMANDS
========

plugins
-------

List EasyNav `pluginlib` plugins grouped by category (maps managers, planners, controllers, localizers, and filters).

**Usage**

.. code-block:: bash

   ros2 easynav plugins [--mapsmanager] [--localizer] [--planner] [--controller]
                        [--costmap-filters] [--navmap-filters]
                        [--grep SUBSTR] [--show-lib] [--show-xml]
                        [--json] [--pretty] [--debug]

**Options**

- ``--mapsmanager``        Only show mapsmanager plugins.
- ``--localizer``          Only show localizer plugins.
- ``--planner``            Only show planner plugins.
- ``--controller``         Only show controller plugins.
- ``--costmap-filters``    Only show Costmap2D filter plugins.
- ``--navmap-filters``     Only show NavMap filter plugins.
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


nav-state
---------

Print live **NavState** updates (robot pose, velocities, goal status, etc.) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav nav-state [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav nav-state --duration 30


goal-info
---------

Print live **Goal Manager** information (pending goals, active goal, completion/failure events) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav goal-info [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav goal-info --duration 20


navigation-control
------------------

Print live **Navigation Control** status (control loop metrics, setpoints, internal flags) for a given duration.

**Usage**

.. code-block:: bash

   ros2 easynav navigation-control [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav navigation-control --duration 60


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


timetats
--------

Print live **time-stats** in a TUI-like loop for the given duration.  
(Uses screen refresh to render a table; the command name in the package is **``timetats``**, as shipped.)

**Usage**

.. code-block:: bash

   ros2 easynav timetats [--duration SECONDS]

**Options**

- ``--duration SECONDS``   Seconds to run (default: ``5000.0``)

**Example**

.. code-block:: bash

   ros2 easynav timetats --duration 15


OPTIONS (Common)
================

Many live-print commands accept:

- ``--duration SECONDS`` — time window to keep printing; default is long (``5000.0``) for continuous sessions.


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
   ros2 easynav nav-state --duration 45

   # Watch goal manager events while sending goals from RViz
   ros2 easynav goal-info --duration 60

   # Observe control loop values
   ros2 easynav navigation-control --duration 30

   # Tail robot velocities
   ros2 easynav twist --duration 10

   # Render periodic time-stats with screen refresh
   ros2 easynav timetats --duration 20


SEE ALSO
========

- :doc:`../howtos/index`
- :doc:`../developer_guide/index`
- `ROS 2 CLI <https://docs.ros.org>`_ (ros2cli)
