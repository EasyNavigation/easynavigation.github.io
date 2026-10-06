.. _install_binaries:

=====================
Install from Binaries
=====================

EasyNav is also distributed as binary packages, with APT (Debian packages from the ROS 2
buildfarm) and Pixi (self-contained conda packages).

.. warning::
   The binary packages are still **0.4.x**. This documentation describes EasyNav 0.5.0, not
   released yet: its parameter files, the recovery system, the safety mode and the PlayGrounds do
   not work with 0.4.x. Until 0.5.0 is released, :ref:`build from source <build_from_source>`.

.. contents:: On this page
   :local:
   :depth: 1

.. _release_status:

Release status
--------------

This documentation follows the ``rolling`` branch of the EasyNav repositories, which will be
released as **0.5.0** for rolling, lyrical, kilted and jazzy. The binary packages (APT and Pixi) are still
**0.4.x**:

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Distro
     - APT
     - Pixi
     - Source branch
   * - rolling
     - —
     - 0.4.2
     - ``rolling`` (0.5.0 in development)
   * - lyrical
     - 0.4.2
     - 0.4.2
     - ``lyrical`` (0.4.x)
   * - kilted
     - 0.4.1
     - 0.4.1
     - ``kilted`` (0.4.x)
   * - jazzy
     - 0.4.0
     - 0.4.0
     - ``jazzy`` (0.4.x)

.. _install_apt:

Install from binaries (APT)
----------------------------

EasyNav is released as binary Debian packages through the ROS 2 buildfarm for
**jazzy** (0.4.0), **kilted** (0.4.1) and **lyrical** (0.4.2). **Rolling** has no APT packages:
use :ref:`build_from_source`.
It needs ROS 2 installed from its official APT repositories.

.. code-block:: bash

   sudo apt update
   sudo apt install ros-<distro>-easynav

Installing plugins (APT)
~~~~~~~~~~~~~~~~~~~~~~~~

``ros-<distro>-easynav`` only installs the **core** EasyNav framework (``easynav_system``,
``easynav_sensors``, ...). Controllers, localizers, planners and maps managers are
shipped as separate packages — see the full catalogue at :doc:`../plugins/index` — and
you need to install the ones your configuration actually uses. For example, for a costmap
configuration with the Regulated Pure Pursuit controller:

.. code-block:: bash

   sudo apt install \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

Not every plugin is packaged (see :ref:`package_availability`): in **jazzy**,
``easynav-costmap-localizer`` and ``easynav-simple-localizer`` are missing.

.. _install_pixi:

Install via Pixi
-----------------

EasyNav publishes prebuilt `Pixi <https://pixi.sh>`_/conda packages on
`prefix.dev <https://prefix.dev>`_, in the Intelligent Robotics Lab channels (``irl-<distro>``),
built on top of `RoboStack <https://robostack.github.io>`_. A Pixi environment is fully
self-contained: it ships its own ROS 2 distribution, so you do **not** need a system ROS 2
install.

.. note::
   The official RoboStack channels only have EasyNav for **jazzy** (0.4.0). The ``pixi.toml``
   files below use the IRL channels, which have it for all four distros.

For each ROS 2 distro, download the corresponding ``pixi.toml`` below and save it as
``~/easynav_ws/pixi.toml`` — the same workspace directory used throughout this guide
and in :doc:`../getting_started/index`. Then run:

.. code-block:: bash

   cd ~/easynav_ws
   pixi install
   pixi shell

``pixi shell`` opens a shell with ROS 2 and EasyNav ready to use (e.g. ``ros2 run
easynav_system system_main ...``). You can also prefix any command with ``pixi run`` instead of
entering the shell.

Rolling
~~~~~~~

:download:`pixi.toml <pixi_envs/rolling/pixi.toml>`

.. literalinclude:: pixi_envs/rolling/pixi.toml
   :language: toml

Lyrical
~~~~~~~

:download:`pixi.toml <pixi_envs/lyrical/pixi.toml>`

.. literalinclude:: pixi_envs/lyrical/pixi.toml
   :language: toml

Kilted
~~~~~~

:download:`pixi.toml <pixi_envs/kilted/pixi.toml>`

.. literalinclude:: pixi_envs/kilted/pixi.toml
   :language: toml

Jazzy
~~~~~

:download:`pixi.toml <pixi_envs/jazzy/pixi.toml>`

.. literalinclude:: pixi_envs/jazzy/pixi.toml
   :language: toml

Installing plugins (Pixi)
~~~~~~~~~~~~~~~~~~~~~~~~~

Just like the APT metapackage, ``ros-<distro>-easynav`` in the ``pixi.toml`` files
above only pulls in the **core** framework. Controllers, localizers, planners and
maps managers live in separate packages — browse the full catalogue at
:doc:`../plugins/index` — and must be added on top with ``pixi add``. For example:

.. code-block:: bash

   pixi add \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

Replace ``<distro>`` with your target distro (``rolling``, ``lyrical``, ``kilted`` or
``jazzy``).

.. _package_availability:

Package availability
--------------------

Packages that are not in every installation method:

.. list-table::
   :header-rows: 1
   :widths: 34 22 22 22

   * - Package
     - APT (0.4.x)
     - Pixi (0.4.x)
     - Source (``rolling``)
   * - ``easynav_recovery``, ``easynav_simple_recovery``, ``easynav_diagnostic_recovery``
     - No
     - No
     - Yes
   * - ``easynav_mhamcl_localizer``
     - No
     - No
     - Yes
   * - ``easynav_costmap_localizer``, ``easynav_simple_localizer``
     - kilted, lyrical
     - Yes
     - Yes
   * - ``navmap_tools`` (NavMap command-line tools)
     - No
     - No
     - Yes
   * - ``navmap_rviz_plugin``
     - Yes
     - No
     - Yes

The PlayGrounds, ``easynav_behaviors``, ``easynav_nav2_bridge`` and the other example packages
are only distributed as source (see :doc:`../getting_started/index`).

