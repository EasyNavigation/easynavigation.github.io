.. _install_binaries:

=====================
Install from Binaries
=====================

EasyNav is distributed as binary packages with APT (Debian packages from the ROS 2 buildfarm, the
**recommended** way, for every distro but rolling) and Pixi (self-contained conda packages, for
every distro).

.. contents:: On this page
   :local:
   :depth: 1

.. _release_status:

Release status
--------------

EasyNav **0.5.0** is released for every distro, with the same content in all of them. Each distro
has its own branch in the repositories:

.. list-table::
   :header-rows: 1
   :widths: 25 25 25 25

   * - Distro
     - APT
     - Pixi
     - Source branch
   * - rolling
     - —
     - 0.5.0
     - ``rolling``
   * - lyrical
     - 0.5.0
     - 0.5.0
     - ``lyrical``
   * - kilted
     - 0.5.0
     - 0.5.0
     - ``kilted``
   * - jazzy
     - 0.5.0
     - 0.5.0
     - ``jazzy``
   * - humble
     - 0.5.0
     - 0.5.0
     - ``humble``

.. _install_apt:

Install from binaries (APT, recommended)
----------------------------------------

EasyNav is released as Debian packages through the ROS 2 buildfarm for **humble**, **jazzy**,
**kilted** and **lyrical**. **Rolling** has no APT packages for now: use :ref:`Pixi <install_pixi>`
or :ref:`build_from_source`. It needs ROS 2 installed from its official APT repositories.

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

The :doc:`../playgrounds/index` are packaged too (except in humble), e.g.
``ros-<distro>-easynav-playground-kobuki``.

.. _install_pixi:

Install via Pixi
-----------------

Use `Pixi <https://pixi.sh>`_ when you cannot install ROS 2 from APT (another Linux, no root, or
several distros side by side). A Pixi environment is fully self-contained: it ships its own ROS 2
distribution, so you do **not** need a system ROS 2 install.

The ROS 2 packages come from `RoboStack <https://robostack.github.io>`_ (``robostack-<distro>``
channels). EasyNav's conda packages, and the few dependencies RoboStack lacks, are in the
Intelligent Robotics Lab channels on `prefix.dev <https://prefix.dev>`_:

.. note::
   EasyNav 0.5.0 will soon be available in the RoboStack channels too. Meanwhile, use the IRL
   channels below, as the ``pixi.toml`` files of this page do.

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Distro
     - IRL channel
   * - rolling
     - https://prefix.dev/fmrico/irl-rolling
   * - lyrical
     - https://prefix.dev/fmrico/irl-lyrical
   * - kilted
     - https://prefix.dev/irl-kilted
   * - jazzy
     - https://prefix.dev/irl-jazzy
   * - humble
     - https://prefix.dev/fmrico/irl-humble

.. note::
   Keep the IRL channel **before** RoboStack's in ``channels``: Pixi takes each package from the
   first channel that has it, and some RoboStack channels have older EasyNav versions.

For each ROS 2 distro, download the corresponding ``pixi.toml`` below and save it as
``~/easynav_ws/pixi.toml`` — the same workspace directory used throughout this guide
and in :doc:`../getting_started/index`. Then run:

.. code-block:: bash

   cd ~/easynav_ws
   pixi install
   pixi shell

``pixi shell`` opens a shell with ROS 2 and EasyNav ready to use. You can also prefix any command
with ``pixi run`` instead of entering the shell.

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

Humble
~~~~~~

:download:`pixi.toml <pixi_envs/humble/pixi.toml>`

.. literalinclude:: pixi_envs/humble/pixi.toml
   :language: toml

Installing plugins (Pixi)
~~~~~~~~~~~~~~~~~~~~~~~~~

Just like with APT, ``ros-<distro>-easynav`` in the ``pixi.toml`` files above only pulls in the
**core** framework. Controllers, localizers, planners and maps managers live in separate
packages — browse the full catalogue at :doc:`../plugins/index` — and must be added on top with
``pixi add``. For example:

.. code-block:: bash

   pixi add \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

Replace ``<distro>`` with your target distro (``humble``, ``jazzy``, ``kilted``, ``lyrical`` or
``rolling``).

.. _package_availability:

Package availability
--------------------

Every package of EasyNav, its plugins, NavMap (including ``navmap_tools`` and
``navmap_rviz_plugin``), yaets, ``easynav_nav2_bridge`` and the PlayGrounds is available with APT
(humble, jazzy, kilted and lyrical) and Pixi (also rolling), except:

- The **PlayGrounds** are not available for **humble**: their simulation needs the Gazebo vendor
  packages (``gz_sim_vendor``) of jazzy and later.
- ``easynav_behaviors`` and the other example packages are only distributed as source (see
  :doc:`../getting_started/index`).
