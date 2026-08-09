
.. _build_and_install:

=================
Build & Install
=================

This page explains how to install and build the **EasyNavigation (EasyNav)** framework and how to set up your development environment.

.. contents:: On this page
   :local:
   :depth: 2

Supported platforms
-------------------

EasyNav targets modern Linux distributions and the following ROS 2 releases:

- **rolling** — tracks the latest supported Ubuntu release
- **lyrical** — Ubuntu 26.04 (Resolute)
- **kilted** — Ubuntu 24.04 (Noble)
- **jazzy** — Ubuntu 24.04 (Noble)

.. note::
   If you are using a different ROS 2 release, contributions to extend the support
   matrix are very welcome.

Installation methods
---------------------

EasyNav can be installed in three ways:

- :ref:`install_apt` — precompiled Debian packages. Available for **jazzy**,
  **kilted** and **lyrical** (not **rolling**).
- :ref:`install_pixi` — precompiled Pixi/conda packages, self-contained (bundles
  its own ROS 2). Available for **rolling**, **jazzy**, **kilted** and **lyrical**.
- :ref:`build_from_source` — clone and build with colcon. Available for all four
  supported distros.

Prerequisites
-------------

The prerequisites below apply to the **APT** and **build from source** methods. If you
install via **Pixi**, everything (including ROS 2 itself) is provided by the Pixi
environment and no system-wide ROS 2 installation is required — you can skip ahead to
:ref:`install_pixi`.

1. ROS 2 (jazzy, kilted, lyrical or rolling)

   Follow the official ROS 2 installation instructions for your platform.
   Ensure your ROS 2 environment is sourced before building EasyNav.

   .. code-block:: bash

      # Example (adjust to your ROS 2 distro):
      source /opt/ros/kilted/setup.bash

2. ROS dependencies

   .. code-block:: bash

      sudo rosdep init
      rosdep update

.. _install_apt:

Install from binaries (APT)
----------------------------

EasyNav is released as binary Debian packages through the ROS 2 buildfarm for
**jazzy**, **kilted** and **lyrical**.

.. note::
   **Rolling** does not have APT/binary packages, since ROS 2 Rolling is not released
   through the Debian buildfarm. Use :ref:`install_pixi` or
   :ref:`build_from_source` instead.

Jazzy
~~~~~

.. code-block:: bash

   sudo apt update
   sudo apt install ros-jazzy-easynav

Kilted
~~~~~~

.. code-block:: bash

   sudo apt update
   sudo apt install ros-kilted-easynav

Lyrical
~~~~~~~

.. code-block:: bash

   sudo apt update
   sudo apt install ros-lyrical-easynav

Installing plugins (APT)
~~~~~~~~~~~~~~~~~~~~~~~~

``ros-<distro>-easynav`` only installs the **core** EasyNav framework (``easynav_system``,
``easynav_sensors``, ...). Controllers, localizers, planners and maps managers are
shipped as separate packages — see the full catalogue at :doc:`../plugins/index` — and
you need to install the ones your configuration actually uses.

For example, the ``costmap`` + ``rpp`` example configuration
(``easynav_indoor_testcase/robots_params/costmap.rpp.params.yaml``) needs:

.. code-block:: bash

   sudo apt install \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

And the ``simple`` + ``serest`` example configuration
(``easynav_indoor_testcase/robots_params/simple.serest_params.yaml``) needs:

.. code-block:: bash

   sudo apt install \
     ros-<distro>-easynav-simple-maps-manager \
     ros-<distro>-easynav-simple-localizer \
     ros-<distro>-easynav-simple-planner \
     ros-<distro>-easynav-serest-controller

.. note::
   Plugin package availability via APT currently varies per distro: as of this
   writing, ``easynav-costmap-localizer``, ``easynav-regulated-pp-controller`` and
   ``easynav-simple-localizer`` are only published for **lyrical**. If a plugin
   package is missing for your distro, use :ref:`install_pixi` (which has broader
   plugin coverage) or :ref:`build_from_source`.

.. _install_pixi:

Install via Pixi
-----------------

EasyNav publishes prebuilt `Pixi <https://pixi.sh>`_/conda packages on
`prefix.dev <https://prefix.dev>`_. A Pixi environment is fully self-contained: it
ships its own ROS 2 distribution, so you do **not** need a system ROS 2 install.

For each ROS 2 distro, download the corresponding ``pixi.toml`` below and save it as
``~/easynav_ws/pixi.toml`` — the same workspace directory used throughout this guide
and in :doc:`../getting_started/index`. Then run:

.. code-block:: bash

   cd ~/easynav_ws
   pixi install
   pixi shell

``pixi shell`` opens a shell with ROS 2 and EasyNav ready to use (e.g. ``ros2 launch
easynav ...``). You can also prefix any command with ``pixi run`` instead of entering
the shell.

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
:doc:`../plugins/index` — and must be added on top with ``pixi add``.

For example, to run the ``costmap`` + ``rpp`` example configuration
(``easynav_indoor_testcase/robots_params/costmap.rpp.params.yaml``):

.. code-block:: bash

   pixi add \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

And for the ``simple`` + ``serest`` example configuration
(``easynav_indoor_testcase/robots_params/simple.serest_params.yaml``):

.. code-block:: bash

   pixi add \
     ros-<distro>-easynav-simple-maps-manager \
     ros-<distro>-easynav-simple-localizer \
     ros-<distro>-easynav-simple-planner \
     ros-<distro>-easynav-serest-controller

Replace ``<distro>`` with your target distro (``rolling``, ``jazzy``, ``kilted`` or
``lyrical``). Unlike APT, these plugin packages are available on the Pixi channels
for all four distros.

.. _build_from_source:

Build from source
-------------------

Workspace layout
~~~~~~~~~~~~~~~~

We recommend a standard ROS 2 workspace:

.. code-block:: bash

   mkdir -p ~/easynav_ws/src
   cd ~/easynav_ws

Clone sources
~~~~~~~~~~~~~

You can retrieve EasyNav sources by cloning the monorepo(s) you need. Each repository
has one branch per supported ROS 2 distro — pick the block matching your target distro.

.. note::
   Unlike the APT and Pixi methods, cloning ``easynav_plugins`` already brings in
   **all** official plugins (see :doc:`../plugins/index`) — ``colcon build`` will
   build every controller, localizer, planner and maps manager, so no extra
   installation step is needed here.

Rolling
^^^^^^^

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone -b rolling https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b rolling https://github.com/EasyNavigation/NavMap.git
   git clone -b rolling https://github.com/EasyNavigation/easynav_plugins.git
   git clone -b rolling https://github.com/fmrico/yaets.git

Lyrical
^^^^^^^

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone -b lyrical https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b lyrical https://github.com/EasyNavigation/NavMap.git
   git clone -b lyrical https://github.com/EasyNavigation/easynav_plugins.git
   git clone -b lyrical https://github.com/fmrico/yaets.git

Kilted
^^^^^^

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone -b kilted https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b kilted https://github.com/EasyNavigation/NavMap.git
   git clone -b kilted https://github.com/EasyNavigation/easynav_plugins.git
   git clone -b kilted https://github.com/fmrico/yaets.git

Jazzy
^^^^^

.. code-block:: bash

   cd ~/easynav_ws/src
   git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b jazzy https://github.com/EasyNavigation/NavMap.git
   git clone -b jazzy https://github.com/EasyNavigation/easynav_plugins.git
   git clone -b jazzy https://github.com/fmrico/yaets.git

Install dependencies
~~~~~~~~~~~~~~~~~~~~

From the workspace root, resolve all package dependencies with rosdep:

.. code-block:: bash

   cd ~/easynav_ws
   rosdep install --from-paths src --ignore-src -y -r

Configure and build
~~~~~~~~~~~~~~~~~~~

Use colcon to build the workspace. You may enable symlink-install for faster iteration.

.. code-block:: bash

   cd ~/easynav_ws
   colcon build --symlink-install

Source the overlay
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Source ROS 2 first (jazzy / kilted / lyrical / rolling)
   source /opt/ros/<distro>/setup.bash
   # Then source the workspace
   source ~/easynav_ws/install/setup.bash

Run tests (optional)
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~/easynav_ws
   colcon test --ctest-args -R easynav  # run EasyNav-related tests
   colcon test-result --verbose


Troubleshooting
---------------

- **Missing rosdep keys**

  Run ``rosdep check --from-paths src --ignore-src`` to diagnose. If a dependency
  is truly missing on your platform, consider opening an issue with details.

- **CMake not finding ROS packages**

  Ensure you have sourced the correct ROS 2 distro and your workspace install
  before building or running executables.

  .. code-block:: bash

     source /opt/ros/<distro>/setup.bash
     source ~/easynav_ws/install/setup.bash

- **ABI / compiler issues**

  Remove the build, install, and log folders and rebuild:

  .. code-block:: bash

     cd ~/easynav_ws
     rm -rf build install log
     colcon build --merge-install

Uninstall / clean
-----------------

Since this is a workspace overlay, you can remove it safely:

.. code-block:: bash

   rm -rf ~/easynav_ws

Next steps
----------

- :doc:`../getting_started/index` — quick start with simulation and first launch
- :doc:`../howtos/index` — step-by-step guides for mapping, navigation, and deployment
- :doc:`../developer_guide/index` — in-depth documentation for developers and contributors

.. toctree::
   :hidden:
