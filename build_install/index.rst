
.. _build_and_install:

=================
Build & Install
=================

This page explains how to install and build the **EasyNavigation (EasyNav)** framework.
If you are new here, start with :doc:`../getting_started/index` for a high‑level overview,
then return to this guide to set up your development environment.

.. contents:: On this page
   :local:
   :depth: 2

Supported platforms
-------------------

EasyNav targets modern Linux distributions and ROS 2 releases:

- **Ubuntu 24.04 (Noble)** — ROS 2 *kilted* (primary CI target)
- **Ubuntu 24.04 (Noble)** — ROS 2 *rolling*
- Other platforms may work but are not actively tested.

.. note::
   If you are using a different distribution or ROS 2 release, contributions to extend
   the support matrix are very welcome.

Prerequisites
-------------


1. ROS 2 (kilted or rolling)

   Follow the official ROS 2 installation instructions for your platform.
   Ensure your ROS 2 environment is sourced before building EasyNav.

   .. code-block:: bash

      # Example (adjust to your ROS 2 distro):
      source /opt/ros/kilted/setup.bash

2. ROS dependencies

   .. code-block:: bash

      sudo rosdep init
      rosdep update

Install from binaries (APT)
---------------------------

Binary packages will be provided via APT for Ubuntu + ROS 2 as they become available.

.. admonition:: Coming soon
   :class: hint

   Prebuilt Debian packages are planned. Once published, you will be able to run:

   .. code-block:: bash

      sudo apt update
      sudo apt install ros-kilted-easynav

Build from source
-----------------

Workspace layout
~~~~~~~~~~~~~~~~

We recommend a standard ROS 2 workspace:

.. code-block:: bash

   mkdir -p ~/ros/ros2/easynav_ws/src
   cd ~/ros/ros2/easynav_ws

Clone sources
~~~~~~~~~~~~~

You can retrieve EasyNav sources either by cloning the monorepo(s) you need:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws/src
   # Core/meta repositories (examples — adjust to your needs)
   git clone https://github.com/EasyNavigation/EasyNavigation.git
   git clone https://github.com/EasyNavigation/easynav_plugins.git
   git clone https://github.com/EasyNavigation/NavMap.git


Install dependencies
~~~~~~~~~~~~~~~~~~~~

From the workspace root, resolve all package dependencies with rosdep:

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   rosdep install --from-paths src --ignore-src -y -r

Configure and build
~~~~~~~~~~~~~~~~~~~

Use colcon to build the workspace. You may enable symlink-install for faster iteration.

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   colcon build --symlink-install

Source the overlay
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Source ROS 2 first (kilted / rolling)
   source /opt/ros/kilted/setup.bash
   # Then source the workspace
   source ~/ros/ros2/easynav_ws/install/setup.bash

Run tests (optional)
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   cd ~/ros/ros2/easynav_ws
   colcon test --ctest-args -R easynav  # run EasyNav-related tests
   colcon test-result --verbose

Common options
--------------

- **Release build:** ``--cmake-args -DCMAKE_BUILD_TYPE=Release``
- **Verbose build:** ``--event-handlers console_cohesion+``
- **Single package:** ``--packages-select <pkg_name>``
- **Parallel jobs:** ``--parallel-workers $(nproc)``

Troubleshooting
---------------

- **Missing rosdep keys**

  Run ``rosdep check --from-paths src --ignore-src`` to diagnose. If a dependency
  is truly missing on your platform, consider opening an issue with details.

- **CMake not finding ROS packages**

  Ensure you have sourced the correct ROS 2 distro and your workspace install
  before building or running executables.

  .. code-block:: bash

     source /opt/ros/kilted/setup.bash
     source ~/ros/ros2/easynav_ws/install/setup.bash

- **ABI / compiler issues**

  Remove the build, install, and log folders and rebuild:

  .. code-block:: bash

     cd ~/ros/ros2/easynav_ws
     rm -rf build install log
     colcon build --merge-install

Uninstall / clean
-----------------

Since this is a workspace overlay, you can remove it safely:

.. code-block:: bash

   rm -rf ~/ros/ros2/easynav_ws

Next steps
----------

- :doc:`../getting_started/index` — quick start with simulation and first launch  
- :doc:`../howtos/index` — step-by-step guides for mapping, navigation, and deployment  
- :doc:`../developer_guide/index` — in-depth documentation for developers and contributors

.. toctree::
   :hidden: