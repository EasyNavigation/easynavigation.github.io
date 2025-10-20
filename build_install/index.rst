
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

EasyNav targets modern Linux distributions and ROS 2 releases:

- **Ubuntu 24.04 (Noble)** — ROS 2 *jazzy* (primary CI target)
- **Ubuntu 24.04 (Noble)** — ROS 2 *kilted*
- **Ubuntu 24.04 (Noble)** — ROS 2 *rolling*
- Other platforms may work but are not actively tested.

.. note::
   If you are using a different distribution or ROS 2 release, contributions to extend
   the support matrix are very welcome.

Prerequisites
-------------


1. ROS 2 (jazzy, kilted or rolling)

   Follow the official ROS 2 installation instructions for your platform.
   Ensure your ROS 2 environment is sourced before building EasyNav.

   .. code-block:: bash

      # Example (adjust to your ROS 2 distro):
      source /opt/ros/jazzy/setup.bash

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
      sudo apt install ros-jazzy-easynav

Build from source
-----------------

Workspace layout
~~~~~~~~~~~~~~~~

We recommend a standard ROS 2 workspace:

.. code-block:: bash

   mkdir -p ~/easynav_ws/src
   cd ~/easynav_ws

Clone sources
~~~~~~~~~~~~~

You can retrieve EasyNav sources by cloning the monorepo(s) you need:

.. code-block:: bash

   # Adjust workspace dir and branches to your needs
   cd ~/easynav_ws/src
   # Clone the main EasyNavigation repositories
   git clone -b jazzy https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b jazzy https://github.com/EasyNavigation/NavMap.git
   git clone -b jazzy https://github.com/EasyNavigation/easynav_plugins.git
   # Clone yaets dependency
   git clone -b jazzy-devel https://github.com/fmrico/yaets.git


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

   # Source ROS 2 first (jazzy / kilted / rolling)
   source /opt/ros/jazzy/setup.bash
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

     source /opt/ros/jazzy/setup.bash
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
