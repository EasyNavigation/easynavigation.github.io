.. _build_and_install:

=================
Build & Install
=================

EasyNav runs on Linux with ROS 2 **humble**, **jazzy**, **kilted**, **lyrical** or **rolling**.
The recommended way to install it is with **APT**, using the packages of the ROS 2 buildfarm
(all distros but rolling, which has no APT packages for now: use Pixi or build from source).

.. only:: not latest

   This documentation describes **EasyNav 0.5.0**, the version that APT installs on every distro.

.. only:: latest

   This is the **development** documentation, for the ``rolling`` branches of the repositories:
   it may describe features not released yet. APT installs the latest release, **EasyNav 0.5.0**.

.. _install_quick_apt:

Install with APT (recommended)
------------------------------

.. warning::
   EasyNav 0.5.0 was released to the ROS 2 buildfarm on **October 8, 2026**. Its APT packages are
   available after the next sync of each distro: until then, APT installs the previous version
   (0.4.0 on jazzy, 0.4.1 on kilted, 0.4.2 on lyrical) and nothing on humble. Meanwhile, install
   0.5.0 with :ref:`Pixi <install_pixi>` or :ref:`build it from source <build_from_source>`.

You need `ROS 2 <https://docs.ros.org>`_ installed from its official APT repositories. Install
EasyNav and the plugins your configuration uses; for example, for a costmap configuration with the
Regulated Pure Pursuit controller:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-<distro>-easynav \
     ros-<distro>-easynav-costmap-maps-manager \
     ros-<distro>-easynav-costmap-localizer \
     ros-<distro>-easynav-costmap-planner \
     ros-<distro>-easynav-regulated-pp-controller

Replace ``<distro>`` with yours (``humble``, ``jazzy``, ``kilted`` or ``lyrical``) and
source ROS 2 in every new terminal (``source /opt/ros/<distro>/setup.bash``). See
:ref:`install_apt` for the plugins and the PlayGrounds, and continue with
:doc:`../getting_started/index`.

On **rolling**, without a system ROS 2 or on another Linux, use :ref:`Pixi <install_pixi>`. To
develop EasyNav or a plugin, build it from source.

.. _build_from_source:

Install from source
-------------------

You need `ROS 2 <https://docs.ros.org>`_ installed.

.. only:: not latest

   The commands clone the **EasyNav 0.5.0** release (its git tags), the same on every distro.

.. only:: latest

   The commands clone the ``rolling`` branches (development). On another distro, clone its branch
   instead (``-b humble``, ``-b jazzy``, ``-b kilted`` or ``-b lyrical``), see
   :ref:`release_status`.

1. **Create a workspace and clone EasyNav**, its plugins, NavMap and yaets (the tracing library
   EasyNav uses):

   .. only:: not latest

      .. code-block:: bash

         mkdir -p ~/easynav_ws/src
         cd ~/easynav_ws/src
         git clone -b 0.5.0 https://github.com/EasyNavigation/EasyNavigation.git
         git clone -b 0.5.0 https://github.com/EasyNavigation/easynav_plugins.git
         git clone -b 0.6.0 https://github.com/EasyNavigation/NavMap.git
         git clone -b 1.2.0 https://github.com/fmrico/yaets.git

   .. only:: latest

      .. code-block:: bash

         mkdir -p ~/easynav_ws/src
         cd ~/easynav_ws/src
         git clone -b rolling https://github.com/EasyNavigation/EasyNavigation.git
         git clone -b rolling https://github.com/EasyNavigation/easynav_plugins.git
         git clone -b rolling https://github.com/EasyNavigation/NavMap.git
         git clone -b rolling https://github.com/fmrico/yaets.git

2. **Install the dependencies**:

   .. code-block:: bash

      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -y -r

   (If you never used rosdep, run ``sudo rosdep init`` and ``rosdep update`` first.)

3. **Build**:

   .. code-block:: bash

      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

4. **Source the workspace**, in every new terminal:

   .. code-block:: bash

      source /opt/ros/<distro>/setup.bash
      source ~/easynav_ws/install/setup.bash

That's it: EasyNav and all its official plugins are installed. Continue with
:doc:`../getting_started/index`.

Other options
-------------

- :doc:`binaries` — APT and Pixi packages: plugins, PlayGrounds and the versions available for each
  distro.
- :doc:`realtime` — let EasyNav run its control cycle with real-time priority (recommended on a
  real robot).
- :doc:`troubleshooting` — common problems, running the tests and uninstalling.

.. toctree::
   :hidden:

   binaries
   realtime
   troubleshooting
