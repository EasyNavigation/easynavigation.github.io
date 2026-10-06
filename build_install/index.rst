.. _build_and_install:

=================
Build & Install
=================

EasyNav runs on Linux with ROS 2 **rolling**, **lyrical**, **kilted** or **jazzy**. The
recommended way to install it is to build it from source, in a few minutes.

.. _build_from_source:

Install from source
-------------------

You need `ROS 2 <https://docs.ros.org>`_ installed. The commands use **rolling**, the branch this
documentation describes (EasyNav 0.5.0, soon released for all distros).
On another distro, see :ref:`release_status` for the branches available.

1. **Create a workspace and clone EasyNav**, its plugins, NavMap and yaets (the tracing library
   EasyNav uses):

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

      source /opt/ros/rolling/setup.bash
      source ~/easynav_ws/install/setup.bash

That's it: EasyNav and all its official plugins are installed. Continue with
:doc:`../getting_started/index`.

Other options
-------------

- :doc:`binaries` — APT and Pixi packages, with the versions and packages available for each
  distro.
- :doc:`realtime` — let EasyNav run its control cycle with real-time priority (recommended on a
  real robot).
- :doc:`troubleshooting` — common problems, running the tests and uninstalling.

.. toctree::
   :hidden:

   binaries
   realtime
   troubleshooting
