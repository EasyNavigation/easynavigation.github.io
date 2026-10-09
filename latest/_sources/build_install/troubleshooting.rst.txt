.. _install_troubleshooting:

===============
Troubleshooting
===============

Common problems
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
     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

Uninstall / clean
-----------------

Since this is a workspace overlay, you can remove it safely:

.. code-block:: bash

   rm -rf ~/easynav_ws

Running the tests
-----------------

.. code-block:: bash

   cd ~/easynav_ws
   colcon test --parallel-workers 1 --ctest-args -R easynav  # run EasyNav-related tests
   colcon test-result --verbose
