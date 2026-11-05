.. _playgrounds_install:

============
Installation
============

Install the PlayGround you want with APT in jazzy, kilted or lyrical, or with
:ref:`Pixi <install_pixi>` (also rolling). They are not available for humble. Each one brings its robot, worlds and EasyNav launchers:

.. code-block:: bash

   sudo apt install ros-<distro>-easynav-playground-kobuki

.. only:: latest

   .. note::

      The Ackermann and Aerial PlayGrounds are new in this development version: until the next
      release they are only available from source (below).

To modify them, build them from source instead: clone the repository into your workspace, install
their dependencies and build them (all, or ``--packages-up-to`` the one you want):

.. only:: not latest

   .. code-block:: bash

      cd ~/easynav_ws/src
      git clone -b 0.5.0 https://github.com/EasyNavigation/easynav_playgrounds.git
      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      source install/setup.bash

.. only:: latest

   .. code-block:: bash

      cd ~/easynav_ws/src
      git clone -b rolling https://github.com/EasyNavigation/easynav_playgrounds.git
      cd ~/easynav_ws
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      source install/setup.bash

Every PlayGround's launch files accept, at least, ``params_file`` (EasyNav parameter file) and
``rviz_config`` (RViz2 configuration), so you can try your own parameters on a PlayGround robot:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki easynav_costmap_rpp.launch.yaml \
     params_file:=/path/to/my.params.yaml
