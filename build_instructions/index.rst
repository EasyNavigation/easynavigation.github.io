.. _build-instructions:

Build and Install
#################

Install
*******

EasyNav and has not yet binary packages.

Build
*****

Install ROS
-----------

Please install ROS 2 via the usual `build instructions <https://index.ros.org/doc/ros2/Installation>`_ for your desired distribution.

Build EasyNav
--------------

Create a new workspace, ``easynav_ws``, and clone EasyNav master branch into it and build it. 


.. code:: bash

  mkdir -p ~/easynav_ws/src
  cd ~/easynav_ws/src
  git clone --recursive https://github.com/EasyNavigation/EasyNavigation.git
    
  cd ~/easynav_ws
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro <ros2-distro>
  colcon build --symlink-install
