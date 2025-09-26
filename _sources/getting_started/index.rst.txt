.. _getting_started:

Getting Started
***************

To get started with **EasyNav**, we will use a simple example: a Turtlebot2 robot navigating in a domestic environment.

1. Install EasyNav core packages. Depending on the availability of binary packages for your distro, you have two options:
   
   1. From binaries:
   
   .. code-block:: bash

      sudo apt install ros-${ROS_DISTRO}-easynav-system ros-${ROS_DISTRO}-easynav-tools

   2. From sources:
   
   .. code-block:: bash

      git clone -b ${ROS_DISTRO} https://github.com/fmrico/yaets.git  # Needed by EasyNavigation
      git clone -b ${ROS_DISTRO} https://github.com/EasyNavigation/EasyNavigation.git

2. Create your workspace and move to the `src` directory:

.. code-block:: bash

   mkdir -p easynav_ws/src
   cd easynav_ws/src

2. Clone the required repositories for this example:

.. code-block:: bash

   git clone https://github.com/EasyNavigation/easynav_simple_stack.git
   git clone https://github.com/EasyNavigation/easynav_playground_kobuki.git
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git

- The `easynav_simple_stack` repository provides a collection of plugins for a basic navigation stack. This stack is characterized by its use of a 2D occupancy grid, where each cell can be either free (`0`) or occupied (`1`).
- The `easynav_playground_kobuki` repository includes everything needed to simulate a Turtlebot2 robot, also known as Kobuki.
- The `easynav_indoor_testcase` repository provides you with maps and param files for some test cases.

Some of these repositories specify additional dependencies via `.repos` files. To retrieve all third-party dependencies, run:

.. code-block:: bash

   vcs import . < easynav_playground_kobuki/thirdparty.repos

1. Install dependencies and build the workspace:

.. code-block:: bash

   cd ..  # go back to easynav_ws
   rosdep install --from-paths src --ignore-src -r -y
   reset && colcon build --symlink-install
   source install/setup.bash 


4. Launch the simulator

To start the simulation, run:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_kobuki.launch.py

This command launches a Gazebo simulation of a Turtlebot2 robot in a domestic environment.

.. image:: ../images/kobuki_sim.png
   :align: center
   :alt: Turtlebot2 simulation in Gazebo


If you want to launch the simulator without its graphical interface (useful to save computational resources), you can disable the GUI with the `gui:=false` argument:

.. code-block:: bash

   ros2 launch easynav_playground_kobuki playground_kobuki.launch.py gui:=false

When used together with RViz2, visualizing the simulation is not strictly necessary.

5. Launch EasyNav


Keep the simulator running.

Open a new terminal to launch the EasyNav system. EasyNav is launched through a single executable, where you pass a parameter file indicating which plugins to load for each module:

.. code-block:: bash

   ros2 run easynav_system system_main --ros-args --params-file ~/easynav_ws/src/easynav_indoor_testcase/robots_params/simple.params.yaml

In another terminal, open RViz2:

.. code-block:: bash

   ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

.. image:: ../images/kobuki_simple.png 
   :align: center
   :alt: RViz2 with EasyNav loaded

In RViz2, add the visualizations shown in the figure above. Make sure to set the **QoS** of the map topic to **Transient Local** so the map is correctly displayed.

Once RViz2 is properly configured, you can use the **"2D Goal Pose"** tool (located in the top toolbar) to send navigation goals to the robot. Click on a location in the map to make the robot navigate there autonomously.

.. image:: ../images/kobuki_simple_navigating.png
   :align: center
   :alt: Turtlebot2 navigating using EasyNav


Congratulations! 🎉

If you have reached this point successfully, you now have **EasyNav** up and running!

You are ready to start experimenting with navigation stacks, environment representations, and advanced behaviors. Check the following sections to explore more use cases and configuration options.