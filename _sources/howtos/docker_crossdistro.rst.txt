.. _docker_zenoh_setup:

=============================================
Docker Setup with Zenoh Bridge (Cross-Distro)
=============================================

This tutorial explains how to deploy **EasyNav** using Docker while maintaining interoperability with a robot running a different ROS 2 distribution.

The architecture described here is ideal for scenarios where the robot hardware requires an older LTS version (like **Humble**) but you want to run the latest EasyNav algorithms on a newer distribution (like **Rolling** or **Jazzy**). The **Zenoh Bridge** acts as the middleware glue that enables seamless communication between these containers and the physical hardware.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

We will set up a system composed of three main parts:

1. **The Robot Host**: Runs the hardware drivers on its native ROS 2 distribution (e.g., Humble).
2. **The Docker Container**: Runs EasyNav on ROS 2 Rolling/Jazzy using the provided image.
3. **The Remote PC**: Used for visualization (RViz2) and control.

.. note::
    Ensure that all machines (Robot, Docker Container, and PC) use the same **Zenoh** version to avoid protocol mismatches. The provided Dockerfile uses version **1.7.2**.

---

Step 1: Building the Docker Image (Rolling)
---------------------------------

First you must create the `Docker <https://docs.docker.com/engine/install/ubuntu/>`_ image containing the EasyNav stack and the Zenoh bridge. 

Create a file named ``Dockerfile`` with the following content. This configuration installs the necessary dependencies and compiles the workspace.

.. code-block:: dockerfile

    # Use the official ROS 2 Rolling image as base
    FROM ros:rolling-ros-base-noble

    # Prevent interactive prompts during installation
    ENV DEBIAN_FRONTEND=noninteractive
    # Define the workspace path
    ENV WORKSPACE=/root/easynav_ws

    # Install essential build tools, git, and networking utilities
    RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        wget \
        nano \
        unzip \
        && rm -rf /var/lib/apt/lists/*

    # Install Zenoh Bridge for ROS 2 DDS
    # Using version 1.7.2 as requested by the user
    RUN wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.7.2/zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip && \
        unzip zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip && \
        chmod +x zenoh-bridge-ros2dds && \
        mv zenoh-bridge-ros2dds /usr/local/bin/ && \
        rm zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip

    # Set up the workspace structure
    RUN mkdir -p $WORKSPACE/src
    WORKDIR $WORKSPACE

    # Clone the Easynav project and research repositories
    RUN cd src && \
        git clone -b rolling https://github.com/EasyNavigation/EasyNavigation.git  && \
        git clone -b rolling https://github.com/EasyNavigation/NavMap.git  && \
        git clone -b rolling https://github.com/EasyNavigation/easynav_plugins.git  && \
        git clone -b rolling https://github.com/fmrico/yaets.git

    # Initialize and update rosdep for dependency management
    RUN rosdep update && \
        apt-get update && \
        rosdep install --from-paths src --ignore-src -r -y && \
        rm -rf /var/lib/apt/lists/*

    # Compile the workspace using colcon
    RUN . /opt/ros/rolling/setup.sh && \
        colcon build --symlink-install

    # Configure the environment to source setups automatically
    RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc && \
        echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

    # Create an entrypoint script to launch Zenoh in the background
    RUN echo '#!/bin/bash\n\
    set -e\n\
    # Start Zenoh bridge in the background and redirect output to a log file\n\
    zenoh-bridge-ros2dds > /var/log/zenoh.log 2>&1 &\n\
    echo "Zenoh bridge started in the background."\n\
    # Execute the original command (usually bash)\n\
    exec "$@"' > /usr/local/bin/entrypoint.sh && \
    chmod +x /usr/local/bin/entrypoint.sh

    # Use the custom script as the entrypoint
    ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
    CMD ["bash"]

Build the image with the tag ``easynav_rolling``:

.. code-block:: bash

    docker build -t easynav_rolling .

---

Step 2: Preparing the Robot (Host)
----------------------------------

The robot needs to run the **zenoh-bridge-ros2dds** to expose its topics to the network efficiently. Since the robot might be running an older distro like Humble, the bridge ensures compatibility with the Rolling container.

1. **Download the Bridge**: Ensure you download the same version used in the Dockerfile (v1.7.2).


.. code-block:: bash

    wget https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.7.2/zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip
    unzip zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip
    chmod +x zenoh-bridge-ros2dds
    sudo mv zenoh-bridge-ros2dds /usr/local/bin/
    rm zenoh-plugin-ros2dds-1.7.2-x86_64-unknown-linux-gnu-standalone.zip




2. **Run the bridge on the robot**:

If you use a specific ``ROS_DOMAIN_ID``, export it before running the bridge.

.. code-block:: bash

    # Example for a robot using Domain ID 23
    export ROS_DOMAIN_ID=23
    ./zenoh-bridge-ros2dds

This command will start the bridge in peer discovery mode.

---

Step 3: Running the Container
-----------------------------

Now we launch the EasyNav container on the robot (or on a computer connected to the robot). We use ``--net=host`` and ``--ipc=host``.

.. code-block:: bash

    # Run the container with host networking and shared memory
    docker run -it --net=host --ipc=host --name easynav_test -e ROS_DOMAIN_ID=23 easynav_rolling

Inside the container you can now start the navigation system. The entrypoint script automatically starts the Zenoh bridge in the background.

.. code-block:: bash

    # Inside the container
    ros2 launch easynav_system system_main ...

.. tip::
    If you exited the container and want to restart it later use the following commands:

    .. code-block:: bash

        # Start the container
        docker start easynav_test

        # Open a new terminal in the container
        docker exec -it easynav_test bash

---

Step 4: Visualization from a Remote PC
--------------------------------------

To control the robot and visualize data from your laptop you also need the Zenoh bridge.

1. **Install the Bridge** on your PC (Check Step 2).
2. **Connect to the Robot**: Run the bridge in client mode connecting to the robot IP address.

.. code-block:: bash

    # Replace <ROBOT_IP> with the actual IP address of the robot
    # Port 7447 is the default Zenoh peering port
    zenoh-bridge-ros2dds -e tcp/<ROBOT_IP>:7447

3. **Open RViz**: Open a new terminal and set the Domain ID to match the one used in the bridge.

.. code-block:: bash

    export ROS_DOMAIN_ID=23
    ros2 run rviz2 rviz2

You should now be able to see topics from both the Humble robot (Hardware) and the Rolling Docker container (EasyNav) seamlessly integrated in RViz.
