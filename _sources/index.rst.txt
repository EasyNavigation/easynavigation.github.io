.. _documentation_home:

*****
|LPN|
*****

Overview
########

**EasyNavigation (EasyNav)** is an open-source navigation system for ROS 2, designed to be:

- ✅ **Representation-agnostic**, supporting a wide variety of environment models: 2D costmaps, elevation-aware gridmaps, Octomap-based 3D representations, raw point clouds, or hybrid combinations.
- ⚡ **Real-time capable**, minimizing latency between perception and action.
- 🧩 **Modular**, through a plugin architecture and reusable navigation stacks.
- 🚀 **Lightweight and simple to deploy**, using a single binary and a parameter file for configuration.
- 🧪 **Simulation-ready**, thanks to a rich collection of **PlayGrounds** with different robots and environments.

EasyNav is developed by the [Intelligent Robotics Lab](https://intelligentroboticslab.gsyc.urjc.es/) at Universidad Rey Juan Carlos and aims to be a flexible, extensible, and practical alternative to existing ROS 2 navigation stacks like Nav2.

It is structured around:

- A **core**, responsible for real-time data processing and behavior execution.
- **Stacks**, which specialize the system for different environment types or use cases.
- **Plugins**, that define localization, path planning, control, and map representation methods.
- **PlayGrounds**, which offer complete simulation scenarios with robots, maps, and example configurations.

Whether you are building robots for indoor or outdoor environments, structured or unstructured terrains, EasyNav provides the flexibility and performance needed to achieve robust autonomous navigation.

We invite you to explore, use, and contribute to this project!

📚 Learn more about the contributors and project organization in :ref:`about`.

📦 Source code: `github.com/EasyNavigation/EasyNavigation <https://github.com/EasyNavigation/EasyNavigation>`_

📢 Want to contribute? Feel free to open an issue, suggest a feature, or submit a pull request!

If you use this software in your work, please consider citing our next paper.

.. toctree::
   :hidden:

   getting_started/index.rst
   build_instructions/index.rst
   about/index.rst
