.. _documentation_home:

*****
|LPN|
*****

.. raw:: html

    <h1 align="center">
      <div>
        <div style="position: relative; padding-bottom: 0%; overflow: hidden; max-width: 100%; height: auto;">
          <iframe width="450" height="300" src="https://www.youtube.com/embed/MssaLixuv2g?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
          <iframe width="450" height="300" src="https://www.youtube.com/embed/1GeKgw3Pbr4?autoplay=1&mute=1" frameborder="1" allowfullscreen></iframe>
        </div>
      </div>
    </h1>


Overview
########

**EasyNavigation (EasyNav)** is an open-source navigation system for ROS 2, designed to be:

- ✅ **Representation-agnostic**, supporting a wide variety of environment models: 2D costmaps, NavMap 3D navigable surfaces, Bonxai probabilistic voxel maps, or hybrid combinations.
- ⚡ **Real-time capable**, minimizing latency between perception and action.
- 🧩 **Modular**, through a plugin architecture and reusable navigation stacks.
- 🚀 **Lightweight and simple to deploy**, using a single binary and a parameter file for configuration.
- 🧪 **Simulation-ready**, thanks to a rich collection of **PlayGrounds** with different robots and environments.
- 🛡️ **Built for safety and runtime reconfiguration**: a :ref:`safety mode <safety>` to deploy it next to a certified safety channel (validated configuration, velocity limits, real-time monitoring, heartbeat), and a :ref:`recovery system <recovery>` that detects failures and reacts to them, even reconfiguring the navigation while it runs.
- 🎯 **Ready for every user**: hobbyists who want a robot navigating in minutes, academia and research labs that need to experiment with new methods, and industry that needs predictable, safe behavior on real robots.

EasyNav is developed by the `Intelligent Robotics Lab <https://intelligentroboticslab.gsyc.urjc.es/>`_ at Universidad Rey Juan Carlos and aims to be a flexible, extensible, and practical alternative to existing ROS 2 navigation stacks like Nav2.

We believe in Open Source and in giving the community multiple options to choose from, that adapt to different scenarios and constraints. **EasyNav is not intended to replace Nav2**, which we admire and take as inspiration. Instead, it aims to offer flexibility and meet specific requirements in areas where Nav2 might be less suitable.

It is structured around:

- A **core**, responsible for real-time data processing and behavior execution.
- **Stacks**, which specialize the system for different environment types or use cases.
- **Plugins**, that define localization, path planning, control, and map representation methods.
- :doc:`PlayGrounds <playgrounds/index>`, which offer complete simulation scenarios with robots, maps, and example configurations: indoors, outdoors and with omnidirectional robots.

Whether you are building robots for indoor or outdoor environments, structured or unstructured terrains, EasyNav provides the flexibility and performance needed to achieve robust autonomous navigation.

We invite you to explore, use, and contribute to this project!

🧭 Already using Nav2? The :ref:`migration_nav2` shows how to try EasyNav on your robot.

📚 Learn more about the contributors and project organization in :ref:`about`.

📦 Source code: `github.com/EasyNavigation/EasyNavigation <https://github.com/EasyNavigation/EasyNavigation>`_

📢 Want to contribute? Feel free to open an issue, suggest a feature, or submit a pull request!

If you use this software in your work, please consider citing our next paper.

.. toctree::
   :hidden:

   build_install/index.rst
   getting_started/index.rst
   playgrounds/index.rst
   plugins/index.rst
   howtos/index.rst
   developer_guide/index.rst
   migration_guide/index.rst
   about/index.rst
