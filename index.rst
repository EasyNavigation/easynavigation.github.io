:hide-toc:

.. _documentation_home:

.. rst-class:: en-home

*****
|LPN|
*****

.. raw:: html

   <div class="en-hero">
     <div class="en-hero__body">
       <span class="en-hero__kicker">Open-source navigation for ROS 2</span>
       <h1>Robots navigating in minutes, <em>predictable</em> on real hardware</h1>
       <p>EasyNavigation is a modular, real-time and representation-agnostic navigation system:
          costmaps, NavMap 3D surfaces or Bonxai voxel maps, one binary and one parameter file.</p>
       <a class="md-button md-button--primary" href="getting_started/index.html">Get started</a>
       <a class="md-button" href="playgrounds/index.html">Try a PlayGround</a>
       <a class="md-button" href="plugins/index.html">Browse the plugins</a>
     </div>
     <div class="en-hero__logo"><img src="_images/easynav_logo.png" alt="EasyNav logo"></div>
   </div>

   <div class="en-stats">
     <div class="en-stat"><span class="en-stat__value">3</span><span class="en-stat__label">Map representations</span></div>
     <div class="en-stat"><span class="en-stat__value">23</span><span class="en-stat__label">Navigation plugins</span></div>
     <div class="en-stat"><span class="en-stat__value">6</span><span class="en-stat__label">Simulated PlayGrounds</span></div>
     <div class="en-stat"><span class="en-stat__value">4</span><span class="en-stat__label">ROS 2 distributions</span></div>
   </div>

.. image:: images/easynav_logo.png
   :class: en-hidden

Where do you want to start?
###########################

.. raw:: html

   <div class="en-cards">
     <a class="en-card" href="getting_started/index.html">
       <span class="en-card__icon">🚀</span><span class="en-card__title">New to EasyNav</span>
       <span class="en-card__text">Install it and get a simulated robot navigating with a single launch file.</span>
       <span class="en-card__go">Getting started →</span></a>
     <a class="en-card" href="howtos/index.html">
       <span class="en-card__icon">🧪</span><span class="en-card__title">Research and teaching</span>
       <span class="en-card__text">Swap planners, controllers, localizers and maps, and compare methods on the same robot.</span>
       <span class="en-card__go">HowTos →</span></a>
     <a class="en-card" href="developer_guide/safety.html">
       <span class="en-card__icon">🛡️</span><span class="en-card__title">Industry and real robots</span>
       <span class="en-card__text">Safety mode, real-time monitoring, heartbeat and recovery for deployments.</span>
       <span class="en-card__go">Safety mode →</span></a>
     <a class="en-card" href="developer_guide/index.html">
       <span class="en-card__icon">🧩</span><span class="en-card__title">Plugin developers</span>
       <span class="en-card__text">Learn the core design and the blackboard, and write your own plugins.</span>
       <span class="en-card__go">Developers guide →</span></a>
   </div>

Overview
########

**EasyNavigation (EasyNav)** is an open-source navigation system for ROS 2, designed to be:

.. raw:: html

   <div class="en-features">
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Representation-agnostic</b>
       <span>2D costmaps, NavMap 3D navigable surfaces, Bonxai probabilistic voxel maps, or hybrid combinations.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Real-time capable</b>
       <span>Minimal latency between perception and action.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Modular</b>
       <span>A plugin architecture and reusable navigation stacks.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Lightweight and simple to deploy</b>
       <span>A single binary and a parameter file for configuration.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Simulation-ready</b>
       <span>A rich collection of PlayGrounds with different robots and environments.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Built for safety and runtime reconfiguration</b>
       <span><a href="developer_guide/safety.html">Safety mode</a> next to a certified safety channel, and a
       <a href="developer_guide/recovery.html">recovery system</a> that detects failures and reacts to them.</span></div></div>
     <div class="en-feature"><i class="en-feature__dot"></i><div><b>Ready for every user</b>
       <span>Hobbyists, academia and research labs, and industry that needs predictable, safe behavior.</span></div></div>
   </div>

EasyNav is developed by the `Intelligent Robotics Lab <https://intelligentroboticslab.gsyc.urjc.es/>`_ at Universidad Rey Juan Carlos and aims to be a flexible, extensible, and practical alternative to existing ROS 2 navigation stacks like Nav2.

We believe in Open Source and in giving the community multiple options to choose from, that adapt to different scenarios and constraints. **EasyNav is not intended to replace Nav2**, which we admire and take as inspiration. Instead, it aims to offer flexibility and meet specific requirements in areas where Nav2 might be less suitable.

How the pieces fit
##################

.. raw:: html

   <div class="en-arch">
     <div class="en-arch__box en-arch__box--core"><b>Core</b>
       <span>Real-time data processing and behavior execution.</span></div>
     <div class="en-arch__box"><b>Stacks</b>
       <span>Specialize the system for different environment types or use cases.</span></div>
     <div class="en-arch__box"><b><a href="plugins/index.html">Plugins</a></b>
       <span>Localization, path planning, control and map representation methods.</span></div>
     <div class="en-arch__box"><b><a href="playgrounds/index.html">PlayGrounds</a></b>
       <span>Complete simulations with robots, maps and example configurations.</span></div>
   </div>

Whether you are building robots for indoor or outdoor environments, structured or unstructured terrains, EasyNav provides the flexibility and performance needed to achieve robust autonomous navigation.

See it running
##############

.. raw:: html

   <div class="en-videos">
     <iframe src="https://www.youtube.com/embed/MssaLixuv2g?mute=1" title="EasyNav video 1" allowfullscreen loading="lazy"></iframe>
     <iframe src="https://www.youtube.com/embed/1GeKgw3Pbr4?mute=1" title="EasyNav video 2" allowfullscreen loading="lazy"></iframe>
   </div>

Get involved
############

📚 Learn more about the contributors and project organization in :ref:`about`.

📦 Source code: `github.com/EasyNavigation/EasyNavigation <https://github.com/EasyNavigation/EasyNavigation>`_

If you use this software in your work, please consider citing our next paper.

.. raw:: html

   <div class="en-cta">
     <p><b>Want to contribute?</b> Open an issue, suggest a feature, or submit a pull request.</p>
     <a class="md-button md-button--primary" href="https://github.com/EasyNavigation/EasyNavigation">Contribute on GitHub</a>
   </div>

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
