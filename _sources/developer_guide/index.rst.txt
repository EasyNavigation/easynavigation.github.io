.. developer_guide:

================
Developers Guide
================

The **EasyNav Developers Guide** provides a technical overview of the internal architecture of the framework and its main design principles.  
It is intended for developers who wish to understand the codebase, extend the system with new modules, or contribute to its evolution.

EasyNav is designed as a lightweight, modular navigation framework built on ROS 2.  
Its architecture relies on clear interfaces, shared data structures, and composable components that can operate in both simulation and real robotic platforms.

.. contents::
   :local:

.. toctree::
   :maxdepth: 2
   :caption: Contents

   design.rst
   blackboard.rst
   perceptions.rst
   commanding.rst

Overview
========

This guide is organized into several chapters, each covering a key subsystem of EasyNav:

- **Design Principles** — Describes the architectural foundations of EasyNav, its modular organization, and execution model.  
- **Blackboard and NavState** — Explains the shared memory model that interconnects all modules.  
- **Perceptions System** — Details how sensory data is represented, processed, and accessed in a unified way.  
- **Commanding Layer** — Describes how the planner, controller, and system nodes cooperate to generate robot motion.

Each chapter provides conceptual explanations, code structure guidelines, and practical examples extracted from the current EasyNav implementation.



Further Reading
===============

For installation and basic usage instructions, refer to the :doc:`../build_install/index` and :doc:`../getting_started/index` sections.  
For stack-specific tutorials and usage examples, see the :doc:`../howtos/index`.
