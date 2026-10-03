.. _developer_guide:

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
   recovery.rst
   safety.rst

Overview
========

This guide is organized into several chapters, each covering a key subsystem of EasyNav:

- **Design Principles** — Describes the architectural foundations of EasyNav, its modular organization, and execution model, together with the robot geometry, the velocity output (robot limits, mux and smoother), and how EasyNav can be reconfigured at runtime.  
- **Blackboard and NavState** — Explains the shared memory model that interconnects all modules.  
- **Perceptions System** — Details how sensory data is represented, processed, and accessed in a unified way.  
- **Commanding Layer** — Describes how applications send navigation goals to EasyNav and follow their progress.
- **Recovery System** — Explains how EasyNav detects and handles problems: the recovery node, the ``RecoveryManagerBase`` plugin, how a recovery moves the robot, the ``SystemActions`` it can request, and the available recovery systems.
- **Safety** — What EasyNav does for safety: stale and invalid velocity commands, configuration checks, the safety mode and its parameters, memory locking, the configuration fingerprint, and fault injection.

Each chapter provides conceptual explanations, code structure guidelines, and practical examples extracted from the current EasyNav implementation.



Further Reading
===============

For installation and basic usage instructions, refer to the :doc:`../build_install/index` and :doc:`../getting_started/index` sections.  
For stack-specific tutorials and usage examples, see the :doc:`../howtos/index`.
