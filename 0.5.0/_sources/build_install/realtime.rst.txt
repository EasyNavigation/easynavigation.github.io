.. _realtime_setup:

======================
Real-time System Setup
======================

EasyNav runs its real-time cycle with ``SCHED_FIFO`` priority **80** (``system_node.use_real_time``,
``true`` by default). Linux only lets a process use that priority if its ``RLIMIT_RTPRIO`` is at
least 80; otherwise EasyNav warns and runs with normal priority, or, in safety mode, does not start
(see :ref:`safety_mode`). If you enable ``system_node.safety.lock_memory``, ``RLIMIT_MEMLOCK`` must
also be unlimited, or EasyNav does not start (see :ref:`safety_memory`).

Check the current limits of your shell:

.. code-block:: bash

   ulimit -r   # RLIMIT_RTPRIO: must be >= 80
   ulimit -l   # RLIMIT_MEMLOCK: must be "unlimited", only for safety.lock_memory

Set them depending on how EasyNav is started.

**From a user session** (terminal, SSH, ``ros2 launch``): create a ``realtime`` group, add your user
to it, and give the group the limits in ``/etc/security/limits.d/``:

.. code-block:: bash

   sudo groupadd -f realtime
   sudo usermod -aG realtime $USER
   sudo tee /etc/security/limits.d/99-easynav-realtime.conf > /dev/null << 'EOF'
   @realtime   -   rtprio    98
   @realtime   -   memlock   unlimited
   EOF

Log out and in again (or reboot) and check ``ulimit -r`` and ``ulimit -l``. These limits apply to
login sessions only, not to systemd services.

**As a systemd service**: set the limits in the ``[Service]`` section of the unit:

.. code-block:: ini

   [Service]
   LimitRTPRIO=98
   LimitMEMLOCK=infinity

**In Docker**:

.. code-block:: bash

   docker run --ulimit rtprio=98 --ulimit memlock=-1:-1 ...

or, with Docker Compose:

.. code-block:: yaml

   services:
     easynav:
       ulimits:
         rtprio: 98
         memlock: -1

If the host kernel uses real-time group scheduling (``CONFIG_RT_GROUP_SCHED``, not enabled in the
standard Ubuntu kernels), the container also needs real-time CPU time (``--cpu-rt-runtime``).

Leave out the ``memlock`` lines if you do not use ``safety.lock_memory``: locking memory has a cost
(see :ref:`safety_memory`).

**Check it worked**: EasyNav logs ``Selected Real-Time`` without a following
``Failed to set Real Time`` warning, and ``[safety.lock_memory] Memory locked`` if enabled. The
scheduling of its threads can be seen with:

.. code-block:: bash

   ps -eLo pid,tid,cls,rtprio,comm | grep system_main

The real-time cycle's thread, and the TF listener thread it starts, show ``FF 80`` (``SCHED_FIFO``,
priority 80); the rest, ``TS`` (normal scheduling).

For lower and more predictable latencies, use a kernel with ``PREEMPT_RT``.
