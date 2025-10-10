.. _serest_controller:

=============================
SeReST Controller Fine-Tuning
=============================

This guide explains how to **tune the SeReST (Safe Reparameterized Time) controller**
to track discrete paths (e.g., A* polylines) precisely, slow down smoothly near goals,
and behave safely around dynamic obstacles.

It includes grouped parameters, recommended defaults, and practical
“if you see X, change Y” advice.

.. contents:: On this page
   :local:
   :depth: 2

Overview
--------

The **SeReST controller** is a real-time trajectory follower used in EasyNav.  
It blends path tracking, safety constraints, and curvature-aware motion limits into a single reactive control loop.

You will typically find it configured in:

.. code-block:: yaml

    controller_node:
      ros__parameters:
        controller_types: [serest]
        serest:
          plugin: easynav_serest_controller/SerestController
          rt_freq: 30.0

---

Quick-Start Defaults
--------------------

Start with these default parameters (adjust namespace as needed):

.. code-block:: yaml

    serest_controller:
      ros__parameters:
        # motion policy
        v_ref:                0.6
        max_linear_speed:     0.6
        max_angular_speed:    1.5
        max_linear_acc:       0.8
        max_angular_acc:      2.0
        allow_reverse:        false

        # tracking gains
        k_theta:              2.0
        k_y:                  1.2
        k_s:                  0.8
        ell:                  0.3

        # safety governor
        a_brake:              1.2
        a_lat_max:            1.5
        d0_margin:            0.20
        tau_latency:          0.10
        d_hard:               0.15
        t_emerg:              0.25

        # local heading smoothing
        blend_base:           0.6
        blend_k_per_v:        0.6
        kappa_max:            2.5

        # progress robustness
        v_progress_min:       0.05
        k_s_share_max:        0.5

        # goal behavior
        goal_pos_tol:         0.07
        goal_yaw_tol_deg:     6.0
        slow_radius:          0.60
        slow_min_speed:       0.03
        final_align_k:        2.0
        final_align_wmax:     0.6

        # corner guard (tight turns)
        corner_guard_enable:  true
        a_lat_soft:           1.1
        corner_gain_ey:       1.5
        corner_gain_eth:      0.7
        corner_gain_kappa:    0.4
        corner_min_alpha:     0.35
        corner_boost_omega:   0.8
        apex_ey_des:          0.05

---

Core Concepts
-------------

- **Tracking controller:** keeps the robot close to the reference path using heading and lateral-error feedback  
  (``k_theta``, ``k_y``, ``ell``) plus a small longitudinal “suction” term (``k_s``).

- **Safety governor:** dynamically re-parameterizes time, reducing forward progress when approaching obstacles
  or when curvature demands lower speeds.

- **Local heading smoothing:** blends headings around corners to remove discontinuities while preserving the geometric path.

- **Goal behavior:** decelerates within a configurable radius, aligning to the final yaw precisely.

- **Corner guard:** limits lateral acceleration and boosts yaw authority to prevent drifting wide on tight turns.

---

Parameter-by-Parameter Guidance
-------------------------------

Tracking Gains
^^^^^^^^^^^^^^

- **``k_theta`` (1.5–3.0; default 2.0)**  
  Heading stiffness. ↑ if turns feel sluggish; ↓ if oscillating.

- **``k_y`` (0.8–2.0; default 1.2)**  
  Lateral correction authority. ↑ if robot cuts corners; ↓ if wobbly.

- **``ell`` (0.25–0.5 m; default 0.3)**  
  Lateral lookahead. ↑ to reduce twitchiness; ↓ for faster convergence.

- **``k_s`` (0.4–1.2; default 0.8)**  
  Longitudinal suction. ↑ if drifting along the segment; ↓ if resisting motion.

Motion Limits & Reference
^^^^^^^^^^^^^^^^^^^^^^^^^

- **``v_ref`` (0.4–0.8 m/s)** – Desired cruise speed.  
- **``max_linear_speed`` / ``max_angular_speed``** – Respect hardware limits.  
- **``max_linear_acc`` / ``max_angular_acc``** – ↑ if sluggish; ↓ if slipping.

Progress Robustness
^^^^^^^^^^^^^^^^^^^^

- **``v_progress_min`` (0.03–0.10)** – Minimum forward motion; prevents stalls.  
- **``k_s_share_max`` (0.3–0.7)** – Caps suction influence; ↑ if stalling, ↓ if overshooting laterally.

Safety Governor
^^^^^^^^^^^^^^^

- **``a_brake`` (1.0–2.0)** – Deceleration limit for safe stopping.  
- **``a_lat_max`` (1.2–2.5)** – Max lateral acceleration for curvature limits.  
- **``d0_margin`` (0.15–0.30)** – Safety margin; ↑ for larger robots or noisy sensors.  
- **``tau_latency`` (0.08–0.15 s)** – Compensates sensor/control delay.  
- **``d_hard`` / ``t_emerg``** – Emergency stop thresholds; ↑/↓ to adjust conservatism.

Local Heading Smoothing
^^^^^^^^^^^^^^^^^^^^^^^

- **``blend_base`` (0.4–0.9)** – Base length for corner blending.  
- **``blend_k_per_v`` (0.4–0.9)** – Extra smoothing proportional to velocity.  
- **``kappa_max`` (2.0–3.5)** – Curvature cap for feed-forward yawing.

Goal Behavior
^^^^^^^^^^^^^

- **``slow_radius`` (0.5–1.0 m)** – Begin slowing within this range.  
- **``goal_pos_tol`` (0.05–0.10 m)** – Position tolerance for success.  
- **``goal_yaw_tol_deg`` (4–10°)** – Final yaw tolerance.  
- **``slow_min_speed`` (0.02–0.05)** – Maintains slow progress.  
- **``final_align_k`` / ``final_align_wmax``** – Tune final rotation precision.

Corner Guard
^^^^^^^^^^^^

- **``corner_guard_enable`` (bool)** – Enables corner control logic.  
- **``a_lat_soft`` (0.9–1.4)** – Corner-specific accel cap; ↓ if drifting, ↑ if braking too much.  
- **``corner_gain_ey`` (1.0–2.0)** – Penalizes outside drift.  
- **``corner_gain_eth`` (0.5–1.0)** – Penalizes heading error in turns.  
- **``corner_gain_kappa`` (0.3–0.7)** – Penalizes curvature magnitude.  
- **``corner_min_alpha`` (0.3–0.5)** – Minimum speed scaling in corners.  
- **``corner_boost_omega`` (0.6–1.0)** – Yaw boost when outside curve.  
- **``apex_ey_des`` (0.03–0.08)** – Small inward bias to stay centered at apex.

---

Troubleshooting by Symptom
--------------------------

Robot “moonwalks” (tries to go backward)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Set ``allow_reverse: false`` (default).  
If it still pulls backward, reduce ``k_s`` and increase ``v_progress_min``.

Stalls near corners or from standstill
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Increase ``v_progress_min`` and/or ``k_s_share_max``.  
Increase ``blend_base`` for smoother heading transitions.

Oscillates near goal
^^^^^^^^^^^^^^^^^^^^
Increase ``slow_radius`` or ``goal_pos_tol``.  
Lower ``slow_min_speed``; reduce ``final_align_k`` or ``final_align_wmax`` if overshooting.

Cuts corners or rides outside
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Enable corner guard. Decrease ``a_lat_soft``.  
Increase ``corner_gain_ey`` and/or ``corner_boost_omega``.  
If still wide, raise ``blend_base`` or reduce ``v_ref``.

Overreacts laterally (“nervous”)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Increase ``ell``; reduce ``k_y``.  
If only in corners, lower ``corner_boost_omega``.

Feels sluggish to turn
^^^^^^^^^^^^^^^^^^^^^^
Increase ``k_theta``.  
Verify ``max_angular_speed`` and ``max_angular_acc`` are not too low.  
Reduce ``corner_gain_eth`` or ``corner_gain_kappa`` if over-limiting.

Emergency stops too often
^^^^^^^^^^^^^^^^^^^^^^^^^
Reduce conservatism: increase ``t_emerg``, reduce ``d_hard`` or ``d0_margin``.  
Only do this if sensing latency is well-characterized.

Obstacle distance seems too pessimistic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Lower ``lethal_cost_threshold`` (if available) or reduce ``dist_search_radius``.  
Consider publishing measured ``closest_obstacle_distance`` in NavState.

---

Recommended Tuning Order
------------------------

1. **Set kinematics:** choose safe ``max_*`` and ``v_ref`` values.  
2. **Tune tracking:** adjust ``k_theta``, ``k_y``, ``ell`` for stable path following.  
3. **Verify safety:** confirm emergency stop triggers correctly.  
4. **Optimize corners:** tune ``blend_*``, ``a_lat_soft``, and corner gains.  
5. **Smooth goal arrival:** adjust ``slow_radius`` and ``final_align_*``.  
6. **Ensure progress:** tweak ``v_progress_min`` and ``k_s_share_max`` if stalling occurs.

---

FAQ
---

**Q: Can I enable reverse motion?**  
A: Yes. Set ``allow_reverse: true``. Tune goal and slow behavior carefully to prevent unwanted reversals.

**Q: My path is jagged—what should I do?**  
A: Increase ``blend_base`` and/or ``blend_k_per_v``. Reduce ``kappa_max`` and ``v_ref`` slightly.

**Q: How aggressive should ``a_lat_soft`` be?**  
A: For indoor robots with good traction, 1.0–1.4 m/s² works well.  
On slippery or uneven surfaces, use 0.8–1.0 m/s² for safety.
