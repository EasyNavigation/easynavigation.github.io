.. _easynav_gridmap_stack/serest_controller:

SeReST Controller Fine-Tuning
-----------------------------

This guide helps you tune the SeReST (Safe Reparameterized Time) controller to
track discrete paths (e.g., A* polylines) crisply, slow down gracefully near
the goal, and behave safely around dynamic obstacles.

The parameters below are grouped by purpose. Typical starting values and
actionable “if you see X, change Y” advice are included.

.. contents::
   :local:
   :depth: 2

Quick-Start Defaults
^^^^^^^^^^^^^^^^^^^^

Put these into your YAML (adjust namespace to your setup):

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

Core Concepts
^^^^^^^^^^^^^

* **Tracking controller** keeps the robot close to the path using heading and
  lateral error feedback (``k_theta``, ``k_y``, ``ell``) and a small longitudinal
  “suction” (``k_s``).

* **Safety governor** re-parameterizes time (reduces forward progress) according
  to distance to the nearest obstacle and curvature-based limits. It can also
  trigger an emergency stop via TTC / distance thresholds.

* **Local heading smoothing** blends headings around polyline corners to avoid
  discontinuities; it does *not* deform the path, preserving path fidelity.

* **Goal behavior** slows down within a radius and finishes with an in-place
  alignment to the final yaw when position tolerance is met.

* **Corner guard** reduces speed and boosts yaw when you drift to the *outside*
  of a tight curve, preventing wide arcs.

Parameter-by-Parameter Guidance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Tracking Gains
""""""""""""""

- ``k_theta`` (typical: 1.5–3.0; default 2.0)
  *Effect:* heading stiffness.  
  *Increase if:* turns feel sluggish or heading error decays slowly.  
  *Decrease if:* oscillatory heading (wiggle) at speed.

- ``k_y`` (typical: 0.8–2.0; default 1.2)
  *Effect:* lateral correction authority.  
  *Increase if:* robot rides outside of the “lane” or cuts corners.  
  *Decrease if:* lateral oscillations, especially on coarse grids.

- ``ell`` (typical: 0.25–0.5 m; default 0.3)
  *Effect:* lateral nonlinearity scale (like an effective lookahead).  
  *Increase if:* controller is too twitchy laterally.  
  *Decrease if:* lateral convergence is too slow.

- ``k_s`` (typical: 0.4–1.2; default 0.8)
  *Effect:* longitudinal “suction” to centerline in Frenet dynamics.  
  *Increase if:* robot tends to drift along the segment instead of re-centering.  
  *Decrease if:* tendency to fight forward progress from standstill.

Motion Limits & Reference
"""""""""""""""""""""""""

- ``v_ref`` (typical: 0.4–0.8 m/s; default 0.6)  
  Desired cruise speed before safety/curvature scaling.

- ``max_linear_speed``, ``max_angular_speed``  
  Respect the robot’s true capabilities; don’t set these higher than your platform can achieve.

- ``max_linear_acc``, ``max_angular_acc``  
  *Increase if:* robot feels lethargic on starts/stops.  
  *Decrease if:* traction limited, wheel slip, or tipping risk.

Progress Robustness
"""""""""""""""""""

- ``v_progress_min`` (typical: 0.03–0.10 m/s; default 0.05)  
  Minimum forward speed when reasonably aligned; prevents “stalls” at zero.

- ``k_s_share_max`` (typical: 0.3–0.7; default 0.5)  
  Caps how much the lateral suction term can eat into forward progress.  
  *Increase if:* robot still stalls near corners.  
  *Decrease if:* it pushes too hard and overshoots laterally.

Safety Governor
"""""""""""""""

- ``a_brake`` (typical: 1.0–2.0 m/s²; default 1.2)  
  Used in safe-speed-from-distance; set below physical max for robustness.

- ``a_lat_max`` (typical: 1.2–2.5 m/s²; default 1.5)  
  Global lateral accel cap for curvature speed limit (non-corner scenarios).

- ``d0_margin`` (typical: 0.15–0.30 m; default 0.20)  
  Geometric safety buffer; increase on larger robots or noisy sensing.

- ``tau_latency`` (typical: 0.08–0.15 s; default 0.10)  
  Accounts for sensing + control delay; increase if you observe late braking.

- ``d_hard`` (typical: 0.10–0.25 m; default 0.15), ``t_emerg`` (0.2–0.4 s; default 0.25)  
  Emergency stop triggers. Make more conservative (larger ``d_hard``, smaller ``t_emerg``)
  if near-collisions are observed.

Local Heading Smoothing (Corners on Polylines)
""""""""""""""""""""""""""""""""""""""""""""""

- ``blend_base`` (typical: 0.4–0.9 m; default 0.6)  
  Base length over which to blend headings around vertices.

- ``blend_k_per_v`` (typical: 0.4–0.9 s; default 0.6)  
  Additional blend proportional to speed; higher = smoother at high speed.

- ``kappa_max`` (typical: 2.0–3.5 1/m; default 2.5)  
  Caps surrogate curvature used for feed-forward/limits.

Goal Behavior (Smooth Arrival)
""""""""""""""""""""""""""""""

- ``slow_radius`` (typical: 0.5–1.0 m; default 0.6)  
  Start slowing down here. Increase to remove overshoot near goals.

- ``goal_pos_tol`` (typical: 0.05–0.10 m; default 0.07)  
  Consider position achieved within this radius.

- ``goal_yaw_tol_deg`` (typical: 4–10 deg; default 6)  
  Orientation tolerance to declare final arrival.

- ``slow_min_speed`` (typical: 0.02–0.05 m/s; default 0.03)  
  Keeps progress inside slow zone; reduce if you see micro-oscillations.

- ``final_align_k`` (typical: 1.5–3.0; default 2.0), ``final_align_wmax`` (0.4–0.8 rad/s; default 0.6)  
  Controls the in-place final yaw alignment. Increase ``final_align_k`` or
  ``final_align_wmax`` if final rotation is sluggish; decrease if it overshoots.

Corner Guard (Tight Turns)
""""""""""""""""""""""""""

- ``corner_guard_enable`` (default: true)  
  Enables the whole cornering safety scheme.

- ``a_lat_soft`` (typical: 0.9–1.4 m/s²; default 1.1)  
  *Corner-specific* lateral accel cap (more conservative than ``a_lat_max``).
  Reduce if you still drift wide; increase if it brakes too much in curves.

- ``corner_gain_ey`` (typical: 1.0–2.0; default 1.5)  
  Penalizes “being outside” the curve (positive signed lateral error).  
  Increase to cut speed harder when you drift wide.

- ``corner_gain_eth`` (typical: 0.5–1.0; default 0.7)  
  Penalizes large heading error in turns.  
  Increase if you understeer; decrease if it brakes for minor angular noise.

- ``corner_gain_kappa`` (typical: 0.3–0.7; default 0.4)  
  Penalizes high curvature itself. Increase to slow more in very tight corners.

- ``corner_min_alpha`` (typical: 0.30–0.50; default 0.35)  
  Lower bound on speed scaling in corners. Raise to prevent excessive slowdown.

- ``corner_boost_omega`` (typical: 0.6–1.0; default 0.8)  
  Multiplier that boosts yaw authority when outside the curve.  
  Increase if you still open the turn; decrease if over-rotating at apex.

- ``apex_ey_des`` (typical: 0.03–0.08 m; default 0.05)  
  Desired small inward bias at apex to avoid riding the outside.

Troubleshooting by Symptom
^^^^^^^^^^^^^^^^^^^^^^^^^^

Robot “moonwalks” (tries to go backwards)
"""""""""""""""""""""""""""""""""""""""""
*Set ``allow_reverse: false``* unless you intend reverse.  
Ensure forward-only clamps are active (already default) and reduce ``k_s`` if
longitudinal suction is pulling progress negative at standstill.
Increase ``v_progress_min`` to push out of zero-speed stiction.

Stalls near corners or from standstill
""""""""""""""""""""""""""""""""""""""
Increase ``v_progress_min``; increase ``k_s_share_max`` so lateral suction
cannot cancel too much forward progress. Consider a slightly larger ``blend_base``
for smoother heading transitions.

Oscillates near goal
""""""""""""""""""""
Increase ``slow_radius`` and/or ``goal_pos_tol`` slightly. Lower
``slow_min_speed`` (but keep > 0). Reduce ``final_align_wmax`` or
``final_align_k`` if in-place alignment overshoots.

Cuts corners / rides outside in tight turns
"""""""""""""""""""""""""""""""""""""""""""
Enable corner guard (default true). Decrease ``a_lat_soft``. Increase
``corner_gain_ey`` and/or ``corner_boost_omega``. Increase ``apex_ey_des`` a bit.
If still wide, reduce ``v_ref`` or raise ``blend_base`` to smooth heading earlier.

Overreacts laterally (snappy, “nervous” path hugging)
"""""""""""""""""""""""""""""""""""""""""""""""""""""
Increase ``ell``. Reduce ``k_y`` slightly. Reduce ``corner_boost_omega`` if only
in corners. If on very coarse grids, increase ``blend_base`` to avoid rapid
heading changes.

Feels sluggish to turn on straights
"""""""""""""""""""""""""""""""""""
Increase ``k_theta``. Ensure ``max_angular_speed`` and ``max_angular_acc`` are
not too low. If corner guard reduces yaw authority too much, lower
``corner_gain_eth`` and ``corner_gain_kappa``.

Emergency stops too often
"""""""""""""""""""""""""
Reduce conservatism: increase ``t_emerg`` (e.g., 0.35), reduce ``d_hard``,
reduce ``d0_margin``, or increase ``v_ref`` a bit. **Only** do this if your
sensing/latency is trustworthy.

Abstacle distance seems too pessimistic
"""""""""""""""""""""""""""""""""""""""
Lower ``lethal_cost_threshold`` (if exposed), reduce ``dist_search_radius``, or
prefer publishing a measured ``closest_obstacle_distance`` into the NavState.

Tuning Order of Operations
^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Set kinematics**: ``max_*``, ``v_ref`` to platform-safe values.  
2. **Basic tracking**: tune ``k_theta``, ``k_y``, ``ell`` for stable path hugging.  
3. **Safety**: verify emergency stop triggers with test obstacles.  
4. **Corners**: set ``blend_*``, ``a_lat_soft``, and corner guard gains.  
5. **Goal behavior**: adjust ``slow_radius``, ``goal_pos_tol``,
   ``final_align_*`` for a smooth, precise finish.  
6. **Robust progress**: tweak ``v_progress_min``, ``k_s_share_max`` if stalls occur.

FAQ
^^^

**Q: Can I allow reverse?**  
A: Yes, set ``allow_reverse: true``. Use with care; tune goal/slow behaviors to
avoid reverse within the final meters unless desired.

**Q: What if my path is very jagged (low map resolution)?**  
A: Increase ``blend_base`` and/or ``blend_k_per_v``. Consider lowering
``kappa_max`` and reducing ``v_ref`` slightly.

**Q: How aggressive should ``a_lat_soft`` be?**  
A: For indoor AMRs with solid traction, 1.0–1.4 m/s² is typical. On slippery or
uneven surfaces, go lower (0.8–1.0).

