.. _perceptions:


Sensor Input and Perception Handling
====================================

In EasyNav, **all sensor input flows through a single component**: the ``SensorsNode``. This node is responsible for:

- subscribing to sensor topics defined in the parameter file,
- converting sensor data into unified internal formats (e.g., point clouds, images),
- organizing and storing perceptions into groups (e.g., `"points"`, `"image"`),
- publishing a fused view (optional),
- writing the current valid perceptions into the `NavState` under their corresponding group key.

Sensor Configuration
--------------------

Sensors are defined via ROS 2 parameters under the `sensors_node` configuration. For example:

.. code-block:: yaml

   sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser1, camera1]
       perception_default_frame: odom
       laser1:
         topic: /scan_raw
         type: sensor_msgs/msg/LaserScan
         group: points
       camera1:
         topic: /rgbd_camera/points
         type: sensor_msgs/msg/PointCloud2
         group: points

All distance sensors (e.g., `LaserScan`, `PointCloud2`) must be grouped under the `"points"` group. These are converted internally into **`PointPerception`** instances and aggregated into a single structure called **`PointPerceptions`**, which is written into the `NavState` under the key `"points"`.

If image data is included:

.. code-block:: yaml

   image1:
     topic: /rgbd_camera/image_raw
     type: sensor_msgs/msg/Image
     group: image

Then the group `"image"` will appear in the `NavState`, using the `ImagePerception` type.

Processing Point Perceptions
----------------------------

To work with fused or filtered 3D points, EasyNav provides the utility class **`PointPerceptionsOpsView`**.

You typically retrieve the `PointPerceptions` from the `NavState`. It is
recommended to use ``const auto &`` to avoid unnecessary copies:

.. code-block:: cpp

  if (!nav_state.has("points")) return;
  const auto & perceptions = nav_state.get<PointPerceptions>("points");

Then create an operations view and apply chained operations:

.. code-block:: cpp

   auto points = PointPerceptionsOpsView(perceptions)
     .downsample(0.2)                             // reduce density
     .fuse("base_link")                          // transform all points to base_link
     .filter({-1.0, -1.0, 0.0}, {1.0, 1.0, 2.0})  // spatial crop
     .as_points();                                // retrieve std::vector<Point3D>

The view provides a fluent interface to manipulate the point cloud. Each operation returns a new
`PointPerceptionsOpsView` (or a lightweight wrapper) so that calls can be chained with `.`.

Operation Summary
-----------------

+--------------------+----------------------------+------------------------------------------+
| Operation          | Return Type                | Description                              |
+====================+============================+==========================================+
| `filter(...)`      | `PointPerceptionsOpsView`  | Filters points inside a bounding box     |
+--------------------+----------------------------+------------------------------------------+
| `downsample(res)`  | `PointPerceptionsOpsView`  | Voxel downsampling                       |
+--------------------+----------------------------+------------------------------------------+
| `fuse(frame)`      | `PointPerceptionsOpsView`  | Transforms all perceptions to a frame    |
+--------------------+----------------------------+------------------------------------------+
| `collapse()`       | `PointPerceptionsOpsView`  | Merge similar points into one            |
+--------------------+----------------------------+------------------------------------------+
| `as_points()`      | `std::vector<Point3D>`     | Exports data as raw 3D point list        |
+--------------------+----------------------------+------------------------------------------+

Lazy operations, frames and ``collapse``
----------------------------------------

Some operations in ``PointPerceptionsOpsView`` accept a ``lazy`` flag. This flag controls
**when** the operation is applied and in **which frame** the bounds or collapse values
are interpreted, and it is designed to significantly reduce execution time by delaying
expensive work until the latest possible moment.

``filter(min_bounds, max_bounds, lazy_post_fuse)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Signature (simplified):

.. code-block:: cpp

   PointPerceptionsOpsView &
   filter(const std::vector<double> & min_bounds,
          const std::vector<double> & max_bounds,
          bool lazy_post_fuse = true);

Behavior
~~~~~~~~

- If you have called ``fuse("frame")`` before and ``lazy_post_fuse == true``:

  - The filter is **not** applied immediately.
  - The bounds are stored internally and applied **later**, inside ``as_points()``.
  - The comparison is done **after** transforming points to the fused frame.
  - The bounds are therefore interpreted in the **target frame** of ``fuse()``
    (e.g. ``"map"``, ``"base_link"``).

- In all other cases (no ``fuse()`` or ``lazy_post_fuse == false``):

  - The filter is applied **immediately**, in the **current frame** of each perception.
  - The indices of the kept points are updated at once.

Recommended usage
~~~~~~~~~~~~~~~~~

- Use ``lazy_post_fuse = true`` when you fuse to a frame and define your bounding box
  in that frame:

  .. code-block:: cpp

     auto points = PointPerceptionsOpsView(perceptions)
       .fuse("map")
       .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN}, /*lazy_post_fuse=*/true)
       .as_points();  // filter is applied in "map" frame

- Use ``lazy_post_fuse = false`` when you do **not** fuse, or when you want to reduce
  data early in the sensor frame:

  .. code-block:: cpp

     auto points = PointPerceptionsOpsView(perceptions)
       .filter({0.0, NAN, NAN}, {5.0, NAN, NAN}, /*lazy_post_fuse=*/false)
       .downsample(0.3)
       .as_points();  // filter is applied in the sensor frame

Pitfalls
~~~~~~~~

- Avoid chaining multiple ``fuse()`` calls with lazy filters between them:

  .. code-block:: cpp

     auto view = PointPerceptionsOpsView(perceptions)
       .fuse("map")
       .filter({-5, -5, 0.0}, {5, 5, 2.0}, true)   // intended in "map"
       .fuse("base_link")
       .filter({-1, -1, 0.0}, {1, 1, 1.0}, true);  // intended in "base_link"?

  All lazy filters are finally evaluated in the **last fused frame** (here
  ``"base_link"``), which probably does **not** match the original intention for
  the first filter. In these cases, either use ``lazy_post_fuse = false`` where
  you really want an immediate filter, or keep a single fused frame for all
  lazy filters.

``collapse(collapse_dims, lazy)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Signature (simplified):

.. code-block:: cpp

   PointPerceptionsOpsView &
   collapse(const std::vector<double> & collapse_dims,
            bool lazy = true);

Behavior
~~~~~~~~

- ``lazy == true``:

  - The underlying point clouds are **not modified**.
  - Collapse values (x, y, z) are stored and applied only when exporting points
    (``as_points()`` or ``as_points(int)``).
  - Works for both owning and non-owning views.

- ``lazy == false``:

  - Requires an **owning** view (created from a ``PointPerceptions &&`` or a
    single ``PointPerception``).
  - Directly overwrites the stored point clouds (x, y, z set to the collapse
    values).
  - If the view is non-owning, the operation is ignored and a warning is logged.

Recommended usage
~~~~~~~~~~~~~~~~~

- Prefer ``lazy = true`` in most plugins:

  .. code-block:: cpp

     auto ground_2d = PointPerceptionsOpsView(perceptions)
       .collapse({NAN, NAN, 0.0}, /*lazy=*/true)  // force z = 0.0 on export
       .as_points();

  This keeps the original 3D data intact and only flattens it in the exported
  cloud.

- Use ``lazy = false`` only when you **own** the underlying container and you
  intentionally want to rewrite it for later reuse:

  .. code-block:: cpp

     PointPerceptions cloud = get_point_perceptions(raw_perceptions);
     auto view = PointPerceptionsOpsView(std::move(cloud));

     view.collapse({NAN, NAN, 0.0}, /*lazy=*/false);  // permanently flatten z
     auto points = view.as_points();                  // data was already modified

Pitfalls
~~~~~~~~

- Calling ``collapse(..., false)`` on a non-owning view does **nothing useful**:

  .. code-block:: cpp

     PointPerceptionsOpsView view(perceptions);          // non-owning
     view.collapse({NAN, NAN, 0.0}, /*lazy=*/false);     // ignored, warning logged

  In this case, you should either keep ``lazy = true`` (safe and effective on
  export), or create an owning view if you really need to modify the original
  data.

Example: Updating a Map
-----------------------

Many components use fused and filtered points to update occupancy or elevation maps:

.. code-block:: cpp

  const auto & perceptions = nav_state.get<PointPerceptions>("points");

   auto fused = PointPerceptionsOpsView(perceptions)
     .downsample(dynamic_map_.resolution())       // reduce point density
     .fuse("map")                                // transform to map frame
     .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})    // ignore ground clutter
     .as_points();

   for (const auto & p : fused) {
     if (dynamic_map_.check_bounds_metric(p.x, p.y)) {
       auto [cx, cy] = dynamic_map_.metric_to_cell(p.x, p.y);
       dynamic_map_.at(cx, cy) = 1;
     }
   }

Fused Visualization
-------------------

If the `SensorsNode` has subscribers on its output topic, it will publish the fused perception result after processing:

.. code-block:: cpp

   if (percept_pub_->get_subscription_count() > 0) {
     auto fused = PointPerceptionsOpsView(perceptions)
       .fuse(perception_default_frame_);

     auto fused_points = fused.as_points();
     auto msg = points_to_rosmsg(fused_points);

     msg.header.frame_id = perception_default_frame_;
     msg.header.stamp = fused.get_perceptions()[0]->stamp;
     percept_pub_->publish(msg);
   }

Extending to Other Modalities
-----------------------------

In addition to `"points"` and `"image"`, developers can add new groups and corresponding `PerceptionBase`-derived classes. All perceptions:

- inherit from `PerceptionBase`,
- have a `stamp`, `frame_id`, and `valid` flag,
- are grouped by semantic label (e.g., `"points"`),
- are automatically managed by the `SensorsNode`.

---

This unified and extensible perception handling design allows plugins to focus on **what** data they need, not **how** it was acquired, filtered, or transformed.
