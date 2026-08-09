.. _perceptions:


Sensor Input and Perception Handling
====================================

In EasyNav, **all sensor input flows through a single component**: the ``SensorsNode``. This node is responsible for:

- subscribing to sensor topics defined in the parameter file, loading a ``PerceptionHandler`` plugin
  for each one (auto-detected from the message type, or explicit via ``plugin:``),
- converting sensor data into unified internal formats (point clouds, images, IMU/GNSS/odometry
  readings, 3D detections),
- writing the current data for each sensor into the ``NavState`` under its own key, and, only if
  explicitly configured, registering that key as a member of a named group,
- publishing a fused point-cloud view on ``sensors_node/perceptions`` (only while there are
  subscribers on that topic).

Sensor Configuration
--------------------

Sensors are defined via ROS 2 parameters under the `sensors_node` configuration. For example
(matching the shipped reference file
``easynav_indoor_testcase/robots_params/costmap.serest.params.yaml``):

.. code-block:: yaml

   sensors_node:
     ros__parameters:
       use_sim_time: true
       forget_time: 0.5
       sensors: [laser1]
       laser1:
         topic: scan_raw
         type: sensor_msgs/msg/LaserScan

Each entry under ``sensors:`` only needs a ``topic`` and a ``type``. ``plugin`` is **optional**:
``SensorsNode`` auto-detects the right ``PerceptionHandler`` from the message type using a
built-in table, so you only need ``plugin:`` for a custom/non-standard handler (see
:doc:`../howtos/custom_perception_plugin` for a full worked example). The built-in
mapping is:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - ROS 2 message type
     - Auto-selected handler / perception type
   * - ``sensor_msgs/msg/LaserScan``
     - ``PointPerceptionHandler`` / ``PointPerception``
   * - ``sensor_msgs/msg/PointCloud2``
     - ``PointPerceptionHandler`` / ``PointPerception``
   * - ``sensor_msgs/msg/Image``
     - ``ImagePerceptionHandler`` / ``ImagePerception``
   * - ``sensor_msgs/msg/Imu``
     - ``IMUPerceptionHandler`` / ``IMUPerception``
   * - ``sensor_msgs/msg/NavSatFix``
     - ``GNSSPerceptionHandler`` / ``GNSSPerception``
   * - ``nav_msgs/msg/Odometry``
     - ``OdometryPerceptionHandler`` / ``OdometryPerception``
   * - ``vision_msgs/msg/Detection3DArray``
     - ``DetectionsPerceptionHandler`` / ``DetectionsPerception``

Each perception type declares a conventional ``default_group_`` name (``"points"``, ``"image"``,
``"imu"``, ``"gnss"``, ``"odom"``, ``"detections"`` respectively), but ``SensorsNode`` does
**not** apply it automatically — a sensor is only added to a group if you set ``group:``
explicitly for it.

Grouping sensors: ``group:`` is optional and changes visibility
-----------------------------------------------------------------

By default (no ``group:`` set), a sensor's perception is written to the ``NavState`` under its
own key only. This is what almost every built-in controller, localizer and maps manager expects:
they retrieve point-based perceptions with ``nav_state.get_no_group<PointPerception>()`` (see
below), which returns every perception of that type that does **not** belong to any group.

Setting ``group: <name>`` on a sensor does two things:

- it still writes the perception under its own key, **and**
- it additionally registers that key as a member of the ``<name>`` group.

Because ``get_no_group<T>()`` explicitly excludes any key that belongs to a group, grouping a
sensor **removes** it from that default pool — it becomes visible only to code that explicitly
calls ``nav_state.get_group<T>("<name>")`` for that exact group name. This is used, for example,
by ``easynav_fusion_localizer`` to separate its GNSS sensor(s) into a dedicated ``"gnss"`` group:

.. code-block:: yaml

   sensors_node:
     ros__parameters:
       sensors: [imu, gps, laser1]
       imu:
         topic: /imu/data
         type: sensor_msgs/msg/Imu
       gps:
         topic: /gps/fix
         type: sensor_msgs/msg/NavSatFix
         group: gnss
       laser1:
         topic: /front_laser/points
         type: sensor_msgs/msg/PointCloud2

.. warning::
   Only set ``group:`` on a sensor if the consumer plugin you are using actually looks for that
   named group (via ``get_group<T>("name")``). Most stock EasyNav plugins (AMCL-style localizers,
   the SeReST/MPC/MPPI/RPP/VFF controllers, the Costmap/NavMap obstacle filters) read point
   perceptions with ``get_no_group<PointPerception>()``, so grouping a point sensor under
   ``"points"`` for no reason would make it invisible to all of them.

Processing Point Perceptions
----------------------------

To work with fused or filtered 3D points, EasyNav provides the utility class **`PointPerceptionsOpsView`**.

Almost every built-in plugin retrieves point perceptions with ``get_no_group``, since sensors are
ungrouped by default (see above). It is recommended to use ``const auto &`` to avoid unnecessary
copies:

.. code-block:: cpp

  const auto & perceptions = nav_state.get_no_group<PointPerception>();

``nav_state.get_by_type<PointPerception>()`` retrieves every `PointPerception` stored in the
`NavState`, regardless of grouping (this is what ``SensorsNode`` itself uses to build the fused
visualization topic, see below). ``nav_state.get_group<PointPerception>("points")`` retrieves only
the sensors that were explicitly placed in the ``"points"`` group with ``group: points`` — use it
only if your own configuration actually groups sensors that way.

Then create an operations view and apply chained operations:

.. code-block:: cpp

   auto points = PointPerceptionsOpsView(perceptions)
     .downsample(0.2)                             // reduce density
     .fuse("base_link")                          // transform all points to base_link
     .filter({-1.0, -1.0, 0.0}, {1.0, 1.0, 2.0})  // spatial crop
     .as_points();                                // retrieve pcl::PointCloud<pcl::PointXYZ>

The view provides a fluent interface to manipulate the point cloud. Each operation returns a new
`PointPerceptionsOpsView` (or a lightweight wrapper) so that calls can be chained with `.`.

Operation Summary
-----------------

.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Operation
     - Return Type
     - Description
   * - ``filter(...)``
     - ``PointPerceptionsOpsView &``
     - Filters points inside a bounding box
   * - ``downsample(res)``
     - ``PointPerceptionsOpsView &``
     - Voxel-grid downsampling
   * - ``fuse(frame)``
     - ``PointPerceptionsOpsView &``
     - Transforms all perceptions to a frame
   * - ``collapse(dims)``
     - ``PointPerceptionsOpsView &``
     - Flattens/projects selected dimensions to fixed values (e.g. force z = 0)
   * - ``as_points()``
     - ``pcl::PointCloud<pcl::PointXYZ>``
     - Exports data as a concatenated point cloud

Lazy operations, frames and ``collapse``
----------------------------------------

Some operations in ``PointPerceptionsOpsView`` accept a ``lazy`` flag. This flag controls
**when** the operation is applied and in **which frame** the bounds or collapse values
are interpreted, and it is designed to significantly reduce execution time by delaying
expensive work until the latest possible moment.

``filter(min_bounds, max_bounds, lazy_post_fuse)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Many components use fused and filtered points to update occupancy maps. For example, the
`ObstacleFilter` used by ``easynav_costmap_maps_manager`` does the following:

.. code-block:: cpp

   const auto & perceptions = nav_state.get_no_group<PointPerception>();

   auto view = PointPerceptionsOpsView(perceptions);
   view.downsample(dynamic_map.getResolution())      // reduce point density
     .fuse(tf_info.map_frame, stamp, false)          // transform to map frame
     .filter({NAN, NAN, 0.1}, {NAN, NAN, NAN});      // ignore ground clutter

   const auto & fused = view.as_points();

   for (const auto & p : fused) {
     unsigned int cx, cy;
     if (dynamic_map.worldToMap(p.x, p.y, cx, cy)) {
       dynamic_map.setCost(cx, cy, LETHAL_OBSTACLE);
     }
   }

Fused Visualization
-------------------

If the `SensorsNode` has subscribers on its output topic, it will publish the fused perception result after processing:

.. code-block:: cpp

   const auto & points_perceptions = nav_state->get_by_type<PointPerception>();

   if (percept_pub_->get_subscription_count() > 0 && !points_perceptions.empty()) {
     PointPerceptionsOpsView fused_view(std::move(points_perceptions));

     const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
     const std::string & robot_footprint_frame = tf_info.robot_footprint_frame;

     fused_view.fuse(robot_footprint_frame);
     auto fused_points = fused_view.as_points();
     if (fused_points.empty()) {return;}

     auto msg = points_to_rosmsg(fused_points);
     msg.header.frame_id = robot_footprint_frame;
     const auto & percs = fused_view.get_perceptions();
     msg.header.stamp = (!percs.empty() && percs[0]) ? percs[0]->stamp : now();
     percept_pub_->publish(msg);
   }

The target frame is the robot's footprint frame (``base_footprint`` by default), configured
via the ``robot_footprint_frame`` parameter on the ``system_node`` (see :ref:`design`), not a
per-sensor parameter.

Other Perception Types
----------------------

Beyond ``PointPerception`` and ``ImagePerception``, EasyNav ships four more built-in perception
types, each with its own ``PerceptionHandler`` and message type:

.. list-table::
   :header-rows: 1
   :widths: 25 40 20

   * - Perception class
     - ROS 2 message type
     - Conventional group
   * - ``IMUPerception``
     - ``sensor_msgs/msg/Imu``
     - ``"imu"``
   * - ``GNSSPerception``
     - ``sensor_msgs/msg/NavSatFix``
     - ``"gnss"``
   * - ``OdometryPerception``
     - ``nav_msgs/msg/Odometry``
     - ``"odom"``
   * - ``DetectionsPerception``
     - ``vision_msgs/msg/Detection3DArray``
     - ``"detections"``

All six perception types share the same base:

- inherit from `PerceptionBase`, which provides `stamp`, `frame_id`, `valid`, and `new_data`,
- are ungrouped by default, and only become part of a named group when their sensor entry sets
  ``group:`` (see above),
- are loaded and updated automatically by ``SensorsNode`` once declared under ``sensors:``.

Extending to Other Modalities
------------------------------

To support a ROS 2 message type not covered by the built-in table, implement a new
`PerceptionBase`-derived class and a corresponding `PerceptionHandler` (following the same pattern
as ``PointPerceptionHandler``/``ImagePerceptionHandler``), register it as a pluginlib plugin, and
either reference it explicitly with ``plugin: <your_plugin_name>`` on the sensor entry, or extend
the auto-detection table if you are contributing it back to ``easynav_sensors``.

See :doc:`../howtos/custom_perception_plugin` for a full walkthrough of writing, registering, and
using a custom ``PerceptionHandler`` plugin, based on the real ``easynav_alt_imu_sensor`` example
package.

---

This unified and extensible perception handling design allows plugins to focus on **what** data they need, not **how** it was acquired, filtered, or transformed.
