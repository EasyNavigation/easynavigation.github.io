.. _perceptions:


Sensor Input and Perception Handling
####################################

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

You typically retrieve the `PointPerceptions` from the `NavState`:

.. code-block:: cpp

   if (!nav_state.has("points")) return;
   const auto perceptions = nav_state.get<PointPerceptions>("points");

Then create an operations view:

.. code-block:: cpp

   PointPerceptionsOpsView view(perceptions);

The view provides a fluent interface to manipulate the point cloud. There are two kinds of operations:

- **Lightweight operations**: these return a reference (`PointPerceptionsOpsView &`) and operate on views without copying data.
- **Heavyweight operations**: these return a `std::shared_ptr<PointPerceptionsOpsView>` and typically involve transformations or filtering that produce new point sets.

Common operations include:

.. code-block:: cpp

   auto downsampled = PointPerceptionsOpsView(perceptions)
     .downsample(0.2);  // reduce density (heavy)

   auto fused = downsampled
     ->fuse("base_link");  // transform all points to base_link (heavy)

   auto filtered = fused
     ->filter({-1.0, -1.0, 0.0}, {1.0, 1.0, 2.0});  // spatial crop (heavy)

   auto points = filtered->as_points();  // retrieve std::vector<Point3D>

Operation Summary
-----------------

+--------------------+----------------------------+------------------------------------------+
| Operation          | Return Type                | Description                              |
+====================+============================+==========================================+
| `filter(...)`      | `std::shared_ptr<...>`     | Filters points inside a bounding box     |
+--------------------+----------------------------+------------------------------------------+
| `downsample(res)`  | `std::shared_ptr<...>`     | Voxel downsampling                       |
+--------------------+----------------------------+------------------------------------------+
| `fuse(frame)`      | `std::shared_ptr<...>`     | Transforms all perceptions to a frame    |
+--------------------+----------------------------+------------------------------------------+
| `collapse()`       | `std::shared_ptr<...>`     | Merge similar points into one            |
+--------------------+----------------------------+------------------------------------------+
| `as_points()`      | `std::vector<Point3D>`     | Exports data as raw 3D point list        |
+--------------------+----------------------------+------------------------------------------+

Example: Updating a Map
-----------------------

Many components use fused and filtered points to update occupancy or elevation maps:

.. code-block:: cpp

   const auto perceptions = nav_state.get<PointPerceptions>("points");

   auto fused = PointPerceptionsOpsView(perceptions)
     .downsample(dynamic_map_.resolution())  // reduce point density
     .fuse("map")                             // transform to map frame
     ->filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})  // ignore ground clutter
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

     auto fused_points = fused->as_points();
     auto msg = points_to_rosmsg(fused_points);

     msg.header.frame_id = perception_default_frame_;
     msg.header.stamp = fused->get_perceptions()[0]->stamp;
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
