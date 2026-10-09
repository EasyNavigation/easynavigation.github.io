.. _custom_perception_plugin:

==================================
Writing a Custom Perception Plugin
==================================

This HowTo shows how to write your own **PerceptionHandler** plugin for EasyNavigation (EasyNav),
using the real, shipped example package ``easynav_alt_imu_sensor`` as a guide. It also shows how
to actually use that plugin in a working configuration.

See :ref:`perceptions` for the general perception model (sensor configuration, plugin
auto-detection, groups) before working through this guide.

.. contents:: On this page
   :local:
   :depth: 2

When do you need a custom perception plugin?
---------------------------------------------

``SensorsNode`` already ships handlers for six message types (``LaserScan``/``PointCloud2``,
``Image``, ``Imu``, ``NavSatFix``, ``Odometry``, ``Detection3DArray``), auto-selected from the
sensor's ``type:``. You need a custom plugin when:

- your sensor publishes a ROS 2 message type that is **not** in that list, or
- you want **different behavior** for a type that *is* already supported — for example, extra
  logging, custom filtering, a non-standard conversion, or talking to hardware that needs more
  than a plain topic subscription.

The ``easynav_alt_imu_sensor`` package used in this guide is an instance of the second case: it
reuses ``sensor_msgs/msg/Imu`` and ``easynav::IMUPerception``, but adds a diagnostic message on
every received sample.

Setup
-----

Writing and compiling a new plugin requires the EasyNav headers, so this guide builds
everything **from source** — regardless of whether you normally use APT or Pixi for
day-to-day EasyNav use, see :ref:`build_from_source`. Replace ``<distro>`` below with
your target ROS 2 distro (``rolling``, ``jazzy``, ``kilted`` or ``lyrical``):

.. code-block:: bash

   mkdir -p ~/easynav_ws/src && cd ~/easynav_ws/src
   git clone -b <distro> https://github.com/EasyNavigation/EasyNavigation.git
   git clone -b <distro> https://github.com/EasyNavigation/easynav_plugins.git
   git clone https://github.com/EasyNavigation/easynav_alt_imu_sensor.git
   git clone https://github.com/EasyNavigation/easynav_indoor_testcase.git

Then build and source the workspace:

.. code-block:: bash

   cd ~/easynav_ws
   rosdep install --from-paths src --ignore-src -y -r
   colcon build --symlink-install
   source /opt/ros/<distro>/setup.bash
   source install/setup.bash

The example configuration used later in this guide also exercises the **MPC
Controller**, **NavMap Localizer**, **Bonxai Maps Manager**, **NavMap Maps Manager**
and **NavMap Planner** plugins — all already built above since ``easynav_plugins``
was cloned in full. (If you only need those plugins, without developing a new one,
you can instead install them via APT/Pixi — see :doc:`../build_install/index` — but
``easynav_alt_imu_sensor`` itself, being the subject of this tutorial, is only
distributed as source.)

---

The ``PerceptionHandler`` interface
------------------------------------

Every perception plugin derives from ``easynav::PerceptionHandler``
(``easynav_sensors/types/Perceptions.hpp``). The base class only requires two overrides:

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Method
     - Required
     - Description
   * - ``on_initialize()``
     - Optional
     - Called once, right after the handler is loaded. Reserve the perception object here, read
       the sensor's ``topic``/``type`` parameters, and create the ROS 2 subscription.
   * - ``cycle_rt(nav_state)``
     - Yes
     - Called every real-time cycle. Must write the perception into ``NavState`` under
       ``get_sensor_name()`` and return ``true`` if new data arrived since the last call.

Two protected helpers are available once the handler has been initialized:
``get_node()`` (the parent ``SensorsNode`` lifecycle node) and ``get_sensor_name()`` (the sensor's
key, as listed under ``sensors:``).

.. note::
   ``easynav_alt_imu_sensor``'s own bundled ``README.md`` describes an older shape for this
   interface (``group()``/``create()``/``create_subscription()``/``populate_nav_state()``). That
   interface is no longer what the code implements or what ``PerceptionHandler`` declares — treat
   the header (``Perceptions.hpp``) and the ``.cpp`` files below as the source of truth.

Walking through ``AltIMUPerceptionHandler``
---------------------------------------------

**Header** (``include/easynav_alt_imu_sensor/AltIMUPerceptionHandler.hpp``): the class inherits
directly from ``easynav::PerceptionHandler`` (not from the built-in ``IMUPerceptionHandler`` — it
reimplements the subscription itself so it can add its own logic):

.. code-block:: cpp

   class AltIMUPerceptionHandler : public easynav::PerceptionHandler
   {
   public:
     void on_initialize() override;
     bool cycle_rt(std::shared_ptr<easynav::NavState> nav_state) override;

   private:
     std::shared_ptr<easynav::IMUPerception> perception_data_ {nullptr};
     rclcpp::SubscriptionBase::SharedPtr perception_sub_;
   };

**Implementation** (``src/easynav_alt_imu_sensor/AltIMUPerceptionHandler.cpp``):

.. code-block:: cpp

   void AltIMUPerceptionHandler::on_initialize()
   {
     perception_data_ = std::make_shared<easynav::IMUPerception>();

     auto node = get_node();
     std::string topic, msg_type;

     if (!node->has_parameter(get_sensor_name() + ".topic")) {
       node->declare_parameter(get_sensor_name() + ".topic", std::string{});
     }
     if (!node->has_parameter(get_sensor_name() + ".type")) {
       node->declare_parameter(get_sensor_name() + ".type", std::string{});
     }
     node->get_parameter(get_sensor_name() + ".topic", topic);
     node->get_parameter(get_sensor_name() + ".type", msg_type);

     if (msg_type != "sensor_msgs/msg/Imu") {
       throw std::runtime_error("Unsupported message type for AltIMUPerceptionHandler: " + msg_type);
     }

     auto options = rclcpp::SubscriptionOptions();
     options.callback_group = get_realtime_cbg();
     const auto clock_type = node->get_clock()->get_clock_type();

     perception_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
       topic, rclcpp::QoS(1),
       [this, clock_type](const sensor_msgs::msg::Imu::SharedPtr msg)
       {
         std::cerr << "Alternative IMUPerceptionHandler received IMU message" << std::endl;
         perception_data_->stamp = rclcpp::Time(msg->header.stamp, clock_type);
         perception_data_->frame_id = msg->header.frame_id;
         perception_data_->new_data = true;
         perception_data_->data = *msg;
         perception_data_->valid = true;
       },
       options);
   }

   bool AltIMUPerceptionHandler::cycle_rt(std::shared_ptr<easynav::NavState> nav_state)
   {
     nav_state->set(get_sensor_name(), perception_data_);
     const bool should_trigger = perception_data_->new_data;
     perception_data_->new_data = false;
     return should_trigger;
   }

This is exactly the same pattern used by the built-in ``IMUPerceptionHandler``, ``GNSSPerceptionHandler``,
etc. (see :ref:`perceptions`): declare ``topic``/``type`` under the sensor's own parameter
namespace, subscribe with the real-time callback group (``get_realtime_cbg()``), stash the message
in the perception object and mark ``new_data``, then in ``cycle_rt()`` push it into ``NavState``
and reset the flag. The only custom part is the ``std::cerr`` line in the subscription callback.

At the bottom of the ``.cpp`` file, the class is exported as a pluginlib plugin:

.. code-block:: cpp

   #include "pluginlib/class_list_macros.hpp"
   PLUGINLIB_EXPORT_CLASS(easynav_alt_imu::AltIMUPerceptionHandler, easynav::PerceptionHandler)

Registering the plugin
-----------------------

Three pieces wire the class into pluginlib so ``SensorsNode`` can load it by name.

**1. The plugin descriptor** (``easynav_alt_imu_sensor_plugins.xml``):

.. code-block:: xml

   <class_libraries>
     <library path="easynav_alt_imu_sensor">
       <class name="easynav_alt_imu_sensor/AltIMUPerceptionHandler"
              type="easynav_alt_imu::AltIMUPerceptionHandler"
              base_class_type="easynav::PerceptionHandler">
         <description>
           Alternative IMU perception handler. Behaves like the built-in
           IMUPerceptionHandler but prints "imu alternative" to std::cerr
           on every received message.
         </description>
       </class>
     </library>
   </class_libraries>

The ``name`` attribute (``easynav_alt_imu_sensor/AltIMUPerceptionHandler``) is the string you will
use as ``plugin:`` in your sensor configuration.

**2. ``CMakeLists.txt``** exports that descriptor against ``easynav_sensors`` — the package that
owns the ``PerceptionHandler`` base class — so pluginlib can discover it system-wide:

.. code-block:: cmake

   pluginlib_export_plugin_description_file(
     easynav_sensors easynav_alt_imu_sensor_plugins.xml)

**3. ``package.xml``** depends on ``easynav_sensors``, ``pluginlib``, ``rclcpp_lifecycle`` and
``sensor_msgs``.

Build the plugin package:

.. code-block:: bash

   cd ~/easynav_ws
   colcon build --packages-select easynav_alt_imu_sensor
   source install/setup.bash

Using the plugin
-----------------

Set ``plugin:`` explicitly on the sensor that should use your handler — this overrides
``SensorsNode``'s auto-detected default for that message type:

.. code-block:: yaml

   sensors_node:
     ros__parameters:
       sensors: [imu]
       imu:
         topic: imu/data
         type: sensor_msgs/msg/Imu
         plugin: easynav_alt_imu_sensor/AltIMUPerceptionHandler

This is precisely the configuration used by the real, shipped
``easynav_indoor_testcase/robots_params/bonxai.amcl.params.urjc_alt_imu.yaml`` (a NavMap/Bonxai +
MPC controller setup for the URJC excavation world), together with its matching launch file:

.. code-block:: bash

   ros2 launch easynav_indoor_testcase easynav_bonxai_amcl_altimu.launch.py

Once running, every IMU message logs ``Alternative IMUPerceptionHandler received IMU message`` to
the terminal running ``system_main`` — confirming your plugin (and not the built-in
``IMUPerceptionHandler``) is the one processing the sensor. Everything else (frame handling,
``NavState`` storage, downstream consumers) behaves exactly as it would with the built-in handler,
since ``cycle_rt()`` stores the same ``IMUPerception`` type under the same key.

Writing a plugin for a genuinely new message type
----------------------------------------------------

``AltIMUPerceptionHandler`` reuses an existing perception type (``IMUPerception``). To support a
ROS 2 message type that has no built-in equivalent at all, you additionally need a new
``PerceptionBase``-derived class to hold the data (following ``IMUPerception``, ``GNSSPerception``,
etc. as templates — see :ref:`perceptions`). The plugin-registration steps above are otherwise
identical: implement ``on_initialize()``/``cycle_rt()``, export the class with
``PLUGINLIB_EXPORT_CLASS``, describe it in a ``plugins.xml``, and export that file against
``easynav_sensors`` from ``CMakeLists.txt``.

Notes
-----

- ``plugin:`` is matched against the ``name`` attribute in the plugin's XML descriptor, not the
  C++ class name — make sure your configuration uses the former
  (``easynav_alt_imu_sensor/AltIMUPerceptionHandler``, not ``easynav_alt_imu::AltIMUPerceptionHandler``).
- If ``on_initialize()`` throws (as this example does for an unexpected ``type:``), ``SensorsNode``
  fails to configure — check the log for the exact error before assuming the plugin failed to load.
- A custom handler is discovered the same way as any other ``easynav_sensors`` plugin: as long as
  its package is built and sourced, no extra registration step is needed beyond what is described
  above.
