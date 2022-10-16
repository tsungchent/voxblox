#include "voxblox_ros/transformer.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
// #include <minkindr_conversions/kindr_xml.h>

namespace voxblox {

Transformer::Transformer(const rclcpp::Node::SharedPtr& nh)
    : nh_(nh),
      world_frame_("world"),
      sensor_frame_(""),
      use_tf_transforms_(true),
      timestamp_tolerance_ns_(1000000),
      tf_buffer_(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME),
      tf_listener_(tf_buffer_) {
  //nh_private_.param("world_frame", world_frame_, world_frame_);
  //nh_private_.param("sensor_frame", sensor_frame_, sensor_frame_);
  nh_->set_parameter(rclcpp::Parameter("world_frame", world_frame_));
  nh_->set_parameter(rclcpp::Parameter("sensor_frame", sensor_frame_));

  const double kNanoSecondsInSecond = 1.0e9;
  double timestamp_tolerance_sec =
      timestamp_tolerance_ns_ / kNanoSecondsInSecond;
  // nh_private_.param("timestamp_tolerance_sec", timestamp_tolerance_sec,
  //                   timestamp_tolerance_sec);
  nh_->set_parameter(
      rclcpp::Parameter("timestamp_tolerance_sec", timestamp_tolerance_sec));

  timestamp_tolerance_ns_ =
      static_cast<int64_t>(timestamp_tolerance_sec * kNanoSecondsInSecond);

  // Transform settings.
  // nh_private_.param("use_tf_transforms", use_tf_transforms_,
  //                   use_tf_transforms_);
  nh_->set_parameter(
      rclcpp::Parameter("use_tf_transforms", use_tf_transforms_));

  // If we use topic transforms, we have 2 parts: a dynamic transform from a
  // topic and a static transform from parameters.
  // Static transform should be T_G_D (where D is whatever sensor the
  // dynamic coordinate frame is in) and the static should be T_D_C (where
  // C is the sensor frame that produces the depth data). It is possible to
  // specify T_C_D and set invert_static_tranform to true.
  #if 0  
  // TODO
  if (!use_tf_transforms_) {
    transform_sub_ =
        nh_->create_subscription<geometry_msgs::msg::TransformStamped>(
            "transform", 40, std::bind(&Transformer::transformCallback, this));
    // Retrieve T_D_C from params.
    XmlRpc::XmlRpcValue T_B_D_xml;
    // TODO(helenol): split out into a function to avoid duplication.
    if (nh_private_.getParam("T_B_D", T_B_D_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_D_xml, &T_B_D_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_D", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_D_ = T_B_D_.inverse();
      }
    }
    XmlRpc::XmlRpcValue T_B_C_xml;
    if (nh_private_.getParam("T_B_C", T_B_C_xml)) {
      kindr::minimal::xmlRpcToKindr(T_B_C_xml, &T_B_C_);

      // See if we need to invert it.
      bool invert_static_tranform = false;
      nh_private_.param("invert_T_B_C", invert_static_tranform,
                        invert_static_tranform);
      if (invert_static_tranform) {
        T_B_C_ = T_B_C_.inverse();
      }
    }
  }
  #endif
}

void Transformer::transformCallback(
    const geometry_msgs::msg::TransformStamped& transform_msg) {
  transform_queue_.push_back(transform_msg);
}

bool Transformer::lookupTransform(const std::string& from_frame,
                                  const std::string& to_frame,
                                  const rclcpp::Time& timestamp,
                                  Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  } else {
    return lookupTransformQueue(timestamp, transform);
  }
}

// Stolen from octomap_manager
bool Transformer::lookupTransformTf(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const rclcpp::Time& timestamp,
                                    Transformation* transform) {
  CHECK_NOTNULL(transform);
  tf2::Transform tf_transform;
  rclcpp::Time time_to_lookup = timestamp;

  // Allow overwriting the TF frame for the sensor.
  std::string from_frame_modified = from_frame;
  if (!sensor_frame_.empty()) {
    from_frame_modified = sensor_frame_;
  }

  tf2::TimePoint time_to_lookup_tf =
      std::chrono::time_point<std::chrono::system_clock,
                              std::chrono::nanoseconds>(
          std::chrono::nanoseconds(timestamp.nanoseconds()));
  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_buffer_.canTransform(to_frame, from_frame_modified,
                               time_to_lookup_tf)) {
    return false;
  }

  try {
    tf_buffer_.lookupTransform(to_frame, from_frame_modified, time_to_lookup_tf);
  } catch (tf2::TransformException& ex) {  // NOLINT
    // ROS_ERROR_STREAM(
    //     "Error getting TF transform from sensor data: " << ex.what());
    RCLCPP_ERROR(rclcpp::get_logger("Transformer"),
                 "Error getting TF transform from sensor data: %s", ex.what());
    return false;
  }

  tf2::transformTFToKindr(tf_transform, transform);
  return true;
}

bool Transformer::lookupTransformQueue(const rclcpp::Time& timestamp,
                                       Transformation* transform) {
  CHECK_NOTNULL(transform);
  if (transform_queue_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("Transformer"),
                "%d No match found for transform timestamp: %f as transform "
                "queue is empty.",
                30, timestamp.seconds());
    return false;
  }
  // Try to match the transforms in the queue.
  bool match_found = false;
  std::deque<geometry_msgs::msg::TransformStamped>::iterator it =
      transform_queue_.begin();
  for (; it != transform_queue_.end(); ++it) {
    // If the current transform is newer than the requested timestamp, we need
    // to break.
    if (rclcpp::Time(it->header.stamp) > timestamp) {
      if ((rclcpp::Time(it->header.stamp) - timestamp).nanoseconds() <
          timestamp_tolerance_ns_) {
        match_found = true;
      }
      break;
    }

    if ((timestamp - rclcpp::Time(it->header.stamp)).nanoseconds() <
        timestamp_tolerance_ns_) {
      match_found = true;
      break;
    }
  }

  // Match found basically means an exact match.
  Transformation T_G_D;
  if (match_found) {
    tf2::transformMsgToKindr(it->transform, &T_G_D);
  } else {
    // If we think we have an inexact match, have to check that we're still
    // within bounds and interpolate.
    if (it == transform_queue_.begin() || it == transform_queue_.end()) {
      RCLCPP_WARN(rclcpp::get_logger("Transformer"),
                  "%d, No match found for transform timestamp:  %f,  Queue front: %f,  back:  %f ",
                  30,
                  timestamp.seconds(),
                  rclcpp::Time(transform_queue_.front().header.stamp).seconds(),
                  rclcpp::Time(transform_queue_.back().header.stamp).seconds()
      );
      return false;
    }
    // Newest should be 1 past the requested timestamp, oldest should be one
    // before the requested timestamp.
    Transformation T_G_D_newest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_newest);
    int64_t offset_newest_ns =
        (rclcpp::Time(it->header.stamp) - timestamp).nanoseconds();
    // We already checked that this is not the beginning.
    it--;
    Transformation T_G_D_oldest;
    tf2::transformMsgToKindr(it->transform, &T_G_D_oldest);
    int64_t offset_oldest_ns =
        (timestamp - rclcpp::Time(it->header.stamp)).nanoseconds();

    // Interpolate between the two transformations using the exponential map.
    FloatingPoint t_diff_ratio =
        static_cast<FloatingPoint>(offset_oldest_ns) /
        static_cast<FloatingPoint>(offset_newest_ns + offset_oldest_ns);

    Transformation::Vector6 diff_vector =
        (T_G_D_oldest.inverse() * T_G_D_newest).log();
    T_G_D = T_G_D_oldest * Transformation::exp(t_diff_ratio * diff_vector);
  }

  // If we have a static transform, apply it too.
  // Transform should actually be T_G_C. So need to take it through the full
  // chain.
  *transform = T_G_D * T_B_D_.inverse() * T_B_C_;

  // And also clear the queue up to this point. This leaves the current
  // message in place.
  transform_queue_.erase(transform_queue_.begin(), it);
  return true;
}

}  // namespace voxblox
