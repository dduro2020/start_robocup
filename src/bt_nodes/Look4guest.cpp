// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <math.h>
#include <cmath>

#include "bt_nodes/Look4guest.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>



namespace start_robocup
{
using namespace std::chrono_literals;

Look4guest::Look4guest(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  
}

BT::NodeStatus
Look4guest::tick()
{
  geometry_msgs::msg::TransformStamped odom2person_msg;
  tf2::Stamped<tf2::Transform> odom2person;

  // Search for the person tf
  try {
    odom2person_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(odom2person_msg, odom2person);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  
  // Calculate the distance to the person using pitagoras theorem
  double distance = sqrt(odom2person.getOrigin().x()*odom2person.getOrigin().x() + odom2person.getOrigin().y()*odom2person.getOrigin().y());
  
  if (distance <= 1.5) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}


}  // namespace start_robocup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<start_robocup::Look4guest>("Look4guest");
}