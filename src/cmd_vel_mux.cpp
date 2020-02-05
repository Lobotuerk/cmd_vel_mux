/**
 * @file /src/cmd_vel_mux_nodelet.cpp
 *
 * @brief  Implementation for the command velocity multiplexer
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmd_vel_mux/cmd_vel_mux.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{
/*****************************************************************************
 ** Implementation
 *****************************************************************************/

CmdVelMux::CmdVelMux(rclcpp::NodeOptions options) : rclcpp::Node("cmd_vel_mux", options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), allowed_(VACANT)
{
  std::vector<std::string> names;
  if (!get_parameter("subscribers.names", names))
  {
    RCLCPP_WARN(get_logger(), "No subscribers configured!");
  }

  configureFromParameters(names);

  param_cb_ =
    add_on_set_parameters_callback(std::bind(&CmdVelMux::parameterUpdate, this,
      std::placeholders::_1));

  output_topic_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribe to output topic 'cmd_vel'");

  active_subscriber_pub_ = this->create_publisher<std_msgs::msg::String>("active", rclcpp::QoS(1).transient_local()); // latched topic

  // Notify the world that right now nobody is publishing on cmd_vel yet
  auto active_msg = std::make_unique<std_msgs::msg::String>();
  active_msg->data = "idle";
  active_subscriber_pub_->publish(std::move(active_msg));

  RCLCPP_DEBUG(get_logger(), "CmdVelMux : successfully initialized");
}

void CmdVelMux::configureFromParameters(const std::vector<std::string> & names)
{
  std::vector<std::shared_ptr<CmdVelSub>> new_list(names.size());
  for (unsigned int i = 0; i < names.size(); i++)
  {
    // Parse entries on YAML
    std::string new_sub_name = names[i];
    auto old_sub = std::find_if(list_.begin(), list_.end(),
                                [&new_sub_name](const std::shared_ptr<CmdVelSub>& sub)
                                                {return sub->name_ == new_sub_name;});

    std::string topic;
    double timeout;
    int64_t priority;
    std::string short_desc;
    if (!get_parameter("subscribers." + names[i] + ".topic", topic))
    {
      RCLCPP_WARN(get_logger(), "Subscriber " + names[i] + " has no topic, not adding");
      continue;
    }
    if (!get_parameter("subscribers." + names[i] + ".timeout", timeout))
    {
      RCLCPP_WARN(get_logger(), "Subscriber " + names[i] + " has no timeout, not adding");
      continue;
    }
    if (!get_parameter("subscribers." + names[i] + ".priority", priority))
    {
      RCLCPP_WARN(get_logger(), "Subscriber " + names[i] + " has no priority, not adding");
      continue;
    }
    if (!get_parameter("subscribers." + names[i] + ".short_desc", short_desc))
    {
      short_desc = "No description";
    }
    if (old_sub != list_.end())
    {
      // For names already in the subscribers list, retain current object so we don't re-subscribe to the topic
      new_list[i] = *old_sub;
    }
    else
    {
      new_list[i] = std::make_shared<CmdVelSub>();
    }
    // update existing or new object with the new configuration

    // Fill attributes with a YAML node content
    double new_timeout;
    std::string new_topic;
    new_list[i]->name_ = names[i];
    new_topic = topic;
    new_timeout = timeout;
    new_list[i]->priority_ = priority;
    new_list[i]->short_desc_ = short_desc;

    if (new_topic != new_list[i]->topic_)
    {
      // Shutdown the topic if the name has changed so it gets recreated on configuration reload
      // In the case of new subscribers, topic is empty and shutdown has just no effect
      new_list[i]->topic_ = new_topic;
      new_list[i]->sub_ = nullptr;
    }

    if (new_timeout != new_list[i]->timeout_)
    {
      // Change timer period if the timeout changed
      new_list[i]->timeout_ = new_timeout;
      new_list[i]->timer_ = nullptr;
    }
  }

  list_ = new_list;

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  for (size_t i = 0; i < list_.size(); i++)
  {
    if (!list_[i]->sub_)
    {
      list_[i]->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(list_[i]->topic_, 10, [this, i](const geometry_msgs::msg::Twist::SharedPtr msg){cmdVelCallback(msg, i);});
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                   list_[i]->name_.c_str(), list_[i]->topic_.c_str(),
                   list_[i]->priority_, list_[i]->timeout_);
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : no need to re-subscribe to input topic '%s'", list_[i]->topic_.c_str());
    }

    if (!list_[i]->timer_)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      list_[i]->timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(list_[i]->timeout_)), [this, i]() {timerCallback(i);});
    }

    if (list_[i]->timeout_ > longest_timeout)
    {
      longest_timeout = list_[i]->timeout_;
    }
  }

  if (!common_timer_ || longest_timeout != (common_timer_period_ / 2.0))
  {
    // Create another timer for cmd_vel messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    common_timer_period_ = longest_timeout * 2.0;
    common_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(common_timer_period_)), std::bind(&CmdVelMux::commonTimerCallback, this));
  }

  RCLCPP_INFO(get_logger(), "CmdVelMux : (re)configured");
}

void CmdVelMux::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, unsigned int idx)
{
  // Reset general timer
  common_timer_->reset();

  // Reset timer for this source
  list_[idx]->timer_->reset();

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((allowed_ == VACANT) ||
      (allowed_ == idx)    ||
      (list_[idx]->priority_ > list_[allowed_]->priority_))
  {
    if (allowed_ != idx)
    {
      allowed_ = idx;

      // Notify the world that a new cmd_vel source took the control
      auto active_msg = std::make_unique<std_msgs::msg::String>();
      active_msg->data = list_[idx]->name_;
      active_subscriber_pub_->publish(std::move(active_msg));
    }

    output_topic_pub_->publish(*msg);
  }
}

void CmdVelMux::commonTimerCallback()
{
  if (allowed_ != VACANT)
  {
    // No cmd_vel messages timeout happened for ANYONE, so last active source got stuck without further
    // messages; not a big problem, just dislodge it; but possibly reflect a problem in the controller
    RCLCPP_WARN(get_logger(), "CmdVelMux : No cmd_vel messages from ANY input received in the last %fs", common_timer_period_);
    RCLCPP_WARN(get_logger(), "CmdVelMux : %s dislodged due to general timeout",
                list_[allowed_]->name_.c_str());

    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

void CmdVelMux::timerCallback(unsigned int idx)
{
  if (allowed_ == idx)
  {
    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

rcl_interfaces::msg::SetParametersResult CmdVelMux::parameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<std::string> names;
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter & parameter : parameters)
  {
    if (parameter.get_name() == "subscribers.names")
    {
      get_parameter("subscribers.names", names);
    }
  }

  if (result.successful)
  {
    configureFromParameters(names);
  }

  return result;
}

} // namespace cmd_vel_mux

RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_mux::CmdVelMux)
