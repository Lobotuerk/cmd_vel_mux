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
const std::string CmdVelMux::VACANT = "empty";
CmdVelMux::CmdVelMux(rclcpp::NodeOptions options) : rclcpp::Node("cmd_vel_mux", options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), allowed_(VACANT)
{
  std::map<std::string, rclcpp::Parameter> parameters;
  // Check if there are loaded parameters from config file besides sim_time_used
  if (!get_parameters("", parameters) || parameters.size() <= 1)
  {
    RCLCPP_WARN(get_logger(), "No subscribers configured!");
  }
  else
  {
    configureFromParameters(parseFromParametersMap(parameters));
  }

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

void CmdVelMux::configureFromParameters(const std::map<std::string, ParameterValues> & parameters)
{
  std::map<std::string, ParameterValues>::const_iterator parameter;
  std::map<std::string, std::shared_ptr<CmdVelSub>> new_map;
  //Iterate over all parameters
  for ( parameter = parameters.begin(); parameter != parameters.end(); parameter++ )
  {
    std::string key = parameter->first;
    ParameterValues parameter_values = parameter->second;
    // Check if parameter subscriber has all its necessary values
    if (parameter_values.topic_ == "" || parameter_values.priority_ == -1 || parameter_values.timeout_ == -1) continue;
    if (map_.find(key) != map_.cend())
    {
      // For names already in the subscribers map, retain current object so we don't re-subscribe to the topic
      new_map[key] = map_[key];
    }
    else
    {
      new_map[key] = std::make_shared<CmdVelSub>();
    }

    // update existing or new object with the new configuration

    double new_timeout;
    std::string new_topic;
    new_map[key]->name_ = key;
    new_topic = parameter_values.topic_;
    new_timeout = parameter_values.timeout_;
    new_map[key]->priority_ = parameter_values.priority_;
    new_map[key]->short_desc_ = parameter_values.short_desc_;

    if (new_topic != new_map[key]->topic_)
    {
      // Shutdown the topic if the name has changed so it gets recreated on configuration reload
      // In the case of new subscribers, topic is empty and shutdown has just no effect
      new_map[key]->topic_ = new_topic;
      new_map[key]->sub_ = nullptr;
    }

    if (new_timeout != new_map[key]->timeout_)
    {
      // Change timer period if the timeout changed
      new_map[key]->timeout_ = new_timeout;
      new_map[key]->timer_ = nullptr;
    }
  }

  map_ = new_map;

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  std::map<std::string, std::shared_ptr<CmdVelSub>>::iterator iterator;
  for ( iterator = map_.begin(); iterator != map_.end(); iterator++ )
  {
    std::string key = iterator->first;
    std::shared_ptr<CmdVelSub> values = iterator->second;
    if (!values->sub_)
    {
      map_[key]->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(values->topic_, 10, [this, key](const geometry_msgs::msg::Twist::SharedPtr msg){cmdVelCallback(msg, key);});
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                   values->name_.c_str(), values->topic_.c_str(),
                   values->priority_, values->timeout_);
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "CmdVelMux : no need to re-subscribe to input topic '%s'", values->topic_.c_str());
    }

    if (!values->timer_)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      map_[key]->timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(values->timeout_)), [this, key]() {timerCallback(key);});
    }

    if (values->timeout_ > longest_timeout)
    {
      longest_timeout = values->timeout_;
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

std::map<std::string, ParameterValues> CmdVelMux::parseFromParametersMap(const std::map<std::string, rclcpp::Parameter> & parameters)
{
  std::map<std::string, rclcpp::Parameter>::const_iterator parameter;
  std::map<std::string, ParameterValues> parsed_parameters;
  std::map<std::string, bool> new_priorities;
  used_priorities_ = new_priorities;
  //Iterate over all parameters and parse their content
  for ( parameter = parameters.begin(); parameter != parameters.end(); parameter++ )
  {
      std::string parameter_name = parameter->first;
      rclcpp::Parameter parameter_value = parameter->second;
      if (parameter_name.find(".topic") != std::string::npos && parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        //TODO TYPE WARN MESSAGES
        std::string key = parameter_name.substr(12, parameter_name.find(".topic")-12);
        if (parsed_parameters.find(key) == parsed_parameters.end())
        {
          parsed_parameters[key];
        }
        parsed_parameters[key].topic_ = parameter_value.as_string();
      }
      else if (parameter_name.find(".timeout") != std::string::npos && parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        //TODO TYPE WARN MESSAGES
        std::string key = parameter_name.substr(12, parameter_name.find(".timeout")-12);
        if (parsed_parameters.find(key) == parsed_parameters.end())
        {
          parsed_parameters[key];
        }
        parsed_parameters[key].timeout_ = parameter_value.as_double();
      }
      else if (parameter_name.find(".priority") != std::string::npos && parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        //TODO TYPE WARN MESSAGES
        std::string key = parameter_name.substr(12, parameter_name.find(".priority")-12);

        int priority = parameter_value.as_int();
        if (used_priorities_.find(std::to_string(priority)) == used_priorities_.end())
        {
          used_priorities_[std::to_string(priority)] = true;
          if (parsed_parameters.find(key) == parsed_parameters.end())
          {
            parsed_parameters[key];
          }
          parsed_parameters[key].priority_ = priority;
          continue;
        }
        //TODO PRIORITY ALREADY USED MESSAGE
      }
      else if (parameter_name.find(".short_desc") != std::string::npos && parameter_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        //TODO TYPE WARN MESSAGES
        std::string key = parameter_name.substr(12, parameter_name.find(".short_desc")-12);
        if (parsed_parameters.find(key) == parsed_parameters.end())
        {
          parsed_parameters[key];
        }
        parsed_parameters[key].short_desc_ = parameter_value.as_double();
      }
  }
  return parsed_parameters;
}

void CmdVelMux::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, std::string key)
{
  // if subscriber was deleted or the one being called right now just ignore
  if (map_.find(key) == map_.cend()) return;
  // Reset general timer
  common_timer_->reset();

  // Reset timer for this source
  map_[key]->timer_->reset();

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((allowed_ == VACANT) ||
      (allowed_ == key)    ||
      (map_[key]->priority_ > map_[allowed_]->priority_))
  {
    if (allowed_ != key)
    {
      allowed_ = key;

      // Notify the world that a new cmd_vel source took the control
      auto active_msg = std::make_unique<std_msgs::msg::String>();
      active_msg->data = map_[key]->name_;
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
                map_[allowed_]->name_.c_str());

    // No cmd_vel messages timeout happened to currently active source, so...
    allowed_ = VACANT;

    // ...notify the world that nobody is publishing on cmd_vel; its vacant
    auto active_msg = std::make_unique<std_msgs::msg::String>();
    active_msg->data = "idle";
    active_subscriber_pub_->publish(std::move(active_msg));
  }
}

void CmdVelMux::timerCallback(std::string key)
{
  if (allowed_ == key)
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
  const std::vector<rclcpp::Parameter> & update_parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::map<std::string, rclcpp::Parameter> old_parameters;
  if (!get_parameters("", old_parameters) || old_parameters.size() <= 1)
  {
    result.successful = false;
    result.reason = "no parameters loaded";
  }
  else
  {
    std::map<std::string, ParameterValues> parameters = parseFromParametersMap(old_parameters);
    for (const rclcpp::Parameter & parameter : update_parameters)
    {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        if (parameter.get_name().find(".topic") != std::string::npos)
        {
          std::string key = parameter.get_name().substr(12, parameter.get_name().find(".topic")-12);
          if (parameters.find(key) != parameters.end())
          {
            map_.erase(key);
            parameters.erase(key);
            if (allowed_ == key)
            {
              // Take down the deleted subscriber if it was the one being used as source
              allowed_ = VACANT;

              // ...notify the world that nobody is publishing on cmd_vel; its vacant
              auto active_msg = std::make_unique<std_msgs::msg::String>();
              active_msg->data = "idle";
              active_subscriber_pub_->publish(std::move(active_msg));
            }
          }
          else
          {
            result.successful = false;
            result.reason = "topic was not found to delete";
            break;
          }
        }
        else
        {
          result.successful = false;
          result.reason = "to delete a subscriber, just delete the topic";
          break;
        }
      }
      else if (parameter.get_name().find(".topic") != std::string::npos)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          result.successful = false;
          result.reason = "subscriber's topic must be a string";
          break;
        }
        std::string key = parameter.get_name().substr(12, parameter.get_name().find(".topic")-12);
        if (parameters.find(key) == parameters.end())
        {
          parameters[key];
        }
        parameters[key].topic_ = parameter.as_string();
      }
      else if (parameter.get_name().find(".timeout") != std::string::npos)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.successful = false;
          result.reason = "subscriber's timeout must be a double";
          break;
        }
        std::string key = parameter.get_name().substr(12, parameter.get_name().find(".timeout")-12);
        if (parameters.find(key) == parameters.end())
        {
          parameters[key];
        }
        parameters[key].timeout_ = parameter.as_double();
      }
      else if (parameter.get_name().find(".priority") != std::string::npos)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.successful = false;
          result.reason = "subscriber's priority must be a integer";
          break;
        }
        std::string key = parameter.get_name().substr(12, parameter.get_name().find(".priority")-12);
        if (parameters.find(key) == parameters.end())
        {
          parameters[key];
        }
        unsigned int priority = parameter.as_int();
        if (used_priorities_.find(std::to_string(priority)) != used_priorities_.end())
        {
          result.successful = false;
          result.reason = "subscriber's priority is already in use";
          //TODO PRIORITY ALREADY USED MESSAGE
          continue;
        }
        used_priorities_[std::to_string(priority)] = true;
        parameters[key].priority_ = priority;
      }
      else if (parameter.get_name() == "subscribers.short_desc")
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
        {
          result.successful = false;
          result.reason = "subscriber's short_desc must be a string";
          break;
        }
        std::string key = parameter.get_name().substr(12, parameter.get_name().find(".short_desc")-12);
        if (parameters.find(key) == parameters.end())
        {
          parameters[key];
        }
        parameters[key].short_desc_ = parameter.as_string();
      }
      else
      {
        result.successful = false;
        result.reason = "unknown parameter";
        continue;
      }
    }

    if (result.successful)
    {
      configureFromParameters(parameters);
    }
  }
  return result;
}

} // namespace cmd_vel_mux

RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_mux::CmdVelMux)
