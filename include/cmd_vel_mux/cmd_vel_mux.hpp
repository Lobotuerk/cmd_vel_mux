/**
 * @file /include/yocs_cmd_vel_mux/cmd_vel_mux_nodelet.hpp
 *
 * @brief Structure for the yocs_cmd_vel_mux.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_cmd_vel_mux/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef CMD_VEL_MUX__CMD_VEL_MUX_HPP_
#define CMD_VEL_MUX__CMD_VEL_MUX_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cmd_vel_mux
{

/*****************************************************************************
 ** CmdVelMux
 *****************************************************************************/

class CmdVelMux final : public rclcpp::Node
{
public:
  explicit CmdVelMux(rclcpp::NodeOptions options);
  ~CmdVelMux() override = default;
  CmdVelMux(CmdVelMux && c) = delete;
  CmdVelMux & operator=(CmdVelMux && c) = delete;
  CmdVelMux(const CmdVelMux & c) = delete;
  CmdVelMux & operator=(const CmdVelMux & c) = delete;

private:
  static const unsigned int VACANT       = 666666;  /**< ID for "nobody" active input; anything big is ok */

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_topic_pub_;   /**< Multiplexed command velocity topic */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_subscriber_pub_;  /**< Currently allowed cmd_vel subscriber */
  rclcpp::TimerBase::SharedPtr common_timer_;           /**< No messages from any subscriber timeout */
  double common_timer_period_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  unsigned int allowed_;

  void commonTimerCallback();
  void timerCallback(std::string name);
  void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg, std::string name);

<<<<<<< HEAD
<<<<<<< HEAD
  void configureFromParameters(const std::vector<std::string> & names);
=======
  void configureFromParameters(const std::vector<std::string> & names, const std::map<std::string, std::string> & topics, const std::map<std::string, double> & timeouts, const std::map<std::string, int64_t> & priorities, const std::map<std::string, std::string> & short_descs);
>>>>>>> old_state
=======
  void configureFromParameters(const std::vector<std::string> & names, const std::map<std::string, std::string> & topics, const std::map<std::string, double> & timeouts, const std::map<std::string, int64_t> & priorities, const std::map<std::string, std::string> & short_descs);
>>>>>>> e304fce8c8594224a366faed01a9263e5191c1e2
  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  /*********************
   ** Private Classes
   **********************/

  /**
   * Inner class describing an individual subscriber to a cmd_vel topic
   */
  struct CmdVelSub final
  {
    std::string            name_;         /**< Descriptive name; must be unique to this subscriber */
    std::string            topic_;        /**< The name of the topic */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        sub_;         /**< The subscriber itself */
    rclcpp::TimerBase::SharedPtr             timer_;        /**< No incoming messages timeout */
    double                 timeout_;      /**< Timer's timeout, in seconds  */
    unsigned int           priority_;     /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string            short_desc_;   /**< Short description (optional) */
  };

  std::vector<std::shared_ptr<CmdVelSub>> list_;
  std::vector<std::string> parameters_names;
};

} // namespace cmd_vel_mux

#endif /* CMD_VEL_MUX__CMD_VEL_MUX_HPP_ */
