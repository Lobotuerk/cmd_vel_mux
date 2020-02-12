# cmd_vel mux

## About

ROS 2 package for selecting among a number of incoming geometry_msg/msg/Twist messages, choosing the highest
priority one to republish on the output topic.  It will automatically dislodge streams that are lower
priority or that stop publishing for any reason.  The stream currently in use is published on the
"active" topic.

## Published Topics
* `/active` (`std_msgs/msg/String`) - A latched topic.  Publishes the "name" field of the currently active `geometry_msgs/msg/Twist` stream (see Parameters below), or "idle" if nothing is being a published.
* `/cmd_vel` (`geometry_msgs/msg/Twist`) - The current `geometry_msgs/msg/Twist` message.  The values from the current highest-priority `geometry_msgs/msg/Twist` are republished here without change.

## Subscribed Topics
* `/any` (`geometry_msgs/msg/Twist`) - One topic of type `geometry_msgs/msg/Twist` is subscribed to based on the `topic` parameter (see Parameters below).  The data from the highest-priority of these inputs is used as the output `/cmd_vel`.

## Parameters
Subscribers use a dictionary way of being set up, with `name` being the key:
* `name` (string) - The "name" of each of the input `geometry_msgs/msg/Twist` topics.  This is what will be published on the `active` topic when the currently active publisher changes.
* `topic` (string) - The "topic" corresponding to each of the input `geometry_msgs/msg/Twist` topics.  This is what will be subscribed to.  The length of this list must match the `name` list above.
* `timeout` (double) - The "timeout" of each of the input `geometry_msgs/msg/Twist` topics.  If no data is received on the input topic for this amount of time, the input will be automatically disabled.  The length of this list must match the `name` list above.
* `priority` (integer) - The "priority" of each of the input `geometry_msgs/msg/Twist` topics.  The higher the number, the higher the priority.  Higher priority topics will dislodge lower priority topics and start publishing to the output topic automatically.  The length of this list must match the `name` list above.
* `short_desc` (string) - The "short description" of each of the input `geometry_msgs/msg/Twist` topics.  This is informational only.  The length of this list must match the `name` list above.

## Changing values
The dictionary translates to parameters in the form of `subscribers.` + name + `topic/timeout/priority/short_desc`.
It can be changed on the fly using `ros2 param set` command, followed with the parameter in the form mentioned above and the value.

## Adding subscriber
To add a new subscriber, call the service `/cmd_vel_mux/set_parameters_atomically` with a `rcl_interfaces/srv/SetParametersAtomically` message with a parameters value equal to an array of 4 `rcl_interfaces/msg/Parameter` messages, containing the new subscriber topic,timeout,priority and short_desc. All 4 need to be present for a new subscriber to be added.

## Deleting subscriber
To delete a subscriber, call the service `/cmd_vel_mux/set_parameters_atomically` with a `rcl_interfaces/srv/SetParametersAtomically` message with a parameters value equal to an array of 4 `rcl_interfaces/msg/Parameter` messages, containing the new subscriber topic,timeout,priority and short_desc with the `type` key inside the `rcl/parameter/ParameterValue` being set to 0 (PARAMETER_NOT_SET)/
