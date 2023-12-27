#ifndef GTS_GOAL_PLANNER__HXX
#define GTS_GOAL_PLANNER__HXX

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include "gts_goal_planner/gts_converter.hxx"

#define CLASS_NAME "gts_goal_planner"

#endif