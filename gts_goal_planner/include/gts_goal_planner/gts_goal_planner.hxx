#ifndef GTS_GOAL_PLANNER__HXX
#define GTS_GOAL_PLANNER__HXX

#include <functional>
#include <memory>
#include <thread>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <gts_msgs/action/goal_planner.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "gts_goal_planner/gts_converter.hxx"

#define RCL_NODE_NAME "gts_goal_planner"
#define CLASS_NAME "gts_goal_planner"

#define ACTION_SERVER_NAME "gts_goal_planner/planner"

using std::placeholders::_1;
using std::placeholders::_2;

namespace gts_goal_planner
{
    namespace planner
    {
        class GoalPlanner : public rclcpp::Node
        {
        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::CallbackGroup::SharedPtr gts_goal_planner_server_cb_group_;
            rclcpp_action::Server<gts_msgs::action::GoalPlanner>::SharedPtr gts_goal_planner_server_;

            rclcpp::CallbackGroup::SharedPtr ntp_client_cb_group_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr ntp_client_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const gts_msgs::action::GoalPlanner::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void position_test();
        public:
            explicit GoalPlanner(const rclcpp::NodeOptions &node_options);
            virtual ~GoalPlanner();
            void signal_handler(int signal_input);
        };
    }
}

#endif