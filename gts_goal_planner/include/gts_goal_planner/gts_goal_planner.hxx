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
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <gts_msgs/action/goal_planner.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "gts_goal_planner/gts_converter.hxx"

#define RCL_NODE_NAME "gts_goal_planner"
#define CLASS_NAME "gts_goal_planner"

#define GTS_GOAL_PLANNER_SERVER_NAME "/gts_goal_planner/planner"
#define NTP_CLIENT_NAME "/navigate_to_pose"

#define DEFAULT_INT 0
#define DEFAULT_DOUBLE 0.0

using std::placeholders::_1;
using std::placeholders::_2;

namespace gts_goal_planner
{
    namespace planner
    {
        class GoalPlanner : public rclcpp::Node
        {
        private:
            int slam_waypoints_list_index_;
            std::vector<sensor_msgs::msg::NavSatFix> slam_waypoints_list_;
            size_t slam_waypoints_list_size_;

            rclcpp::Node::SharedPtr node_;
            rclcpp::CallbackGroup::SharedPtr gts_goal_planner_server_cb_group_;
            rclcpp_action::Server<gts_msgs::action::GoalPlanner>::SharedPtr gts_goal_planner_server_;

            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr ntp_goal_handle_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal::SharedPtr ntp_goal_;
            rclcpp::CallbackGroup::SharedPtr ntp_client_cb_group_;
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr ntp_client_;

            std_msgs::msg::Header build_header(const char *frame_id);
            rclcpp_action::GoalResponse gts_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const gts_msgs::action::GoalPlanner::Goal> goal);
            rclcpp_action::CancelResponse gts_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void gts_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void gts_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle);
            void ntp_send_goal();
            void ntp_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
            void ntp_goal_response_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle);
            void ntp_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result);
            void position_test();
        public:
            explicit GoalPlanner(const rclcpp::NodeOptions &node_options);
            virtual ~GoalPlanner();
            void signal_handler(int signal_input);
        };
    }
}

#endif