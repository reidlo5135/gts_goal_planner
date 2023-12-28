#include "gts_goal_planner/gts_goal_planner.hxx"

gts_goal_planner::planner::GoalPlanner::GoalPlanner(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
    : Node(RCL_NODE_NAME, node_options)
{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

    if (this->node_ != nullptr)
    {
        RCLCPP_INFO(this->node_->get_logger(), "[%s] node has been created", RCL_NODE_NAME);
    }
    else
    {
        RCUTILS_LOG_ERROR_NAMED(RCL_NODE_NAME, "failed to create %s node", RCL_NODE_NAME);
        exit(0);
    }

    const rcl_action_server_options_t &gts_goal_planner_server_opts = rcl_action_server_get_default_options();
    this->gts_goal_planner_server_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->gts_goal_planner_server_ = rclcpp_action::create_server<gts_msgs::action::GoalPlanner>(
        this->node_,
        GTS_GOAL_PLANNER_SERVER_NAME,
        std::bind(&gts_goal_planner::planner::GoalPlanner::gts_handle_goal, this, _1, _2),
        std::bind(&gts_goal_planner::planner::GoalPlanner::gts_handle_cancel, this, _1),
        std::bind(&gts_goal_planner::planner::GoalPlanner::gts_handle_accepted, this, _1),
        gts_goal_planner_server_opts,
        this->gts_goal_planner_server_cb_group_);

    this->ntp_goal_ = std::make_shared<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::Goal>();
    
    const rcl_action_client_options_t &ntp_client_opts = rcl_action_client_get_default_options();
    this->ntp_client_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->ntp_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->node_,
        NTP_CLIENT_NAME,
        this->ntp_client_cb_group_,
        ntp_client_opts);
}

gts_goal_planner::planner::GoalPlanner::~GoalPlanner()
{
}

rclcpp_action::GoalResponse gts_goal_planner::planner::GoalPlanner::gts_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const gts_msgs::action::GoalPlanner::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(this->node_->get_logger(), "gts handle_goal");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse gts_goal_planner::planner::GoalPlanner::gts_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{
    RCLCPP_INFO(this->node_->get_logger(), "gts cancel_goal");

    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void gts_goal_planner::planner::GoalPlanner::gts_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{
    std::thread{std::bind(&gts_goal_planner::planner::GoalPlanner::gts_execute, this, _1), goal_handle}.detach();
}

void gts_goal_planner::planner::GoalPlanner::gts_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{
    RCLCPP_INFO(this->node_->get_logger(), "gts execute_goal");
    const std::shared_ptr<const gts_msgs::action::GoalPlanner_Goal> goal = goal_handle->get_goal();

    std::vector<sensor_msgs::msg::NavSatFix, std::allocator<sensor_msgs::msg::NavSatFix>> goal_request = goal->gps_goal_list;
    this->slam_waypoints_list_ = goal_request;
    this->slam_waypoints_list_index_ = 0;
    this->slam_waypoints_list_size_ = this->slam_waypoints_list_.size();

    RCLCPP_INFO(this->node_->get_logger(), "gts execute_goal list size : [%d]", slam_waypoints_list_size_);

    this->ntp_send_goal();

    std::shared_ptr<gts_msgs::action::GoalPlanner::Feedback> feedback = std::make_shared<gts_msgs::action::GoalPlanner::Feedback>();
    std::shared_ptr<gts_msgs::action::GoalPlanner::Result> result = std::make_shared<gts_msgs::action::GoalPlanner::Result>();

    goal_handle->succeed(result);
}

void gts_goal_planner::planner::GoalPlanner::ntp_send_goal()
{
    bool is_slam_waypoints_list_empty = this->slam_waypoints_list_.empty();

    if (is_slam_waypoints_list_empty)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "ntp send goal slam_waypoints_list is empty...");
        return;
    }

    const double &slam_x = this->slam_waypoints_list_[slam_waypoints_list_index_].longitude;
    const double &slam_y = this->slam_waypoints_list_[slam_waypoints_list_index_].latitude;

    bool is_navigate_to_pose_server_ready = this->ntp_client_->wait_for_action_server(std::chrono::seconds(5));

    if (!is_navigate_to_pose_server_ready)
    {
        RCLCPP_ERROR(this->node_->get_logger(), "ntp action server is not available after waiting... vacate SLAM waypoints_list");
        this->slam_waypoints_list_.clear();
        this->slam_waypoints_list_index_ = DEFAULT_INT;
        this->slam_waypoints_list_size_ = DEFAULT_INT;
        return;
    }

    RCLCPP_INFO(
        this->node_->get_logger(),
        "ntp action server is ready\n\tslam x : [%f]\n\tslam y : [%f]",
        slam_x,
        slam_y);

    geometry_msgs::msg::PoseStamped::UniquePtr geometry_msgs_pose_stamped = std::make_unique<geometry_msgs::msg::PoseStamped>();

    geometry_msgs::msg::Point::UniquePtr geometry_msgs_point = std::make_unique<geometry_msgs::msg::Point>();
    geometry_msgs_point->set__x(slam_x);
    geometry_msgs_point->set__y(slam_y);
    geometry_msgs_point->set__z(DEFAULT_DOUBLE);

    geometry_msgs::msg::Quaternion::UniquePtr geometry_msgs_quaternion = std::make_unique<geometry_msgs::msg::Quaternion>();
    geometry_msgs_quaternion->set__x(DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__y(DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__z(DEFAULT_DOUBLE);
    geometry_msgs_quaternion->set__w(DEFAULT_DOUBLE);

    geometry_msgs::msg::Pose::UniquePtr geometry_msgs_pose = std::make_unique<geometry_msgs::msg::Pose>();

    const geometry_msgs::msg::Point &&geometry_msgs_point_moved = std::move(*geometry_msgs_point);
    geometry_msgs_pose->set__position(geometry_msgs_point_moved);

    const geometry_msgs::msg::Quaternion &&geometry_msgs_quaternion_moved = std::move(*geometry_msgs_quaternion);
    geometry_msgs_pose->set__orientation(geometry_msgs_quaternion_moved);

    const geometry_msgs::msg::Pose &&geometry_msgs_pose_moved = std::move(*geometry_msgs_pose);
    geometry_msgs_pose_stamped->set__pose(geometry_msgs_pose_moved);

    const geometry_msgs::msg::PoseStamped &&geometry_msgs_pose_stamped_moved = std::move(*geometry_msgs_pose_stamped);
    this->ntp_goal_->set__pose(geometry_msgs_pose_stamped_moved);

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions navigate_to_pose_send_goal_opts = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    navigate_to_pose_send_goal_opts.feedback_callback = std::bind(&gts_goal_planner::planner::GoalPlanner::ntp_feedback_cb, this, _1, _2);
    navigate_to_pose_send_goal_opts.goal_response_callback = std::bind(&gts_goal_planner::planner::GoalPlanner::ntp_goal_response_cb, this, _1);
    navigate_to_pose_send_goal_opts.result_callback = std::bind(&gts_goal_planner::planner::GoalPlanner::ntp_result_cb, this, _1);

    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> navigate_to_pose_goal_future = this->ntp_client_->async_send_goal(*ntp_goal_, navigate_to_pose_send_goal_opts);
    this->ntp_goal_handle_ = navigate_to_pose_goal_future.get();

    RCLCPP_INFO(
        this->node_->get_logger(),
        "ntp goal sent\n\tpose_x : [%f]\n\tpose_y : [%f]\n\torien_z: [%f]\n\torien_w : [%f]",
        ntp_goal_->pose.pose.position.x,
        ntp_goal_->pose.pose.position.y,
        ntp_goal_->pose.pose.orientation.z,
        ntp_goal_->pose.pose.orientation.w);
}

void gts_goal_planner::planner::GoalPlanner::ntp_feedback_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{

}

void gts_goal_planner::planner::GoalPlanner::ntp_goal_response_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
{

}

void gts_goal_planner::planner::GoalPlanner::ntp_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &wrapped_result)
{

}

std_msgs::msg::Header gts_goal_planner::planner::GoalPlanner::build_header(const char *frame_id)
{
    std::chrono::_V2::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::_V2::system_clock::duration current_time_duration = current_time.time_since_epoch();

    const int32_t &current_time_sec = std::chrono::duration_cast<std::chrono::seconds>(current_time_duration).count();
    const int32_t &current_time_nanosec = current_time_sec % 1000000000;

    builtin_interfaces::msg::Time::UniquePtr time = std::make_unique<builtin_interfaces::msg::Time>();
    time->set__sec(current_time_sec);
    time->set__nanosec(current_time_nanosec);

    std_msgs::msg::Header::UniquePtr header = std::make_unique<std_msgs::msg::Header>();
    header->set__frame_id(frame_id);

    const builtin_interfaces::msg::Time &&time_moved = std::move(*time);
    header->set__stamp(time_moved);

    const std_msgs::msg::Header &&header_moved = std::move(*header);

    return header_moved;
}

void gts_goal_planner::planner::GoalPlanner::position_test()
{
    gts_goal_planner::position::Point test_pos_1 = gts_goal_planner::position::Point();
    test_pos_1.set__x(0.0);
    test_pos_1.set__y(0.0);

    gts_goal_planner::position::Point test_pos_2 = gts_goal_planner::position::Point();
    test_pos_2.set__x(1238.0);
    test_pos_2.set__y(765);

    gts_goal_planner::position::Point test_pos_3 = gts_goal_planner::position::Point();
    test_pos_3.set__x(415.0);
    test_pos_3.set__y(235.0);

    gts_goal_planner::position::Point test_pos_4 = gts_goal_planner::position::Point();
    test_pos_4.set__x(574.0);
    test_pos_4.set__y(235.0);

    gts_goal_planner::position::Point test_pos_5 = gts_goal_planner::position::Point();
    test_pos_5.set__x(1210.0);
    test_pos_5.set__y(235.0);

    gts_goal_planner::position::Point test_pos_6 = gts_goal_planner::position::Point();
    test_pos_6.set__x(1210.0);
    test_pos_6.set__y(541.0);

    int slam_map_width = 1238;
    int slam_map_height = 765;

    double intersection_start_point_lon = 128.858009083;
    double intersection_start_point_lat = 35.157430158;
    double intersection_end_point_lon = 128.858870603;
    double intersection_end_point_lat = 35.158056682;

    int std_point_slam_x1 = 415;
    int std_point_slam_y1 = 235;
    int std_point_slam_x2 = 1210;
    int std_point_slam_y2 = 541;

    double std_point_lon1 = 128.8579836;
    double std_point_lat1 = 35.1576298;
    double std_point_lon2 = 128.858333;
    double std_point_lat2 = 35.15818;

    int shift = 0;

    gts_goal_planner::position::Point std_point_1 = gts_goal_planner::position::Point();
    std_point_1.set__x(std_point_lon1);
    std_point_1.set__y(std_point_lat1);

    gts_goal_planner::position::Point std_point_2 = gts_goal_planner::position::Point();
    std_point_2.set__x(std_point_lon2);
    std_point_2.set__y(std_point_lat2);

    gts_goal_planner::position::Point start_point = gts_goal_planner::position::Point();
    start_point.set__x(intersection_start_point_lon);
    start_point.set__y(intersection_start_point_lat);

    gts_goal_planner::position::Point end_point = gts_goal_planner::position::Point();
    end_point.set__x(intersection_end_point_lon);
    end_point.set__y(intersection_end_point_lat);

    std::shared_ptr<gts_goal_planner::converter::Converter> converter = std::make_shared<gts_goal_planner::converter::Converter>();

    converter->initialize(
        std_point_slam_x1, std_point_slam_y1,
        std_point_slam_x2, std_point_slam_y2,
        slam_map_width, slam_map_height,
        std_point_1, std_point_2,
        start_point, end_point);

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [1]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_1 = converter->convert_slam_to_gps(test_pos_1.get__x(), test_pos_1.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_1 = converter->convert_gps_to_slam(gps_pos_test_1->get__x(), gps_pos_test_1->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_1->get__x(), gps_pos_test_1->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_1->get__x(), slam_pos_test_1->get__y());
    printf("============================\n\n");

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [2]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_2 = converter->convert_slam_to_gps(test_pos_2.get__x(), test_pos_2.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_2 = converter->convert_gps_to_slam(gps_pos_test_2->get__x(), gps_pos_test_2->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_2->get__x(), gps_pos_test_2->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_2->get__x(), slam_pos_test_2->get__y());
    printf("============================\n\n");

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [3]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_3 = converter->convert_slam_to_gps(test_pos_3.get__x(), test_pos_3.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_3 = converter->convert_gps_to_slam(gps_pos_test_3->get__x(), gps_pos_test_3->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_3->get__x(), gps_pos_test_3->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_3->get__x(), slam_pos_test_3->get__y());
    printf("============================\n\n");

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [4]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_4 = converter->convert_slam_to_gps(test_pos_4.get__x(), test_pos_4.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_4 = converter->convert_gps_to_slam(gps_pos_test_4->get__x(), gps_pos_test_4->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_4->get__x(), gps_pos_test_4->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_4->get__x(), slam_pos_test_4->get__y());
    printf("============================\n\n");

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [5]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_5 = converter->convert_slam_to_gps(test_pos_5.get__x(), test_pos_5.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_5 = converter->convert_gps_to_slam(gps_pos_test_5->get__x(), gps_pos_test_5->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_5->get__x(), gps_pos_test_5->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_5->get__x(), slam_pos_test_5->get__y());
    printf("============================\n\n");

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] [6]", CLASS_NAME);
    std::shared_ptr<gts_goal_planner::position::Point> gps_pos_test_6 = converter->convert_slam_to_gps(test_pos_6.get__x(), test_pos_6.get__y());
    std::shared_ptr<gts_goal_planner::position::Point> slam_pos_test_6 = converter->convert_gps_to_slam(gps_pos_test_6->get__x(), gps_pos_test_6->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] GPS : [%f, %f]", CLASS_NAME, gps_pos_test_6->get__x(), gps_pos_test_6->get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] SLAM : [%f, %f]", CLASS_NAME, slam_pos_test_6->get__x(), slam_pos_test_6->get__y());
}

int main(int argc, const char *const *argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = std::make_shared<gts_goal_planner::planner::GoalPlanner>();
    rclcpp::executors::MultiThreadedExecutor multi_threaded_executor;
    multi_threaded_executor.add_node(node);
    multi_threaded_executor.spin();
    rclcpp::shutdown();
    return 0;
}