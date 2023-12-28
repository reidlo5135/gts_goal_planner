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

    this->gts_goal_planner_server_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->gts_goal_planner_server_ = rclcpp_action::create_server<gts_msgs::action::GoalPlanner>(
        this->node_,
        ACTION_SERVER_NAME,
        std::bind(&gts_goal_planner::planner::GoalPlanner::handle_goal, this, _1, _2),
        std::bind(&gts_goal_planner::planner::GoalPlanner::handle_cancel, this, _1),
        std::bind(&gts_goal_planner::planner::GoalPlanner::handle_accepted, this, _1),
        rcl_action_server_get_default_options(),
        this->gts_goal_planner_server_cb_group_);
}

gts_goal_planner::planner::GoalPlanner::~GoalPlanner()
{

}

rclcpp_action::GoalResponse gts_goal_planner::planner::GoalPlanner::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const gts_msgs::action::GoalPlanner::Goal> goal)
{

}

rclcpp_action::CancelResponse gts_goal_planner::planner::GoalPlanner::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{

}

void gts_goal_planner::planner::GoalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{

}

void gts_goal_planner::planner::GoalPlanner::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<gts_msgs::action::GoalPlanner>> goal_handle)
{

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