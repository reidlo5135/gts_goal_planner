#include "gts_goal_planner/gts_converter.hxx"

gts_goal_planner::converter::Converter::Converter()
{
    this->map_ = std::make_shared<gts_goal_planner::position::Map>();
    this->area_offset_ = std::make_shared<gts_goal_planner::position::Point>();
    this->lon_lat_LB_ = std::make_shared<gts_goal_planner::position::Point>();
    this->lon_lat_RT_ = std::make_shared<gts_goal_planner::position::Point>();
}

gts_goal_planner::converter::Converter::~Converter()
{
}

void gts_goal_planner::converter::Converter::initialize(
    int x1, int y1,
    int x2, int y2,
    int slam_width, int slam_height,
    gts_goal_planner::position::Point map_point_1, gts_goal_planner::position::Point map_point_2,
    gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat)
{
    std::vector<gts_goal_planner::position::Point> mapping_map_area_arr = this->convert_slam_virtual_map_area(
        x1, y1,
        x2, y2,
        slam_width, slam_height,
        map_point_1, map_point_2,
        start_lon_lat, end_lon_lat);

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] initialize mapping_map_area_arr[0]\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, mapping_map_area_arr[0].get__x(), mapping_map_area_arr[0].get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] initialize mapping_map_area_arr[1]\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, mapping_map_area_arr[1].get__x(), mapping_map_area_arr[1].get__y());

    gts_goal_planner::position::Point lon_lat_LB = mapping_map_area_arr[0];
    gts_goal_planner::position::Point lon_lat_RT = mapping_map_area_arr[1];

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] initialize lon_lat_LB\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, lon_lat_LB.get__x(), lon_lat_LB.get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] initialize lon_lat_RT\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, lon_lat_RT.get__x(), lon_lat_RT.get__y());

    this->init_area(
        slam_width, slam_height,
        start_lon_lat, end_lon_lat,
        lon_lat_LB, lon_lat_RT);
}

void gts_goal_planner::converter::Converter::init_area(
    int slam_width, int slam_height,
    gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat,
    gts_goal_planner::position::Point lon_lat_LB, gts_goal_planner::position::Point lon_lat_RT)
{
    double slam_rotation_angle = this->get_angle(start_lon_lat.get__x(), start_lon_lat.get__y(), end_lon_lat.get__x(), end_lon_lat.get__y());
    this->map_->set__slam_rotation_angle(slam_rotation_angle);

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area slam_rotation_angle : [%f]", CLASS_NAME, this->map_->get__slam_rotation_angle());

    if (slam_rotation_angle < 0)
    {
        return;
    }

    double mapping_map_width = round(sin(slam_rotation_angle) * slam_height + cos(slam_rotation_angle) * slam_width);
    this->map_->set__mapping_map_width(mapping_map_width);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area mapping_map_width : [%f]", CLASS_NAME, this->map_->get__mapping_map_width());

    double mapping_map_height = round(cos(slam_rotation_angle) * slam_height + sin(slam_rotation_angle) * slam_width);
    this->map_->set__mapping_map_height(mapping_map_height);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area mapping_map_height : [%f]", CLASS_NAME, this->map_->get__mapping_map_height());

    double area_offset_x = round(sin(slam_rotation_angle) * slam_height);
    this->area_offset_->set__x(area_offset_x);
    this->area_offset_->set__y(0.0);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area area_offset_x : [%f]", CLASS_NAME, this->area_offset_->get__x());

    this->lon_lat_LB_ = std::make_shared<gts_goal_planner::position::Point>(lon_lat_LB);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area lon_lat_LB_\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, this->lon_lat_LB_->get__x(), this->lon_lat_LB_->get__y());

    this->lon_lat_RT_ = std::make_shared<gts_goal_planner::position::Point>(lon_lat_RT);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] init_area lon_lat_RT_\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, this->lon_lat_RT_->get__x(), this->lon_lat_RT_->get__y());
}

std::shared_ptr<gts_goal_planner::position::Point> gts_goal_planner::converter::Converter::convert_gps_to_slam(double longitude, double latitude)
{
    double m_x = ((longitude - lon_lat_LB_->get__x()) / (lon_lat_RT_->get__x() - lon_lat_LB_->get__x())) * this->map_->get__mapping_map_width();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_gps_to_slam m_x : [%f]", CLASS_NAME, m_x);

    double m_y = ((latitude - lon_lat_LB_->get__y()) / (lon_lat_RT_->get__y() - lon_lat_LB_->get__y())) * this->map_->get__mapping_map_height();
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_gps_to_slam m_y : [%f]", CLASS_NAME, m_y);

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_gps_to_slam type : [%d]", CLASS_NAME, gts_goal_planner::position::WorkType::GPS);

    std::shared_ptr<gts_goal_planner::position::Point> s_point = convert_slam_pos(static_cast<int>(m_x), static_cast<int>(m_y), gts_goal_planner::position::WorkType::GPS);

    if (s_point == nullptr)
    {
        RCUTILS_LOG_ERROR(RCL_NODE_NAME, "[%s] convert_gps_to_slam convert_slam_pos is nullptr...aborting", CLASS_NAME);
        return nullptr;
    }

    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_gps_to_slam s_point\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, s_point->get__x(), s_point->get__y());

    return s_point;
}

std::shared_ptr<gts_goal_planner::position::Point> gts_goal_planner::converter::Converter::convert_slam_to_gps(int x, int y)
{
    bool is_mapping_map_lower_than_zero = (this->map_->get__mapping_map_width() <= 0) || (this->map_->get__mapping_map_height() <= 0);

    if (is_mapping_map_lower_than_zero)
    {
        RCUTILS_LOG_ERROR(RCL_NODE_NAME, "[%s] convert_slam_to_gps mapping_map is lower than zero...aborting", CLASS_NAME);
        return nullptr;
    }

    std::shared_ptr<gts_goal_planner::position::Point> s_point = this->convert_slam_pos(x, y, gts_goal_planner::position::WorkType::SLAM);

    if (s_point == nullptr)
    {
        RCUTILS_LOG_ERROR(RCL_NODE_NAME, "[%s] convert_slam_to_gps convert_slam_pos is nullptr...aborting", CLASS_NAME);
        return nullptr;
    }

    double longitude = this->lon_lat_LB_->get__x() + (this->lon_lat_RT_->get__x() - this->lon_lat_LB_->get__x()) * (s_point->get__x() / this->map_->get__mapping_map_width());
    double latitude = this->lon_lat_LB_->get__y() + (this->lon_lat_RT_->get__y() - this->lon_lat_LB_->get__y()) * (s_point->get__y() / this->map_->get__mapping_map_height());

    std::shared_ptr<gts_goal_planner::position::Point> point = std::make_shared<gts_goal_planner::position::Point>();
    point->set__x(longitude);
    point->set__y(latitude);

    return point;
}

std::shared_ptr<gts_goal_planner::position::Point> gts_goal_planner::converter::Converter::convert_slam_pos(int x, int y, gts_goal_planner::position::WorkType type)
{
    double slam_rotation_angle = this->map_->get__slam_rotation_angle();

    if (slam_rotation_angle <= 0)
    {
        RCUTILS_LOG_ERROR(RCL_NODE_NAME, "[%s] convert_slam_pos slam_rotation_angle is lower than zero...aborting", CLASS_NAME);
        return nullptr;
    }

    std::shared_ptr<gts_goal_planner::position::Point> point = std::make_shared<gts_goal_planner::position::Point>();

    if (type == gts_goal_planner::position::WorkType::SLAM)
    {
        double a = atan2(y, x);
        double len = sqrt(x * x + y * y);
        double m_x = cos(slam_rotation_angle + a) * len + this->area_offset_->get__x();
        double m_y = sin(slam_rotation_angle + a) * len;

        point->set__x(m_x);
        point->set__y(m_y);
    }
    else if (type == gts_goal_planner::position::WorkType::GPS)
    {
        double x_std = x - this->area_offset_->get__x();
        double y_std = y;
        double b = atan2(y_std, x_std);
        double len = round(sqrt(x_std * x_std + y_std * y_std));
        double s_x = cos(b - slam_rotation_angle) * len;
        double s_y = sin(b - slam_rotation_angle) * len;

        point->set__x(s_x);
        point->set__y(s_y);
    }
    else
    {
        return nullptr;
    }

    return point;
}

std::vector<gts_goal_planner::position::Point> gts_goal_planner::converter::Converter::convert_slam_virtual_map_area(
    int x1, int y1,
    int x2, int y2,
    int width, int height,
    gts_goal_planner::position::Point map_point_1, gts_goal_planner::position::Point map_point_2,
    gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat)
{
    double slam_rotation_angle = this->get_angle(start_lon_lat.get__x(), start_lon_lat.get__y(), end_lon_lat.get__x(), end_lon_lat.get__y());
    double slam_std_angle = this->get_angle(map_point_1.get__x(), map_point_1.get__y(), map_point_2.get__x(), map_point_2.get__y()) - slam_rotation_angle;

    double pt_distance_for_gps = get_distance_in_meter(map_point_1.get__y(), map_point_1.get__x(), map_point_2.get__y(), map_point_2.get__x());
    double x_distance_for_gps = pt_distance_for_gps * cos(slam_std_angle);
    double y_distance_for_gps = pt_distance_for_gps * sin(slam_std_angle);

    double x_distance_for_slam = abs(x2 - x1);
    double y_distance_for_slam = abs(y2 - y1);

    if (y_distance_for_gps == 0 || y_distance_for_slam == 0)
    {
        y_distance_for_gps = x_distance_for_gps;
        y_distance_for_slam = x_distance_for_slam;
    }

    double x_distance_per_pix = x_distance_for_gps / x_distance_for_slam;
    double y_distance_per_pix = y_distance_for_gps / y_distance_for_slam;

    double rt_point_angle = atan2((height - y2) * y_distance_per_pix, (width - x2) * x_distance_per_pix);
    double rt_dist_slam = sqrt(pow((height - y2) * y_distance_per_pix, 2) + pow((width - x2) * x_distance_per_pix, 2));

    double lb_point_angle = atan2(y1 * y_distance_per_pix, x1 * x_distance_per_pix);
    double lb_dist_slam = sqrt(pow(y1 * y_distance_per_pix, 2) + pow(x1 * x_distance_per_pix, 2));

    double height_distance = height * y_distance_per_pix;

    gts_goal_planner::position::Point right_top_pos = this->get_moving_lon_lat(
        map_point_2.get__x(), map_point_2.get__y(),
        rt_dist_slam, (slam_rotation_angle + rt_point_angle));
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area right_top_pos\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, right_top_pos.get__x(), right_top_pos.get__y());

    gts_goal_planner::position::Point left_bottom_pos = this->get_moving_lon_lat(
        map_point_1.get__x(), map_point_1.get__y(),
        lb_dist_slam, (slam_rotation_angle + lb_point_angle + M_PI));
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area left_bottom_pos\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, left_bottom_pos.get__x(), left_bottom_pos.get__y());

    gts_goal_planner::position::Point right_bottom_pos = this->get_moving_lon_lat(
        right_top_pos.get__x(), right_top_pos.get__y(),
        height_distance, (slam_rotation_angle + (M_PI + M_PI_2)));
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area right_bottom_pos\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, right_bottom_pos.get__x(), right_bottom_pos.get__y());

    gts_goal_planner::position::Point left_top_pos = this->get_moving_lon_lat(
        left_bottom_pos.get__x(), left_bottom_pos.get__y(),
        height_distance, (slam_rotation_angle + M_PI_2));
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area left_top_pos\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, left_top_pos.get__x(), left_top_pos.get__y());

    std::shared_ptr<gts_goal_planner::position::Point> left_point = std::make_shared<gts_goal_planner::position::Point>();
    left_point->set__x(left_top_pos.get__x());
    left_point->set__y(left_bottom_pos.get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area left_point\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, left_point->get__x(), left_point->get__y());

    std::shared_ptr<gts_goal_planner::position::Point> right_point = std::make_shared<gts_goal_planner::position::Point>();
    right_point->set__x(right_bottom_pos.get__x());
    right_point->set__y(right_top_pos.get__y());
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area right_point\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, right_point->get__x(), right_point->get__y());

    std::vector<gts_goal_planner::position::Point> mapping_map_area_arr;
    mapping_map_area_arr.push_back(*left_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area mapping_map_area_arr[0]\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, mapping_map_area_arr[0].get__x(), mapping_map_area_arr[0].get__y());

    mapping_map_area_arr.push_back(*right_point);
    RCUTILS_LOG_INFO_NAMED(RCL_NODE_NAME, "[%s] convert_slam_virtual_map_area mapping_map_area_arr[1]\n\tx : [%f]\n\ty : [%f]", CLASS_NAME, mapping_map_area_arr[1].get__x(), mapping_map_area_arr[1].get__y());

    return mapping_map_area_arr;
}

gts_goal_planner::position::Point gts_goal_planner::converter::Converter::get_moving_lon_lat(double lon, double lat, double distance, double radian)
{
    double dist_per_lat_degree = distance_in_meter_by_haversine(static_cast<int>(lat), lon, static_cast<int>(lat + 1), lon);
    double dist_per_lon_degree = distance_in_meter_by_haversine(lat, static_cast<int>(lon), lat, static_cast<int>(lon + 1));

    if (dist_per_lat_degree <= 0)
    {
        dist_per_lon_degree = 110941;
    }

    if (dist_per_lon_degree <= 0)
    {
        dist_per_lon_degree = 91290;
    }

    double quadrant_1 = 90 * M_PI / M_CIRC;
    double quadrant_2 = 180 * M_PI / M_CIRC;
    double quadrant_3 = 270 * M_PI / M_CIRC;

    double longitude_move = (sqrt(pow(distance, 2) - pow(sin(radian) * distance, 2))) / dist_per_lon_degree;
    double latitude_move = (sqrt(pow(distance, 2) - pow(cos(radian) * distance, 2))) / dist_per_lat_degree;

    double latitude = 0.0;
    double longitude = 0.0;

    if (quadrant_1 >= radian)
    {
        longitude = lon + longitude_move;
        latitude = lat + latitude_move;
    }
    else if (quadrant_2 >= radian)
    {
        longitude = lon - longitude_move;
        latitude = lat + latitude_move;
    }
    else if (quadrant_3 >= radian)
    {
        longitude = lon - longitude_move;
        latitude = lat - latitude_move;
    }
    else
    {
        longitude = lon + longitude_move;
        latitude = lat - latitude_move;
    }

    std::unique_ptr<gts_goal_planner::position::Point> point = std::make_unique<gts_goal_planner::position::Point>();

    point->set__x(longitude);
    point->set__y(latitude);

    const gts_goal_planner::position::Point &&point_moved = std::move(*point);

    return point_moved;
}

double gts_goal_planner::converter::Converter::get_angle(double lon1, double lat1, double lon2, double lat2)
{
    double y1 = lat1 * M_PI / M_CIRC;
    double y2 = lat2 * M_PI / M_CIRC;

    double x1 = lon1 * M_PI / M_CIRC;
    double x2 = lon2 * M_PI / M_CIRC;

    double y = sin(x2 - x1) * cos(y2);
    double x = cos(y1) * sin(y2) - sin(y1) * cos(y2) * cos(x2 - x1);

    double theta = atan2(y, x);
    double angle = M_PI_2 - theta;

    return angle;
}

double gts_goal_planner::converter::Converter::get_distance_in_meter(double lat1, double lon1, double lat2, double lon2)
{
    double theta = lon1 - lon2;
    double dist = sin(this->deg2rad(lat1)) * sin(this->deg2rad(lat2)) + cos(this->deg2rad(lat1)) * cos(this->deg2rad(lat2)) * cos(this->deg2rad(theta));

    dist = M_RADIUS * acos(dist) * 1000.0;

    return dist;
}

double gts_goal_planner::converter::Converter::deg2rad(double deg)
{
    double radian = deg * M_PI / M_CIRC;

    return radian;
}

double gts_goal_planner::converter::Converter::rad2deg(double rad)
{
    double degree = rad * M_CIRC / M_PI;

    return degree;
}

double gts_goal_planner::converter::Converter::distance_in_meter_by_haversine(double lat1, double lon1, double lat2, double lon2)
{
    double delta_latitude = abs(this->deg2rad(lat1 - lat2));
    double delta_longitude = abs(this->deg2rad(lon1 - lon2));

    double sin_delta_lat = sin(delta_latitude / 2);
    double sin_delta_lon = sin(delta_longitude / 2);

    double square_root = sqrt(sin_delta_lat * sin_delta_lat + cos(this->deg2rad(lat1)) * cos(this->deg2rad(lat2)) * sin_delta_lon * sin_delta_lon);

    double distance = 2 * M_RADIUS * asin(square_root) * 1000.0;

    return distance;
}