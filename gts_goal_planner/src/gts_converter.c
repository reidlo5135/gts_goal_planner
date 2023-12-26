#include "gts_goal_planner/gts_converter.h"

MapInfo map_info_;
Position area_offeset_;
Position lon_lat_LB_;
Position lon_lat_RT_;

void converter_initialize(int x1, int y1, int x2, int y2, int slam_width, int slam_height, Position map_point_1, Position map_point_2, Position start_lon_lat, Position end_lon_lat)
{
    Position *mapping_map_area_arr = convert_slam_virtual_map_area(
        x1, y1,
        x2, y2,
        slam_width, slam_height,
        map_point_1, map_point_2,
        start_lon_lat, end_lon_lat);

    Position lon_lat_LB = mapping_map_area_arr[0];
    printf("converter_initialize lon_lat_LB\n\tx : [%f]\n\ty : [%f]\n", lon_lat_LB.x, lon_lat_LB.y);

    Position lon_lat_RT = mapping_map_area_arr[1];
    printf("converter_initialize lon_lat_RT\n\tx : [%f]\n\ty : [%f]\n", lon_lat_RT.x, lon_lat_RT.y);

    init_area(slam_width, slam_height, start_lon_lat, end_lon_lat, lon_lat_LB, lon_lat_RT);

    free(mapping_map_area_arr);
}

void init_area(int slam_width, int slam_height, Position start_lon_lat, Position end_lon_lat, Position lon_lat_LB, Position lon_lat_RT)
{
    map_info_.slam_rotation_angle = get_angle(start_lon_lat.x, start_lon_lat.y, end_lon_lat.x, end_lon_lat.y);
    printf("init_area map_info_.slam_rotation_angle : [%f]\n", map_info_.slam_rotation_angle);

    double slam_rotation_angle = map_info_.slam_rotation_angle;

    if (slam_rotation_angle < 0)
    {
        return;
    }

    map_info_.mapping_map_width = round(sin(slam_rotation_angle) * slam_height + cos(slam_rotation_angle) * slam_width);
    printf("init_area map_info_.mapping_map_width : [%f]\n", map_info_.mapping_map_width);

    map_info_.mapping_map_height = round(cos(slam_rotation_angle) * slam_height + cos(slam_rotation_angle) * slam_width);
    printf("init_area map_info_.mapping_map_height : [%f]\n", map_info_.mapping_map_height);

    area_offeset_.x = round(sin(slam_rotation_angle) * slam_height);
    printf("init_area area_offeset_.x : [%f]\n", area_offeset_.x);

    area_offeset_.y = 0.0;
    printf("init_area area_offeset_.y : [%f]\n", area_offeset_.y);

    lon_lat_LB_ = lon_lat_LB;
    printf("init_area lon_lat_LB_\n\tx : [%f]\n\ty : [%f]\n", lon_lat_LB_.x, lon_lat_LB_.y);

    lon_lat_RT_ = lon_lat_RT;
    printf("init_area lon_lat_RT_\n\tx : [%f]\n\ty : [%f]\n", lon_lat_RT_.x, lon_lat_RT_.y);
}

Position convert_gps_to_slam(double longitude, double latitude)
{
    double m_x = ((longitude - lon_lat_LB_.x) / (lon_lat_RT_.x - lon_lat_LB_.x)) * map_info_.mapping_map_width;
    printf("convert_gps_to_slam m_x : [%f]\n", m_x);

    double m_y = ((latitude - lon_lat_LB_.y) / (lon_lat_RT_.y - lon_lat_LB_.y)) * map_info_.mapping_map_height;
    printf("convert_gps_to_slam m_y : [%f]\n", m_y);

    WorkType type = GPS;
    printf("convert_gps_to_slam type : [%d]\n", type);

    Position *pos_ptr = convert_slam_pos((int)m_x, (int)m_y, type);
    printf("convert_gps_to_slam pos_ptr\n\tx : [%f]\n\ty : [%f]\n", pos_ptr->x, pos_ptr->y);

    if (pos_ptr == NULL)
    {
        Position default_pos;
        default_pos.x = 0.0;
        default_pos.y = 0.0;

        return default_pos;
    }
    else
    {
        Position pos = *pos_ptr;
        printf("convert_gps_to_slam pos\n\tx : [%f]\n\ty : [%f]\n", pos.x, pos.y);
        free(pos_ptr);
        return pos;
    }
}

Position convert_slam_to_gps(int x, int y)
{
    if (map_info_.mapping_map_width <= 0 || map_info_.mapping_map_height <= 0)
    {
        RCUTILS_LOG_INFO(RCL_NODE_NAME, "map width and height is lower than zero... aborting");
        return *(Position *)NULL;
    }

    WorkType type = SLAM;
    Position *m_pos = convert_slam_pos(x, y, type);

    if (m_pos == NULL)
    {
        Position default_pos;
        default_pos.x = 0.0;
        default_pos.y = 0.0;

        return default_pos;
    }

    double longitude = lon_lat_LB_.x + (lon_lat_RT_.x - lon_lat_LB_.x) * (m_pos->x / map_info_.mapping_map_width);
    double latitude = lon_lat_LB_.y + (lon_lat_RT_.y - lon_lat_LB_.y) * (m_pos->y / map_info_.mapping_map_height);

    free(m_pos);

    Position pos;
    pos.x = longitude;
    pos.y = latitude;

    return pos;
}

Position *convert_slam_pos(int x, int y, WorkType type)
{
    double slam_rotation_angle = map_info_.slam_rotation_angle;

    if (slam_rotation_angle <= 0)
    {
        return NULL;
    }

    Position *pos = (Position *)malloc(sizeof(Position));

    if (type == WT_SLAM)
    {
        double a = atan2(y, x);
        double len = sqrt(x * x + y * y);
        double m_x = cos(slam_rotation_angle + a) * len + area_offeset_.x;
        double m_y = sin(slam_rotation_angle + a) * len;

        pos->x = m_x;
        pos->y = m_y;
    }
    else if (type == WT_GPS)
    {
        double x_std = x - area_offeset_.x;
        double y_std = y;
        double b = atan2(y_std, x_std);
        double len = round(sqrt(x_std * x_std + y_std * y_std));
        double s_x = cos(b - slam_rotation_angle) * len;
        double s_y = sin(b - slam_rotation_angle) * len;

        pos->x = s_x;
        pos->y = s_y;
    }
    else
    {
        return NULL;
    }

    return pos;
}

Position *convert_slam_virtual_map_area(int x1, int y1, int x2, int y2, int width, int height, Position map_point_1, Position map_point_2, Position start_lon_lat, Position end_lon_lat)
{
    double slam_rotation_angle = get_angle(start_lon_lat.x, start_lon_lat.y, end_lon_lat.x, end_lon_lat.y);
    double slam_std_angle = get_angle(map_point_1.x, map_point_1.y, map_point_2.x, map_point_2.y) - slam_rotation_angle;

    double pt_distance_for_gps = get_distance_in_meter(map_point_1.y, map_point_1.x, map_point_2.y, map_point_2.x);
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

    Position right_top_pos = get_moving_lon_lat(map_point_2.x, map_point_2.y, rt_dist_slam, (slam_rotation_angle + rt_point_angle));
    printf("convert_slam_virtual_map_area right_top_pos\n\tx : [%f]\n\ty : [%f]\n", right_top_pos.x, right_top_pos.y);

    Position left_bottom_pos = get_moving_lon_lat(map_point_1.x, map_point_1.y, lb_dist_slam, (slam_rotation_angle + lb_point_angle + M_PI));
    printf("convert_slam_virtual_map_area left_bottom_pos\n\tx : [%f]\n\ty : [%f]\n", left_bottom_pos.x, left_bottom_pos.y);

    Position right_bottom_pos = get_moving_lon_lat(right_top_pos.x, right_top_pos.y, height_distance, (slam_rotation_angle + (M_PI + M_PI_2)));
    printf("convert_slam_virtual_map_area right_bottom_pos\n\tx : [%f]\n\ty : [%f]\n", right_bottom_pos.x, right_bottom_pos.y);

    Position left_top_pos = get_moving_lon_lat(left_bottom_pos.x, left_bottom_pos.y, height_distance, (slam_rotation_angle + M_PI_2));
    printf("convert_slam_virtual_map_area left_top_pos\n\tx : [%f]\n\ty : [%f]\n", left_top_pos.x, left_top_pos.y);

    Position left_pos;
    left_pos.x = left_top_pos.x;
    left_pos.y = left_bottom_pos.y;
    printf("convert_slam_virtual_map_area left_pos\n\tx : [%f]\n\ty : [%f]\n", left_pos.x, left_pos.y);

    Position right_pos;
    right_pos.x = right_bottom_pos.x;
    right_pos.y = right_top_pos.y;
    printf("convert_slam_virtual_map_area right_pos\n\tx : [%f]\n\ty : [%f]\n", right_pos.x, right_pos.y);

    Position *position_arr = (Position *)malloc(2 * sizeof(Position));

    if (position_arr == NULL)
    {
        RCUTILS_LOG_INFO(RCL_NODE_NAME, "failed to allocate position arr in convert_slam_virtual_map_area");
        return NULL;
    }

    position_arr[0] = left_pos;
    printf("convert_slam_virtual_map_area position_arr[0]\n\tx : [%f]\n\ty : [%f]\n", position_arr[0].x, position_arr[0].y);

    position_arr[1] = right_pos;
    printf("convert_slam_virtual_map_area position_arr[1]\n\tx : [%f]\n\ty : [%f]\n", position_arr[1].x, position_arr[1].y);

    return position_arr;
}

Position get_moving_lon_lat(double lon, double lat, double distance, double radian)
{
    double dist_per_lat_degree = distance_in_meter_by_haversine((int)lat, lon, (int)lat + 1, lon);
    printf("get_moving_lon_lat dist_per_lat_degree : [%f]\n", dist_per_lat_degree);

    double dist_per_lon_degree = distance_in_meter_by_haversine(lat, (int)lon, lat, (int)lon + 1);
    printf("get_moving_lon_lat dist_per_lon_degree : [%f]\n", dist_per_lon_degree);

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

    double longitude_move = sqrt(pow(distance, 2) - pow(sin(radian) * distance, 2)) / dist_per_lon_degree;
    printf("get_moving_lon_lat longitude_move : [%f]\n", longitude_move);

    double latitude_move = sqrt(pow(distance, 2) - pow(cos(radian) * distance, 2)) / dist_per_lat_degree;
    printf("get_moving_lon_lat latitude_move : [%f]\n", latitude_move);

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

    Position position;
    position.x = longitude;
    position.y = latitude;
    printf("get_moving_lon_lat position\n\tx : [%f]\n\ty : [%f]\n", position.x, position.y);

    return position;
}

double get_angle(double lon1, double lat1, double lon2, double lat2)
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

double get_distance_in_meter(double lat1, double lon1, double lat2, double lon2)
{
    double theta = lon1 - lon2;
    double dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));

    dist = M_RADIUS * acos(dist) * 1000.0;

    return dist;
}

double deg2rad(double deg)
{
    double radian = deg * M_PI / M_CIRC;

    return radian;
}

double rad2deg(double rad)
{
    double degree = rad * M_CIRC / M_PI;

    return degree;
}

double distance_in_meter_by_haversine(double lat1, double lon1, double lat2, double lon2)
{
    double delta_latitude = abs(deg2rad(lat1 - lat2));
    double delta_longitude = abs(deg2rad(lon1 - lon2));

    double sin_delta_lat = sin(delta_latitude / 2);
    double sin_delta_lon = sin(delta_longitude / 2);

    double square_root = sqrt(sin_delta_lat * sin_delta_lat + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin_delta_lon * sin_delta_lon);

    double distance = 2 * M_RADIUS * asin(square_root) * 1000.0;

    return distance;
}