#include "gts_goal_planner/gts_converter.h"

Position convert_slam_pos(int x, int y, WorkType type)
{
    
}

Position *convert_slam_virtual_map_area(int x1, int y1, int x2, int y2, int width, int height, Position map_point_1, Position map_point_2, Position start_lon_lat, Position end_lon_lat)
{
    double slam_rotation_angle = get_angle(start_lon_lat.x, start_lon_lat.y, end_lon_lat.x, end_lon_lat.y);
    double slam_std_angle = get_angle(map_point_1.x, map_point_1.y, map_point_2.x, map_point_2.y) - slam_rotation_angle;

    double pt_distance_for_gps = get_distance_in_meter(map_point_1.y, map_point_1.x, map_point_2.y, map_point_2.x);
    double x_distance_for_gps = pt_distance_for_gps * cos(slam_std_angle);
    double y_distance_for_gps = pt_distance_for_gps * sin(slam_std_angle);

    double pt_distance_for_slam = abs(x2 - x1);
    double x_distance_for_slam = abs(x2 - x1);
    double y_distance_for_slam = abs(y2 - y2);

    if (y_distance_for_gps == 0 || y_distance_for_slam == 0)
    {
        y_distance_for_gps = x_distance_for_gps;
        y_distance_for_slam = x_distance_for_slam;
    }

    double x_distance_per_pixels = x_distance_for_gps / x_distance_for_slam;
    double y_distance_per_pixels = y_distance_for_gps / y_distance_for_slam;

    double rt_point_angle = atan2((height - y2) * y_distance_per_pixels, (width - 2) * x_distance_per_pixels);
    double rt_dist_slam = sqrt(pow((height - y2) * y_distance_per_pixels, 2) + pow((width - x2) * x_distance_per_pixels, 2));

    double lb_point_angle = atan2(y1 * y_distance_per_pixels, x1 * x_distance_per_pixels);
    double lb_dist_slam = sqrt(pow(y1 * y_distance_per_pixels, 2) + pow(x1 * x_distance_per_pixels, 2));

    double height_distance = height * y_distance_per_pixels;

    Position right_top_pos = get_moving_lon_lat(map_point_2.x, map_point_2.y, rt_dist_slam, (slam_rotation_angle + rt_point_angle));
    Position left_bottom_pos = get_moving_lon_lat(map_point_1.x, map_point_1.y, lb_dist_slam, (slam_rotation_angle + lb_point_angle + M_PI));
    Position right_bottom_pos = get_moving_lon_lat(right_top_pos.x, right_top_pos.y, height_distance, (slam_rotation_angle + (M_PI + M_PI_2)));
    Position left_top_pos = get_moving_lon_lat(left_bottom_pos.x, left_bottom_pos.y, height_distance, (slam_rotation_angle + M_PI_2));

    Position left_pos;
    left_pos.x = left_top_pos.x;
    left_pos.y = left_bottom_pos.y;

    Position right_pos;
    right_pos.x = right_bottom_pos.x;
    right_pos.y = right_top_pos.y;

    Position *position_arr = (Position *)malloc(2 * sizeof(Position));

    if (position_arr == NULL)
    {
        RCUTILS_LOG_INFO(RCL_NODE_NAME, "failed to allocate position arr in convert_slam_virtual_map_area");
        return NULL;
    }

    position_arr[0] = left_pos;
    position_arr[1] = right_pos;

    return position_arr;
}

Position get_moving_lon_lat(double lon, double lat, double distance, double radian)
{
    double dist_per_lat_degree = distance_in_meter_by_haversine((int)lat, lon, (int)lat + 1, lon);
    double dist_per_lon_degree = distance_in_meter_by_haversine(lat, (int)lon, lat, (int)lon + 1);

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
    double latitude_move = sqrt(pow(distance, 2) - pow(cos(radian) * distance, 2)) / dist_per_lat_degree;

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

    return position;
}

double get_angle(double lon1, double lat1, double lon2, double lat2)
{
    double y1 = lat1 * M_PI / M_CIRC;
    double y2 = lat2 * M_PI / M_CIRC;

    double x1 = lon1 * M_PI / M_CIRC;
    double x2 = lon2 * M_PI / M_CIRC;

    double y = sin(x2 - x1) * cos(y2);
    double x = cos(y1) * sin(y2) - sin(y1) * cos(y2) * cos(y2 - y2);

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