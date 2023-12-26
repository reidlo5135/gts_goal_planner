#include "gts_goal_planner/gts_goal_planner.h"

int main(int argc, const char *const *argv)
{
    Position test_pos_1;
    test_pos_1.x = 0.0;
    test_pos_1.y = 0.0;

    Position test_pos_2;
    test_pos_2.x = 1238.0;
    test_pos_2.y = 765;

    Position test_pos_3;
    test_pos_3.x = 415.0;
    test_pos_3.y = 235.0;

    Position test_pos_4;
    test_pos_4.x = 574.0;
    test_pos_4.y = 235.0;

    Position test_pos_5;
    test_pos_5.x = 1210.0;
    test_pos_5.y = 235.0;

    Position test_pos_6;
    test_pos_6.x = 1210.0;
    test_pos_6.y = 541.0;

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

    Position std_point_1;
    std_point_1.x = std_point_lon1;
    std_point_1.y = std_point_lat1;
    printf("main std_point_1\n\tx : [%f]\n\ty : [%f]\n", std_point_1.x, std_point_1.y);

    Position std_point_2;
    std_point_2.x = std_point_lon2;
    std_point_2.y = std_point_lat2;
    printf("main std_point_2\n\tx : [%f]\n\ty : [%f]\n", std_point_2.x, std_point_2.y);

    Position start_point;
    start_point.x = intersection_start_point_lon;
    start_point.y = intersection_start_point_lat;
    printf("main start_point\n\tx : [%f]\n\ty : [%f]\n", start_point.x, start_point.y);

    Position end_point;
    end_point.x = intersection_end_point_lon;
    end_point.y = intersection_end_point_lat;
    printf("main end_point\n\tx : [%f]\n\ty : [%f]\n", end_point.x, end_point.y);

    converter_initialize(
        std_point_slam_x1, std_point_slam_y1,
        std_point_slam_x2, std_point_slam_y2,
        slam_map_width, slam_map_height,
        std_point_1, std_point_2,
        start_point, end_point);

    printf("[1]\n");

    Position gps_pos_test_1 = convert_slam_to_gps(test_pos_1.x, test_pos_1.y);
    Position slam_pos_test_1 = convert_gps_to_slam(gps_pos_test_1.x, gps_pos_test_1.y);
    
    printf("GPS : %f, %f\n", gps_pos_test_1.x, gps_pos_test_1.y);
    printf("SLAM : %f, %f\n", slam_pos_test_1.x , slam_pos_test_1.y);

    return 0;
}