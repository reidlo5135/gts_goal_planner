#ifndef GTS_CONVERTER__H
#define GTS_CONVERTER__H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rcutils/logging_macros.h>

#define __USE_MISC
#include <math.h>

#define M_CIRC 180.0
#define M_RADIUS 6371.0

#define RCL_NODE_NAME "gts_goal_planner"

typedef struct _Position
{
    double x;
    double y;
} Position;

struct Position PositionArr[];

typedef struct _MapInfo
{
    double mapping_map_width;
    double mapping_map_height;
    double slam_rotation_angle;
} MapInfo;

typedef enum _WorkType
{
    SLAM,
    GPS
} WorkType;

void initialize(
    int x1, int y1,
    int x2, int y2,
    int slam_width, int slam_height,
    Position map_point_1, Position map_point_2,
    Position start_lon_lat, Position end_lon_lat);

void init_area(
    int slam_width, int slam_height,
    Position start_lon_lat, Position end_lon_lat,
    Position lon_lat_LB, Position lon_lat_RT);

Position convert_gps_to_slam(double longitude, double latitude);
Position convert_slam_to_gps(int x, int y);
Position convert_slam_pos(int x, int y, WorkType type);
Position *convert_slam_virtual_map_area(
    int x1, int y1,
    int x2, int y2,
    int width, int height,
    Position map_point_1, Position map_point_2,
    Position start_lon_lat, Position end_lon_lat);
Position get_moving_lon_lat(double lon, double lat, double distance, double radian);
double get_angle(double lon1, double lat1, double lon2, double lat2);
double get_distance_in_meter(double lat1, double lon1, double lat2, double lon2);
double deg2rad(double deg);
double rad2deg(double rad);
double distance_in_meter_by_haversine(double lat1, double lon1, double lat2, double lon2);

#endif