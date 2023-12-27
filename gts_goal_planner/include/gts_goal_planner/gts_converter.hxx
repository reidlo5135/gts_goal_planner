#ifndef GTS_CONVERTER__HXX
#define GTS_CONVERTER__HXX

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <vector>

#include <rcutils/logging_macros.h>

#define __USE_MISC
#include <math.h>

#define M_CIRC 180.0
#define M_RADIUS 6371.0

#define RCL_NODE_NAME "gts_goal_planner"
#define CLASS_NAME "gts_converter"

namespace gts_goal_planner
{
    namespace position
    {
        class Point
        {
        private:
            double x_;
            double y_;

        public:
            explicit Point(){};
            virtual ~Point(){};

            double get__x()
            {
                return this->x_;
            }
            void set__x(double x)
            {
                this->x_ = x;
            }
            double get__y()
            {
                return this->y_;
            }
            void set__y(double y)
            {
                this->y_ = y;
            }
        };

        class Map
        {
        private:
            double mapping_map_width_;
            double mapping_map_height_;
            double slam_rotation_angle_;

        public:
            explicit Map(){};
            virtual ~Map(){};

            double get__mapping_map_width()
            {
                return this->mapping_map_width_;
            }
            void set__mapping_map_width(double mapping_map_width)
            {
                this->mapping_map_width_ = mapping_map_width;
            }
            double get__mapping_map_height()
            {
                return this->mapping_map_height_;
            }
            void set__mapping_map_height(double mapping_map_height)
            {
                this->mapping_map_height_ = mapping_map_height;
            }
            double get__slam_rotation_angle()
            {
                return this->slam_rotation_angle_;
            }
            void set__slam_rotation_angle(double slam_rotation_angle)
            {
                this->slam_rotation_angle_ = slam_rotation_angle;
            }
        };

        enum class WorkType
        {
            SLAM = 1,
            GPS = 2
        };
    }

    namespace converter
    {
        class Converter
        {
        private:
            std::shared_ptr<gts_goal_planner::position::Map> map_;
            std::shared_ptr<gts_goal_planner::position::Point> area_offset_;
            std::shared_ptr<gts_goal_planner::position::Point> lon_lat_LB_;
            std::shared_ptr<gts_goal_planner::position::Point> lon_lat_RT_;

        public:
            explicit Converter();
            virtual ~Converter();
            void initialize(
                int x1, int y1,
                int x2, int y2,
                int slam_width, int slam_height,
                gts_goal_planner::position::Point map_point_1, gts_goal_planner::position::Point map_point_2,
                gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat);

            void init_area(
                int slam_width, int slam_height,
                gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat,
                gts_goal_planner::position::Point lon_lat_LB, gts_goal_planner::position::Point lon_lat_RT);

            std::shared_ptr<gts_goal_planner::position::Point> convert_gps_to_slam(double longitude, double latitude);
            std::shared_ptr<gts_goal_planner::position::Point> convert_slam_to_gps(int x, int y);
            std::shared_ptr<gts_goal_planner::position::Point> convert_slam_pos(int x, int y, gts_goal_planner::position::WorkType type);
            std::vector<gts_goal_planner::position::Point> convert_slam_virtual_map_area(
                int x1, int y1,
                int x2, int y2,
                int width, int height,
                gts_goal_planner::position::Point map_point_1, gts_goal_planner::position::Point map_point_2,
                gts_goal_planner::position::Point start_lon_lat, gts_goal_planner::position::Point end_lon_lat);
            gts_goal_planner::position::Point get_moving_lon_lat(double lon, double lat, double distance, double radian);
            double get_angle(double lon1, double lat1, double lon2, double lat2);
            double get_distance_in_meter(double lat1, double lon1, double lat2, double lon2);
            double deg2rad(double deg);
            double rad2deg(double rad);
            double distance_in_meter_by_haversine(double lat1, double lon1, double lat2, double lon2);
        };
    }
}

#endif