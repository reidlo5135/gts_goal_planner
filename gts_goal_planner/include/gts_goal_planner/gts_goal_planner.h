#ifndef GTS_GOAL_PLANNER__H
#define GTS_GOAL_PLANNER__H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <signal.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <action_msgs/msg/goal_status.h>

#include "gts_goal_planner/gts_converter.h"

#define RCCHECK(fn)                                                                                      \
    {                                                                                                    \
        rcl_ret_t temp_rc = fn;                                                                          \
        if ((temp_rc != RCL_RET_OK))                                                                     \
        {                                                                                                \
            RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME,                                                        \
                                   "Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            return 1;                                                                                    \
        }                                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                                    \
    {                                                                                                      \
        rcl_ret_t temp_rc = fn;                                                                            \
        if ((temp_rc != RCL_RET_OK))                                                                       \
        {                                                                                                  \
            RCUTILS_LOG_INFO_NAMED(ROS_NODE_NAME,                                                          \
                                   "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                                  \
    }

#endif