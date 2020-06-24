#pragma once

#include <ros/ros.h>

namespace cras {

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return
 */
ros::Duration remainingTime(const ros::Time &query, double timeout);

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return
 */
ros::Duration remainingTime(const ros::Time &query,
                            const ros::Duration &timeout);

}
