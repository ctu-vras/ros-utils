// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief C interface for ros::console .
 * \author Martin Pecka
 */

#include <ros/console.h>

extern "C" {

extern const int ROSCONSOLE_LEVEL_DEBUG = ros::console::Level::Debug;
extern const int ROSCONSOLE_LEVEL_INFO = ros::console::Level::Info;
extern const int ROSCONSOLE_LEVEL_WARN = ros::console::Level::Warn;
extern const int ROSCONSOLE_LEVEL_ERROR = ros::console::Level::Error;
extern const int ROSCONSOLE_LEVEL_FATAL = ros::console::Level::Fatal;

void ros_console_notifyLoggerLevelsChanged()
{
    ros::console::notifyLoggerLevelsChanged();
}

bool ros_console_set_logger_level(const char* loggerName, int level)
{
    return ros::console::set_logger_level(loggerName, static_cast<ros::console::Level>(level));
}

}
