/**
 * \file
 * \brief Helper for unit tests that eases examining what did rosconsole print.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <vector>

#include <ros/console.h>

/**
 * \brief Log appender that records what it was asked to log.
 */
struct Appender : public ros::console::LogAppender
{
  void log(::ros::console::Level level, const char* str, const char* file, const char* function, int line) override
  {
    this->str = str;
    this->strs.emplace_back(str);
    this->level = level;
    this->levels.push_back(level);
    this->num++;
    this->nums.push_back(this->num);
  }

  /**
   * \brief If it's possible that the log macro did not print anything, make sure to call this function to record the
   *        empty values to the array variables. Also be sure to call resetMsg() before the logging macro.
   */
  void afterLog()
  {
    if (this->str.empty() && this->level == ros::console::Level::Count)
    {
      this->strs.emplace_back("");
      this->levels.push_back(ros::console::Level::Count);
      this->nums.push_back(this->num);
    }
  }

  /**
   * \brief Reset str and level.
   */
  void resetMsg()
  {
    this->str = "";
    this->level = ::ros::console::Level::Count;
  }

  /**
   * \brief Reset str, strs, level, levels, num and nums.
   */
  void reset()
  {
    this->resetMsg();
    this->strs.clear();
    this->levels.clear();
    this->num = 0;
    this->nums.clear();
  }

  //! \brief Last logged string.
  std::string str {};

  //! \brief All messages logged from the last reset() call (latest message last).
  std::vector<std::string> strs;

  //! \brief Last logged level.
  ros::console::Level level {::ros::console::Level::Count};

  //! \brief All levels of logged messages from the last reset() call (latest message last).
  std::vector<ros::console::Level> levels;

  //! \brief Number of logged messages since last reset().
  size_t num {0};

  //! \brief Values of `num` for all messages logged from the last reset() call (latest message last).
  std::vector<size_t> nums;
};

//! \brief The custom log appender. Set it by calling `ros::console::register_appender(&logger);` from main().
Appender logger;
