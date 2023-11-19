// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Interface for resettable nodes and nodelets. Automatic reset on time jumps.
 * \author Martin Pecka
 */

#pragma once

#include <memory>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <cras_cpp_common/log_utils.h>

namespace cras
{

struct ResettablePrivate;

/**
 * \brief Interface for resettable nodes and nodelets.
 *
 * This class subscribes the following topics:
 *  - `/reset` (any type): When a message is received on this topic, reset() is called.
 *  - `~reset` (any type): When a message is received on this topic, reset() is called.
 */
class Resettable : protected ::cras::HasLogger
{
public:
  /**
   * \brief Create the resettable interface. To also wire up to the reset topics, call initRos().
   * \param[in] log The logger to use for message logging.
   */
  explicit Resettable(const ::cras::LogHelperPtr& log);
  virtual ~Resettable();

  /**
   * \brief Do the resetting. Subclasses have to implement the logic.
   */
  virtual void reset() = 0;

  /**
   * \brief Initialize the ROS part of the interface - subscriber to /reset and ~reset topics.
   * \param[in] pnh The (private) node handle to setup the interface for.
   */
  virtual void initRos(const ::ros::NodeHandle& pnh);

private:
  ::std::unique_ptr<::cras::ResettablePrivate> data;  //!< \brief Private implementation (PIMPL) data.
};

struct TimeJumpResettablePrivate;

/**
 * \brief Interface for resettable nodes and nodelets. Automatic reset on time jumps.
 *
 * This class reads the following ROS parameters when initRos() is called:
 *
 *  - `/jump_back_tolerance` (float, default 3.0 s in wall time and 0.0 s in sim time):
 *      Threshold for ROS time jump back detection.
 *  - `~jump_back_tolerance` (float, default from `/jump_back_tolerance`): Threshold for ROS time jump back detection.
 *  - `~reset_on_time_jump_back` (bool, default True): Whether to call reset() when ROS time jumps back.
 *  - `/jump_forward_tolerance` (float, default 10.0 s in sim time and max duration in wall time):
 *      Threshold for ROS time jump forward detection.
 *  - `~jump_forward_tolerance` (float, default from `/jump_forward_tolerance`):
 *      Threshold for ROS time jump forward detection.
 *  - `~reset_on_time_jump_forward` (bool, default True in sim time and False in wall time):
 *      Whether to call reset() when ROS time jumps forward.
 *
 * This class subscribes the following topics:
 *  - `/reset` (any type): When a message is received on this topic, reset() is called.
 *  - `~reset` (any type): When a message is received on this topic, reset() is called.
 */
class TimeJumpResettable : public ::cras::Resettable
{
public:
  /**
   * \brief Create the resettable interface. To also wire up to the reset topics and time jump resets, call initRos().
   * \param[in] log The logger to use for message logging.
   */
  explicit TimeJumpResettable(const ::cras::LogHelperPtr& log);
  ~TimeJumpResettable() override;

  /**
   * \brief Check if ROS time has not jumped back. If it did, call reset().
   * \note This function should be called periodically. Call startAutoCheckTimeJump() to run a timer that will do these
   *       periodic calls automatically. Or you can call this function from your callbacks.
   */
  void checkTimeJump();

  /**
   * \brief Check if ROS time has not jumped back. If it did, call reset().
   * \param[in] now The timestamp that should be interpreted as current time.
   * \note This function should be called periodically. Call startAutoCheckTimeJump() to run a timer that will do these
   *       periodic calls automatically. Or you can call this function from your callbacks.
   */
  virtual void checkTimeJump(const ::ros::Time& now);

  /**
   * \brief Initialize the ROS part of the interface - subscriber to /reset and ~reset topics, read time jump limits.
   * \param[in] pnh The (private) node handle to setup the interface for.
   */
  void initRos(const ::ros::NodeHandle& pnh) override;

  /**
   * \brief Start a background 1 Hz timer that automatically checks for time jumps.
   */
  void startAutoCheckTimeJump();

  /**
   * \brief Start a background timer that automatically checks for time jumps.
   * \param[in] rate Checking rate
   */
  virtual void startAutoCheckTimeJump(const ::ros::WallRate& rate);

  /**
   * \brief Stop the timer that automatically checks for time jumps.
   */
  virtual void stopAutoCheckTimeJump();

private:
  ::std::unique_ptr<::cras::TimeJumpResettablePrivate> data;  //!< \brief Private implementation (PIMPL) data.
};

}
