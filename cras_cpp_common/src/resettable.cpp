// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Interface for resettable nodes and nodelets. Automatic reset on time jumps.
 * \author Martin Pecka
 */

#include <memory>

#include <ros/duration.h>
#include <ros/message_event.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <cras_cpp_common/param_utils/get_param_adapters/node_handle.hpp>
#include <cras_cpp_common/time_utils.hpp>

#include <cras_cpp_common/resettable.h>

namespace cras
{

struct ResettablePrivate
{
  Resettable* resettable {nullptr};  //!< \brief Pointer to the public Resettable instance on which reset() is called.
  cras::LogHelperPtr log {nullptr};  //!< \brief Logger instance.
  bool rosInitialized {false};  //!< \brief Whether rosInit() has been called.
  ros::NodeHandle nh;  //!< \brief The node handle for which rosInit() has been called.
  ros::Subscriber resetSub;  //!< \brief Subscriber to /reset topic.
  ros::Subscriber resetPrivSub;  //!< \brief Subscriber to ~reset topic.

  /**
   * \brief Callback for receiving messages on /reset or ~reset topics. The contents of the messages are ignored.
   */
  void onResetMsg(const ros::MessageEvent<const topic_tools::ShapeShifter>&);
};

struct TimeJumpResettablePrivate
{
  TimeJumpResettable* resettable {nullptr};  //!< \brief Pointer to the public TimeJumpResettable instance on which
                                             //!<        reset() is called.
  cras::LogHelperPtr log {nullptr};  //!< \brief Logger instance.
  bool rosInitialized {false};  //!< \brief Whether rosInit() has been called.
  ros::NodeHandle nh;  //!< \brief The node handle for which rosInit() has been called.
  cras::optional<ros::Time> lastTimeStamp;  //!< \brief Last known ROS time.
  ros::SteadyTimer autoCheckTimer;  //!< \brief The timer that automatically checks for time jumps.
  ros::Duration jumpBackTolerance;  //!< \brief Tolerance for jumps back.
  ros::Duration jumpForwardTolerance;  //!< \brief Tolerance for jumps forward.
  bool resetOnTimeJumpBack {true};  //!< \brief Whether reset() should be called on ROS time jumps back.
  bool resetOnTimeJumpForward {false};  //!< \brief Whether reset() should be called on ROS time jumps forward.

  /**
   * \brief Automatic time jump checker callback.
   */
  void onResetTimer(const ros::SteadyTimerEvent&);
};

void TimeJumpResettablePrivate::onResetTimer(const ros::SteadyTimerEvent&)
{
  if (!ros::Time::isValid())
    return;

  this->resettable->checkTimeJump(ros::Time::now());
}

void ResettablePrivate::onResetMsg(const ros::MessageEvent<const topic_tools::ShapeShifter>&)
{
  this->resettable->reset();
}

Resettable::Resettable(const cras::LogHelperPtr& log) : data(std::make_unique<ResettablePrivate>())
{
  this->data->resettable = this;
  this->data->log = log;
}

Resettable::~Resettable() = default;

TimeJumpResettable::TimeJumpResettable(const cras::LogHelperPtr& log) : cras::Resettable(log),
  data(std::make_unique<TimeJumpResettablePrivate>())
{
  this->data->resettable = this;
  this->data->log = log;
}

TimeJumpResettable::~TimeJumpResettable() = default;

void Resettable::initRos(const ros::NodeHandle& pnh)
{
  if (this->data->rosInitialized)
    return;

  this->data->nh = pnh;

  this->data->resetSub = this->data->nh.subscribe("/reset", 1, &ResettablePrivate::onResetMsg, this->data.get());
  this->data->resetPrivSub = this->data->nh.subscribe("reset", 1, &ResettablePrivate::onResetMsg, this->data.get());

  this->data->rosInitialized = true;
}

void TimeJumpResettable::initRos(const ros::NodeHandle& pnh)
{
  Resettable::initRos(pnh);

  if (this->data->rosInitialized)
    return;

  this->data->nh = pnh;

  const auto isWallTime = ros::Time::isValid() ? ros::Time::isSystemTime() : true;
  cras::BoundParamHelper params(this->data->log, std::make_shared<cras::NodeHandleGetParamAdapter>(this->data->nh));

  auto backTolerance = ros::Duration(isWallTime ? 3 : 0, 0);
  backTolerance = params.getParam("/jump_back_tolerance", backTolerance, "s", {.printMessages = false});
  this->data->jumpBackTolerance = params.getParam("jump_back_tolerance", backTolerance, "s", {.printMessages = false});

  auto forwardTolerance = isWallTime ? ros::Duration::MAX : ros::Duration(10, 0);
  forwardTolerance = params.getParam("/jump_forward_tolerance", forwardTolerance, "s", {.printMessages = false});
  this->data->jumpForwardTolerance = params.getParam(
    "jump_forward_tolerance", forwardTolerance, "s", {.printMessages = false});

  this->data->resetOnTimeJumpBack = params.getParam("reset_on_time_jump_back", true, "", {.printMessages = false});
  this->data->resetOnTimeJumpForward = params.getParam(
    "reset_on_time_jump_forward", !isWallTime, "", {.printMessages = false});

  this->data->rosInitialized = true;
}

void TimeJumpResettable::checkTimeJump()
{
  this->checkTimeJump(::ros::Time::now());
}

void TimeJumpResettable::checkTimeJump(const ros::Time& now)
{
  if ((!this->data->resetOnTimeJumpBack && !this->data->resetOnTimeJumpForward) || !ros::Time::isValid())
    return;

  if (now < ros::Time::MIN)
    return;

  if (this->data->lastTimeStamp.has_value())
  {
    const auto getCrasLogger = [this]{return this->data->log;};
    // Do not invert the following comparisons so that they would contain subtraction - in such a case, the
    // resulting Time instance could go into negative numbers and that is not allowed.
    if (this->data->resetOnTimeJumpBack &&
      *this->data->lastTimeStamp > cras::saturateAdd(now, this->data->jumpBackTolerance))
    {
      CRAS_INFO("ROS time jumped back, resetting.");
      this->reset();
    }
    if (this->data->resetOnTimeJumpForward &&
      cras::saturateAdd(*this->data->lastTimeStamp, this->data->jumpForwardTolerance) < now)
    {
      CRAS_INFO("ROS time jumped forward, resetting.");
      this->reset();
    }
  }

  this->data->lastTimeStamp = now;
}

void TimeJumpResettable::startAutoCheckTimeJump()
{
  this->startAutoCheckTimeJump(ros::WallRate(ros::Duration(1, 0)));
}

void TimeJumpResettable::startAutoCheckTimeJump(const ros::WallRate& rate)
{
  if (!this->data->rosInitialized)
  {
    const auto getCrasLogger = [this]{return this->data->log;};
    CRAS_ERROR("Calling startAutoCheckTimeJump() before initRos()!");
    return;
  }
  this->data->autoCheckTimer = this->data->nh.createSteadyTimer(
    rate.expectedCycleTime(), &TimeJumpResettablePrivate::onResetTimer, this->data.get());
}

void TimeJumpResettable::stopAutoCheckTimeJump()
{
  if (this->data->autoCheckTimer)
    this->data->autoCheckTimer.stop();
}

}
