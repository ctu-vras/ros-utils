#pragma once

/**
 * \file
 * \brief Diagnostic task for topic frequency and timestamp delay (combining FrequencyStatus and TimeStampStatus tasks).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <limits>
#include <memory>
#include <string>
#include <type_traits>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <ros/message_event.h>
#include <ros/message_traits.h>

#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/message_utils.hpp>
#include <cras_cpp_common/time_utils.hpp>

namespace cras
{

/**
 * \brief Diagnostic task for topic frequency and timestamp delay (combining FrequencyStatus and TimeStampStatus tasks).
 * \tparam Message Type of the message. If it contains a header field, the task will automatically check both frequency
 * and timestamp delay. Header-less messages will only have their frequency checked.
 */
template <typename Message, typename = ::std::enable_if_t<::ros::message_traits::IsMessage<Message>::value>>
class TopicStatus : public ::diagnostic_updater::CompositeDiagnosticTask
{
public:
  /**
   * \brief Create the diagnostic task for a header-less message (checking frequency only).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] params Parameters of the task.
   */
  template <typename M = Message, ::std::enable_if_t<!::ros::message_traits::HasHeader<M>::value, bool> = true>
  TopicStatus(const ::std::string& name, const ::cras::TopicStatusParam<Message>& params) :
    ::diagnostic_updater::CompositeDiagnosticTask(name), origParams(params)
  {
    this->freqTask = ::std::make_unique<::diagnostic_updater::FrequencyStatus>(this->origParams);
    this->addTask(this->freqTask.get());
  }

  /**
   * \brief Create the diagnostic task for a header-less message (checking frequency only).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] minRate Minimum allowed frequency.
   * \param[in] maxRate Maximum allowed frequency.
   * \param[in] rateTolerance Tolerance of the rate.
   * \param[in] rateWindowSize Number of updates during which the frequency is computed.
   */
  template <typename M = Message, ::std::enable_if_t<!::ros::message_traits::HasHeader<M>::value, bool> = true>
  explicit TopicStatus(const ::std::string& name,
    const double minRate = 0.0, const double maxRate = ::std::numeric_limits<double>::infinity(),
    const double rateTolerance = 0.1, const int rateWindowSize = 5) :
      TopicStatus(name, ::cras::TopicStatusParam<Message>(minRate, maxRate, rateTolerance, rateWindowSize))
  {
  }

  /**
   * \brief Create the diagnostic task for a message with header (checking frequency and timestamp delay).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] params Parameters of the task.
   */
  template <typename M = Message, ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value, bool> = true>
  TopicStatus(const ::std::string& name, const ::cras::TopicStatusParam<Message>& params) :
    ::diagnostic_updater::CompositeDiagnosticTask(name), origParams(params)
  {
    this->freqTask = ::std::make_unique<::diagnostic_updater::FrequencyStatus>(this->origParams);
    this->stampTask = ::std::make_unique<::diagnostic_updater::SlowTimeStampStatus>(this->origParams);

    this->addTask(this->freqTask.get());
    this->addTask(this->stampTask.get());
  }

  /**
   * \brief Create the diagnostic task for a message with header (checking frequency and timestamp delay).
   * \tparam M SFINAE only. Do not set explicitly.
   * \param[in] name Name of the diagnostic task.
   * \param[in] minRate Minimum allowed frequency.
   * \param[in] maxRate Maximum allowed frequency.
   * \param[in] rateTolerance Tolerance of the rate.
   * \param[in] rateWindowSize Number of updates during which the frequency is computed.
   * \param[in] minDelay Min acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \param[in] maxDelay Max acceptable delay (in s). It can be negative if timestamps in future are expected.
   */
  template <typename M = Message, ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value, bool> = true>
  explicit TopicStatus(const ::std::string& name,
    const double minRate = 0.0, const double maxRate = ::std::numeric_limits<double>::infinity(),
    const double rateTolerance = 0.1, const int rateWindowSize = 5,
    const double minDelay = -1.0, const double maxDelay = 5.0) :
      TopicStatus(name, ::cras::TopicStatusParam<Message>(
        minRate, maxRate, rateTolerance, rateWindowSize, minDelay, maxDelay))
  {
  }

  /**
   * \brief Create the diagnostic task checking frequency of messages and timestamp delay (if the message has header).
   * \param[in] name Name of the diagnostic task.
   * \param[in] params Parameters of the task.
   */
  TopicStatus(const ::std::string& name, const ::cras::SimpleTopicStatusParam<Message>& params) :
      TopicStatus(name, ::cras::TopicStatusParam<Message>(params))
  {
  }

  ~TopicStatus() override
  {
  }

  /**
   * \brief Record that a message has arrived now with the given timestamp.
   * \param[in] stamp Timestamp in the message header.
   */
  virtual void tick(const ::ros::Time& stamp)
  {
    this->freqTask->tick();
    if (this->stampTask != nullptr)
      this->stampTask->tick(stamp);
  }

  /**
   * \brief Record that a message has arrived.
   * \param[in] message The message that arrived.
   */
  virtual void tick(const Message& message)
  {
    this->freqTask->tick();
    if (this->stampTask != nullptr && ::ros::message_traits::HasHeader<::cras::BaseMessage<Message>>::value)
      this->stampTask->tick(*::ros::message_traits::TimeStamp<::cras::BaseMessage<Message>>::pointer(message));
  }

  /**
   * \brief Record that a message has arrived.
   * \param[in] message The message that arrived.
   */
  virtual void tick(const typename Message::Ptr& message)
  {
    this->tick(*message);
  }

  /**
   * \brief Record that a message has arrived.
   * \param[in] message The message that arrived.
   */
  virtual void tick(const typename Message::ConstPtr& message)
  {
    this->tick(*message);
  }

  /**
   * \brief Record that a message has arrived.
   * \param[in] event The message event describing the message that arrived.
   */
  virtual void tick(const ::ros::MessageEvent<Message>& event)
  {
    this->tick(*event.getConstMessage());
  }

  /**
   * \brief Get the expected/average rate. If min and max are the same, their value will be returned. If min rate is
   * non-positive, the max rate is returned. Otherwise, if max rate is infinite, the min rate will be returned. If min
   * is positive and max is finite, their arithmetic mean is returned.
   * \return The expected rate (in Hz).
   */
  ::ros::Rate getExpectedRate() const
  {
    return ::cras::safeRate(this->origParams.getExpectedRate());
  }

  /**
   * \brief Minimum allowed frequency.
   * \return The frequency (in Hz).
   */
  ::ros::Rate getMinRate() const
  {
    return ::cras::safeRate(*this->origParams.min_freq_);
  }

  /**
   * \brief Maximum allowed frequency.
   * \return The frequency (in Hz).
   */
  ::ros::Rate getMaxRate() const
  {
    return ::cras::safeRate(*this->origParams.max_freq_);
  }

  /**
   * \brief Tolerance of frequency.
   * \return The tolerance (0.0 means exact match of the frequency bounds).
   */
  double getRateTolerance() const
  {
    return this->origParams.tolerance_;
  }

  /**
   * \brief Number of updates during which the frequency is computed.
   * \return The window size.
   */
  int getRateWindowSize() const
  {
    return this->origParams.window_size_;
  }

  /**
   * \brief Min acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \tparam M SFINAE only. Do not set explicitly.
   * \return The minimum delay.
   */
  template <typename M = Message, typename = ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value>>
  ::ros::Duration getMinDelay() const
  {
    return ::ros::Duration(this->origParams.min_acceptable_);
  }

  /**
   * \brief Max acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \tparam M SFINAE only. Do not set explicitly.
   * \return The maximum delay.
   */
  template <typename M = Message, typename = ::std::enable_if_t<::ros::message_traits::HasHeader<M>::value>>
  ::ros::Duration getMaxDelay() const
  {
    return ::ros::Duration(this->origParams.max_acceptable_);
  }

protected:
  //! \brief The frequency-checking diagnostic task. This will always be non-null.
  ::std::unique_ptr<::diagnostic_updater::FrequencyStatus> freqTask;

  //! \brief The delay-checking diagnostic task. It will be non-null only for messages with header.
  ::std::unique_ptr<::diagnostic_updater::SlowTimeStampStatus> stampTask;

  //! \brief The parameters via which this task has been configured.
  ::cras::TopicStatusParam<Message> origParams;
};

template <typename Message>
using TopicStatusPtr = ::std::shared_ptr<::cras::TopicStatus<Message>>;

}
