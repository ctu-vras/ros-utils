/**
 * \file
 * \brief TF buffer whose functions with timeout can be interrupted.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>

#include <cras_cpp_common/tf2_utils/interruptible_buffer.h>

namespace cras
{

/**
 * \brief Append timeout info to the given error string if it is not null.
 * \param[in,out] errstr The possibly null error string.
 * \param[in] startTime Time when a TF lookup started.
 * \param[in] timeout The max duration of the TF lookup which has been exceeded.
 */
void conditionallyAppendTimeoutInfo(std::string* errstr, const ros::Time& startTime, const ros::Duration& timeout)
{
  if (!errstr)
    return;
  std::stringstream ss;
  const auto duration = ros::Time::now() - startTime;
  ss << " canTransform returned after " << duration.toSec() << " s, timeout was " << timeout.toSec() << " s.";
  *errstr += ss.str();
}

InterruptibleTFBuffer::InterruptibleTFBuffer(const ::ros::Duration& cacheTime) : tf2_ros::Buffer(cacheTime)
{
}

InterruptibleTFBuffer::InterruptibleTFBuffer(const std::shared_ptr<tf2::BufferCore>& parentBuffer) :
  tf2_ros::Buffer(parentBuffer ? parentBuffer->getCacheLength() : ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)),
  parentBuffer(parentBuffer),
  interruptibleParentBuffer(
    parentBuffer ? std::dynamic_pointer_cast<cras::InterruptibleTFBuffer>(parentBuffer) : nullptr)
{
}

InterruptibleTFBuffer::~InterruptibleTFBuffer()
{
  // When the object is being destroyed, we need to make sure all pending canTransforms() finish. They call ok() in the
  // wait loops. By calling requestStop() here we make sure the loops finish in the next iteration which should be in
  // a few milisecs.
  // Even though requestStop() is virtual, in destructor, the dynamic type of the class is no longer considered and the
  // method is called through the static type, which is InterruptibleTFBuffer (the same holds of the ok() calls done
  // in the canTransform() call because the virtual function tables of the descendant classes are already destroyed).
  this->requestStop();
  // Wait until the callbacks finish.
  this->callbackSemaphore.waitZero();
}

bool InterruptibleTFBuffer::ok() const
{
  if (!this->isOk)
    return false;

  // If the parent buffer is an interruptible buffer, first check if it hasn't been interrupted.
  if (this->interruptibleParentBuffer != nullptr && !this->interruptibleParentBuffer->ok())
    return false;

  return true;
}

void InterruptibleTFBuffer::requestStop()
{
  this->isOk = false;
  // Ignore any new callbacks in the semaphore.
  this->callbackSemaphore.disable();
}

#define CALLBACK_SEMAPHORE_GUARD \
  cras::SemaphoreGuard<cras::ReverseSemaphore> guard(this->callbackSemaphore); \
  if (!guard.acquired()) \
    return false;

bool InterruptibleTFBuffer::canTransform(const std::string& target_frame, const std::string& source_frame,
                                         const ros::Time& time, const ros::Duration timeout) const  // NOLINT
{
  CALLBACK_SEMAPHORE_GUARD
  return this->canTransform(target_frame, source_frame, time, timeout, nullptr);
}

bool InterruptibleTFBuffer::canTransform(const std::string& target_frame, const std::string& source_frame,
                                         const ros::Time& time, const ros::Duration timeout, std::string* errstr) const
{
  // Fast-track exit if we're shutting down.
  if (!this->isOk || !this->ok())
    return false;

  CALLBACK_SEMAPHORE_GUARD

  // Clear the errstr before populating it if it's valid.
  if (errstr)
    errstr->clear();

  const auto* buffer = (this->parentBuffer ? this->parentBuffer.get() : this);

  // Poll for transform if timeout is set.
  const auto startTime = ros::Time::now();
  const auto endTime = startTime + timeout;
  const auto sleepDuration = (std::max)(timeout * this->canTransformPollingScale, this->minPollingDuration);
  // Do not change the "now + 3 >= startTime" to "now >= startTime - 3", otherwise you could get into illegal
  // negative time values.
  while (
    ros::Time::now() < endTime &&
      !buffer->canTransform(target_frame, source_frame, time) &&
      (ros::Time::now() + ros::Duration(3)) >= startTime &&  // Don't wait when we detect a bag loop
      (ros::ok() || !ros::isInitialized()) &&  // Make sure we haven't been stopped
      this->ok()  // Make sure the buffer is not requested to stop
    )
  {
    this->sleep(sleepDuration);
  }

  if (!this->ok() || (ros::isInitialized() && !ros::ok()))
  {
    if (errstr != nullptr)
      *errstr = "Lookup has been interrupted.";
    return false;
  }

  if (ros::Time::now() + ros::Duration(3) < startTime)
  {
    if (errstr != nullptr)
      *errstr = "Time jumped backwards.";
    return false;
  }

  const auto retval = buffer->canTransform(target_frame, source_frame, time, errstr);
  if (!retval)
    conditionallyAppendTimeoutInfo(errstr, startTime, timeout);
  return retval;
}

bool InterruptibleTFBuffer::canTransform(
  const std::string& target_frame, const ros::Time& target_time,
  const std::string& source_frame, const ros::Time& source_time,
  const std::string& fixed_frame, const ros::Duration timeout) const  // NOLINT
{
  CALLBACK_SEMAPHORE_GUARD
  return this->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, nullptr);
}

bool InterruptibleTFBuffer::canTransform(
  const std::string& target_frame, const ros::Time& target_time,
  const std::string& source_frame, const ros::Time& source_time,
  const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr) const
{
  // Fast-track exit if we're shutting down.
  if (!this->isOk || !this->ok())
    return false;

  CALLBACK_SEMAPHORE_GUARD

  // Clear the errstr before populating it if it's valid.
  if (errstr)
    errstr->clear();

  const auto* buffer = (this->parentBuffer ? this->parentBuffer.get() : this);

  // Poll for transform if timeout is set.
  const auto startTime = ros::Time::now();
  const auto endTime = startTime + timeout;
  const auto sleepDuration = (std::max)(timeout * this->canTransformPollingScale, this->minPollingDuration);
  // Do not change the "now + 3 >= startTime" to "now >= startTime - 3", otherwise you could get into illegal
  // negative time values.
  while (
    ros::Time::now() < endTime &&
      !buffer->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame) &&
      (ros::Time::now() + ros::Duration(3)) >= startTime &&  // Don't wait when we detect a bag loop
      (ros::ok() || !ros::isInitialized()) &&  // Make sure we haven't been stopped
      this->ok()  // Make sure the buffer is not requested to stop
    )
  {
    this->sleep(sleepDuration);
  }

  if (!this->ok() || (ros::isInitialized() && !ros::ok()))
  {
    if (errstr != nullptr)
      *errstr = "Lookup has been interrupted.";
    return false;
  }

  if (ros::Time::now() + ros::Duration(3) < startTime)
  {
    if (errstr != nullptr)
      *errstr = "Time jumped backwards.";
    return false;
  }

  const auto retval = buffer->canTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame, errstr);
  if (!retval)
    conditionallyAppendTimeoutInfo(errstr, startTime, timeout);
  return retval;
}

geometry_msgs::TransformStamped InterruptibleTFBuffer::lookupTransform(
  const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
  const ros::Duration timeout) const
{
  cras::SemaphoreGuard<cras::ReverseSemaphore> guard(this->callbackSemaphore);
  if (!guard.acquired()) throw tf2::LookupException("Buffer is exiting");

  this->canTransform(target_frame, source_frame, time, timeout, nullptr);
  const auto* buffer = (this->parentBuffer ? this->parentBuffer.get() : this);
  return buffer->lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::TransformStamped InterruptibleTFBuffer::lookupTransform(
  const std::string& target_frame, const ros::Time& target_time,
  const std::string& source_frame, const ros::Time& source_time,
  const std::string& fixed_frame, const ros::Duration timeout) const
{
  cras::SemaphoreGuard<cras::ReverseSemaphore> guard(this->callbackSemaphore);
  if (!guard.acquired()) throw tf2::LookupException("Buffer is exiting");

  this->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, nullptr);
  const auto* buffer = (this->parentBuffer ? this->parentBuffer.get() : this);
  return buffer->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

bool InterruptibleTFBuffer::setCanTransformPollingScale(double scale)
{
  if (scale <= 0 || scale > 1)
    return false;
  this->canTransformPollingScale = scale;
  return true;
}

bool InterruptibleTFBuffer::setMinPollingDuration(const ros::Duration& duration)
{
  if (duration < ros::Duration(0))
    return false;
  this->minPollingDuration = duration;
  return true;
}

tf2::BufferCore& InterruptibleTFBuffer::getRawBuffer()
{
  return (this->parentBuffer ? *this->parentBuffer : *this);
}

const tf2::BufferCore& InterruptibleTFBuffer::getRawBuffer() const
{
  return (this->parentBuffer ? *static_cast<const tf2::BufferCore*>(this->parentBuffer.get()) : *this);
}

ros::Duration InterruptibleTFBuffer::getCacheLength()
{
  return this->getRawBuffer().getCacheLength();
}

void InterruptibleTFBuffer::clear()
{
  this->getRawBuffer().clear();
}

bool InterruptibleTFBuffer::setTransform(const geometry_msgs::TransformStamped& transform, const std::string& authority,
                                         const bool is_static)
{
  return this->getRawBuffer().setTransform(transform, authority, is_static);
}

geometry_msgs::TransformStamped InterruptibleTFBuffer::lookupTransform(const std::string& target_frame,
                                                                       const std::string& source_frame,
                                                                       const ros::Time& time) const
{
  return this->getRawBuffer().lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::TransformStamped InterruptibleTFBuffer::lookupTransform(const std::string& target_frame,
                                                                       const ros::Time& target_time,
                                                                       const std::string& source_frame,
                                                                       const ros::Time& source_time,
                                                                       const std::string& fixed_frame) const
{
  return this->getRawBuffer().lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
}

bool InterruptibleTFBuffer::canTransform(const std::string& target_frame, const std::string& source_frame,
                                         const ros::Time& time, std::string* error_msg) const
{
  return this->getRawBuffer().canTransform(target_frame, source_frame, time, error_msg);
}

bool InterruptibleTFBuffer::canTransform(const std::string& target_frame,
                                         const ros::Time& target_time,
                                         const std::string& source_frame,
                                         const ros::Time& source_time,
                                         const std::string& fixed_frame,
                                         std::string* error_msg) const
{
  return this->getRawBuffer().canTransform(
    target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
}

std::string InterruptibleTFBuffer::allFramesAsYAML(const double current_time) const
{
  return this->getRawBuffer().allFramesAsYAML(current_time);
}

std::string InterruptibleTFBuffer::allFramesAsYAML() const
{
  return this->getRawBuffer().allFramesAsYAML();
}

std::string InterruptibleTFBuffer::allFramesAsString() const
{
  return this->getRawBuffer().allFramesAsString();
}

}
