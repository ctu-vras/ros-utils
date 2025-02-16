#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief TF message filter. This is basically tf2_ros::MessageFilter, but with support for proper logging.
 * \author Josh Faust, Martin Pecka
 */

// This is mostly a copy of https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/include/tf2_ros/message_filter.h
// at commit 7f9946e2bec491c09da80d3774e4c19704c84c90 . The file was edited to support logging via cras::Logger
// and to fit within the formatting requirements of this package.

/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Josh Faust */

#include <algorithm>
#include <list>
#include <map>
#include <string>
#include <vector>

#include <boost/function.hpp>
#include <boost/bind/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <cras_cpp_common/log_utils.h>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>
#include <ros/node_handle.h>
#include <ros/callback_queue_interface.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/message_filter.h>

#define CRAS_TF_MESSAGEFILTER_DEBUG(fmt, ...) \
  CRAS_DEBUG_NAMED("message_filter", (std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

#define CRAS_TF_MESSAGEFILTER_WARN(fmt, ...) \
  CRAS_WARN_NAMED("message_filter", (std::string("MessageFilter [target=%s]: ") + std::string(fmt)).c_str(), \
    getTargetFramesString().c_str(), __VA_ARGS__)

namespace cras
{

/**
 * \brief Follows the patterns set by the message_filters package to implement a filter which only passes messages
 *        through once there is transform data available
 *
 * The callbacks used in this class are of the same form as those used by roscpp's message callbacks.
 *
 * MessageFilter is templated on a message type.
 *
 * \section example_usage Example Usage
 *
 * If you want to hook a MessageFilter into a ROS topic:
\verbatim
message_filters::Subscriber<MessageType> sub(node_handle_, "topic", 10);
cras::TFMessageFilter<MessageType> tf_filter(log, sub, tf_buffer_, "/map", 10, 0);
tf_filter.registerCallback(&MyClass::myCallback, this);
\endverbatim
 */
template<class M, typename ::std::enable_if_t<ros::message_traits::HasHeader<M>::value>* = nullptr>
class TfMessageFilter :
  public tf2_ros::MessageFilterBase, public message_filters::SimpleFilter<M>, public cras::HasLogger
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef ros::MessageEvent<M const> MEvent;
  typedef boost::function<void(const MConstPtr&, tf2_ros::FilterFailureReason)> FailureCallback;
  typedef boost::signals2::signal<void(const MConstPtr&, tf2_ros::FilterFailureReason)> FailureSignal;

  /**
   * \brief Constructor
   *
   * \param log The logger.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty
   *                     string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param nh The NodeHandle whose callback queue we should add callbacks to
   */
  TfMessageFilter(const cras::LogHelperPtr& log, tf2::BufferCore& bc, const std::string& target_frame,
    uint32_t queue_size, const ros::NodeHandle& nh)
  : HasLogger(log), bc_(bc), queue_size_(queue_size), callback_queue_(nh.getCallbackQueue())
  {
    init();

    setTargetFrame(target_frame);
  }

  /**
   * \brief Constructor
   *
   * \param log The logger.
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty
   *                     string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param nh The NodeHandle whose callback queue we should add callbacks to
   */
  template<class F>
  TfMessageFilter(const cras::LogHelperPtr& log, F& f, tf2::BufferCore& bc, const std::string& target_frame,
    uint32_t queue_size, const ros::NodeHandle& nh)
  : HasLogger(log), bc_(bc), queue_size_(queue_size), callback_queue_(nh.getCallbackQueue())
  {
    init();

    setTargetFrame(target_frame);

    connectInput(f);
  }

  /**
   * \brief Constructor
   *
   * \param log The logger.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty
   *                     string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param cbqueue The callback queue to add callbacks to.  If NULL, callbacks will happen from whatever thread either
   *    a) add() is called, which will generally be when the previous filter in the chain outputs a message, or
   *    b) tf2::BufferCore::setTransform() is called
   */
  TfMessageFilter(const cras::LogHelperPtr& log, tf2::BufferCore& bc, const std::string& target_frame,
    uint32_t queue_size, ros::CallbackQueueInterface* cbqueue)
  : HasLogger(log), bc_(bc), queue_size_(queue_size), callback_queue_(cbqueue)
  {
    init();

    setTargetFrame(target_frame);
  }

  /**
   * \brief Constructor
   *
   * \param log The logger.
   * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
   * \param bc The tf2::BufferCore this filter should use
   * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty
   *                     string here and use the setTargetFrames() function.
   * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
   * \param cbqueue The callback queue to add callbacks to.  If NULL, callbacks will happen from whatever thread either
   *    a) add() is called, which will generally be when the previous filter in the chain outputs a message, or
   *    b) tf2::BufferCore::setTransform() is called
   */
  template<class F>
  TfMessageFilter(const cras::LogHelperPtr& log, F& f, tf2::BufferCore& bc, const std::string& target_frame,
    uint32_t queue_size, ros::CallbackQueueInterface* cbqueue)
  : cras::HasLogger(log), bc_(bc), queue_size_(queue_size), callback_queue_(cbqueue)
  {
    init();

    setTargetFrame(target_frame);

    connectInput(f);
  }

  /**
   * \brief Connect this filter's input to another filter's output.
   *
   * If this filter is already connected, disconnects first.
   */
  template<class F>
  void connectInput(F& f)
  {
    message_connection_.disconnect();
    message_connection_ = f.registerCallback(&TfMessageFilter::incomingMessage, this);
  }

  /**
   * \brief Destructor
   */
  ~TfMessageFilter() override
  {
    message_connection_.disconnect();

    TfMessageFilter::clear();

    CRAS_TF_MESSAGEFILTER_DEBUG(
      "Successful Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, "
      "Messages received: %llu, Total dropped: %llu",
      successful_transform_count_, failed_out_the_back_count_, transform_message_count_, incoming_message_count_,
      dropped_message_count_);
  }

  /**
   * \brief Set the frame you need to be able to transform to before getting a message callback
   */
  void setTargetFrame(const std::string& target_frame) override
  {
    V_string frames;
    frames.push_back(target_frame);
    setTargetFrames(frames);
  }

  /**
   * \brief Set the frames you need to be able to transform to before getting a message callback
   */
  void setTargetFrames(const V_string& target_frames) override
  {
    boost::mutex::scoped_lock frames_lock(target_frames_mutex_);

    target_frames_.resize(target_frames.size());
    std::transform(target_frames.begin(), target_frames.end(), target_frames_.begin(), this->stripSlash);
    expected_success_count_ = target_frames_.size() * (time_tolerance_.isZero() ? 1 : 2);

    std::stringstream ss;
    for (V_string::iterator it = target_frames_.begin(); it != target_frames_.end(); ++it)
    {
      ss << *it << " ";
    }
    target_frames_string_ = ss.str();
  }

  /**
   * \brief Get the target frames as a string for debugging
   */
  std::string getTargetFramesString()
  {
    boost::mutex::scoped_lock lock(target_frames_mutex_);
    return target_frames_string_;
  };

  /**
   * \brief Set the required tolerance for the notifier to return true
   */
  void setTolerance(const ros::Duration& tolerance) override
  {
    boost::mutex::scoped_lock lock(target_frames_mutex_);
    time_tolerance_ = tolerance;
    expected_success_count_ = target_frames_.size() * (time_tolerance_.isZero() ? 1 : 2);
  }

  /**
   * \brief Clear any messages currently in the queue
   */
  void clear() override
  {
    boost::unique_lock< boost::shared_mutex > unique_lock(messages_mutex_);

    CRAS_TF_MESSAGEFILTER_DEBUG("%s", "Cleared");

    bc_.removeTransformableCallback(callback_handle_);
    callback_handle_ = bc_.addTransformableCallback(boost::bind(&TfMessageFilter::transformable, this,
      boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3, boost::placeholders::_4,
      boost::placeholders::_5));

    messages_.clear();
    message_count_ = 0;

    // remove pending callbacks in callback queue as well
    if (callback_queue_)
      callback_queue_->removeByID((uint64_t)this);

    warned_about_empty_frame_id_ = false;
  }

  void add(const MEvent& evt)
  {
    if (target_frames_.empty())
    {
      return;
    }

    namespace mt = ros::message_traits;
    const MConstPtr& message = evt.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    ros::Time stamp = mt::TimeStamp<M>::value(*message);

    if (frame_id.empty())
    {
      messageDropped(evt, tf2_ros::filter_failure_reasons::EmptyFrameID);
      return;
    }

    // iterate through the target frames and add requests for each of them
    MessageInfo info;
    info.handles.reserve(expected_success_count_);
    {
      V_string target_frames_copy;
      // Copy target_frames_ to avoid deadlock from #79
      {
        boost::mutex::scoped_lock frames_lock(target_frames_mutex_);
        target_frames_copy = target_frames_;
      }

      V_string::iterator it = target_frames_copy.begin();
      V_string::iterator end = target_frames_copy.end();
      for (; it != end; ++it)
      {
        const std::string& target_frame = *it;
        tf2::TransformableRequestHandle handle =
          bc_.addTransformableRequest(callback_handle_, target_frame, frame_id, stamp);
        if (handle == 0xffffffffffffffffULL)  // never transformable
        {
          messageDropped(evt, tf2_ros::filter_failure_reasons::OutTheBack);
          return;
        }
        else if (handle == 0)
        {
          ++info.success_count;
        }
        else
        {
          info.handles.push_back(handle);
        }

        if (!time_tolerance_.isZero())
        {
          handle = bc_.addTransformableRequest(callback_handle_, target_frame, frame_id, stamp + time_tolerance_);
          if (handle == 0xffffffffffffffffULL)  // never transformable
          {
            messageDropped(evt, tf2_ros::filter_failure_reasons::OutTheBack);
            return;
          }
          else if (handle == 0)
          {
            ++info.success_count;
          }
          else
          {
            info.handles.push_back(handle);
          }
        }
      }
    }


    // We can transform already
    if (info.success_count == expected_success_count_)
    {
      messageReady(evt);
    }
    else
    {
      boost::unique_lock< boost::shared_mutex > unique_lock(messages_mutex_);
      // If this message is about to push us past our queue size, erase the oldest message
      if (queue_size_ != 0 && message_count_ + 1 > queue_size_)
      {
        ++dropped_message_count_;
        const auto& front = messages_.front();
        CRAS_TF_MESSAGEFILTER_DEBUG(
          "Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)",
          message_count_, (mt::FrameId<M>::value(*front.event.getMessage())).c_str(),
          mt::TimeStamp<M>::value(*front.event.getMessage()).toSec());

        V_TransformableRequestHandle::const_iterator it = front.handles.begin();
        V_TransformableRequestHandle::const_iterator end = front.handles.end();

        for (; it != end; ++it)
        {
          bc_.cancelTransformableRequest(*it);
        }

        messageDropped(front.event, tf2_ros::filter_failure_reasons::Unknown);
        messages_.pop_front();
         --message_count_;
      }

      // Add the message to our list
      info.event = evt;
      messages_.push_back(info);
      ++message_count_;
    }

    CRAS_TF_MESSAGEFILTER_DEBUG("Added message in frame %s at time %.3f, count now %d",
      frame_id.c_str(), stamp.toSec(), message_count_);

    ++incoming_message_count_;
  }

  /**
   * \brief Manually add a message into this filter.
   * \note If the message (or any other messages in the queue) are immediately transformable this will immediately call
   *       through to the output callback, possibly
   * multiple times
   */
  void add(const MConstPtr& message)
  {
    boost::shared_ptr<std::map<std::string, std::string> > header(new std::map<std::string, std::string>);
    (*header)["callerid"] = "unknown";
    ros::WallTime n = ros::WallTime::now();
    ros::Time t(n.sec, n.nsec);
    add(MEvent(message, header, t));
  }

  /**
   * \brief Register a callback to be called when a message is about to be dropped
   * \param callback The callback to call
   */
  message_filters::Connection registerFailureCallback(const FailureCallback& callback)
  {
    boost::mutex::scoped_lock lock(failure_signal_mutex_);
    return message_filters::Connection(boost::bind(
      &TfMessageFilter::disconnectFailure, this, boost::placeholders::_1), failure_signal_.connect(callback));
  }

  virtual void setQueueSize(uint32_t new_queue_size )
  {
    queue_size_ = new_queue_size;
  }

  virtual uint32_t getQueueSize()
  {
    return queue_size_;
  }

private:
  void init()
  {
    message_count_ = 0;
    successful_transform_count_ = 0;
    failed_out_the_back_count_ = 0;
    transform_message_count_ = 0;
    incoming_message_count_ = 0;
    dropped_message_count_ = 0;
    time_tolerance_ = ros::Duration(0.0);
    warned_about_empty_frame_id_ = false;
    expected_success_count_ = 1;

    callback_handle_ = bc_.addTransformableCallback(boost::bind(
      &TfMessageFilter::transformable, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3,
      boost::placeholders::_4, boost::placeholders::_5));
  }

  void transformable(
    tf2::TransformableRequestHandle request_handle, const std::string& /* target_frame */,
    const std::string& /* source_frame */, ros::Time /* time */, tf2::TransformableResult result)
  {
    namespace mt = ros::message_traits;

    boost::upgrade_lock< boost::shared_mutex > lock(messages_mutex_);

    // find the message this request is associated with
    typename L_MessageInfo::iterator msg_it = messages_.begin();
    typename L_MessageInfo::iterator msg_end = messages_.end();
    for (; msg_it != msg_end; ++msg_it)
    {
      auto& info = *msg_it;
      V_TransformableRequestHandle::const_iterator handle_it =
        std::find(info.handles.begin(), info.handles.end(), request_handle);
      if (handle_it != info.handles.end())
      {
        // found msg_it
        ++info.success_count;
        break;
      }
    }

    if (msg_it == msg_end)
    {
      return;
    }

    const auto& info = *msg_it;
    if (info.success_count < expected_success_count_)
    {
      return;
    }

    bool can_transform = true;
    const MConstPtr& message = info.event.getMessage();
    std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
    ros::Time stamp = mt::TimeStamp<M>::value(*message);

    if (result == tf2::TransformAvailable)
    {
      boost::mutex::scoped_lock frames_lock(target_frames_mutex_);
      // make sure we can still perform all the necessary transforms
      typename V_string::iterator it = target_frames_.begin();
      typename V_string::iterator end = target_frames_.end();
      for (; it != end; ++it)
      {
        const std::string& target = *it;
        if (!bc_.canTransform(target, frame_id, stamp))
        {
          can_transform = false;
          break;
        }

        if (!time_tolerance_.isZero())
        {
          if (!bc_.canTransform(target, frame_id, stamp + time_tolerance_))
          {
            can_transform = false;
            break;
          }
        }
      }
    }
    else
    {
      can_transform = false;
    }

    // We will be mutating messages now, require unique lock
    boost::upgrade_to_unique_lock< boost::shared_mutex > uniqueLock(lock);
    if (can_transform)
    {
      CRAS_TF_MESSAGEFILTER_DEBUG("Message ready in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.toSec(), message_count_ - 1);

      ++successful_transform_count_;

      messageReady(info.event);
    }
    else
    {
      ++dropped_message_count_;

      CRAS_TF_MESSAGEFILTER_DEBUG("Discarding message in frame %s at time %.3f, count now %d",
        frame_id.c_str(), stamp.toSec(), message_count_ - 1);
      messageDropped(info.event, tf2_ros::filter_failure_reasons::Unknown);
    }

    messages_.erase(msg_it);
    --message_count_;
  }

  /**
   * \brief Callback that happens when we receive a message on the message topic
   */
  void incomingMessage(const ros::MessageEvent<M const>& evt)
  {
    add(evt);
  }

  void checkFailures()
  {
    if (next_failure_warning_.isZero())
    {
      next_failure_warning_ = ros::WallTime::now() + ros::WallDuration(15);
    }

    if (ros::WallTime::now() >= next_failure_warning_)
    {
      if (incoming_message_count_ - message_count_ == 0)
      {
        return;
      }

      double dropped_pct =
        static_cast<double>(dropped_message_count_) / static_cast<double>(incoming_message_count_ - message_count_);
      if (dropped_pct > 0.95)
      {
        CRAS_TF_MESSAGEFILTER_WARN(
          "Dropped %.2f%% of messages so far. Please turn the [%s.message_notifier] rosconsole logger to DEBUG for more"
          " information.", dropped_pct*100, ROSCONSOLE_DEFAULT_NAME);
        next_failure_warning_ = ros::WallTime::now() + ros::WallDuration(60);

        if (static_cast<double>(failed_out_the_back_count_) / static_cast<double>(dropped_message_count_) > 0.5)
        {
          CRAS_TF_MESSAGEFILTER_WARN("  The majority of dropped messages were due to messages growing older than the "
            "TF cache time.  The last message's timestamp was: %f, and the last frame_id was: %s",
            last_out_the_back_stamp_.toSec(), last_out_the_back_frame_.c_str());
        }
      }
    }
  }

  struct CBQueueCallback : public ros::CallbackInterface
  {
    CBQueueCallback(TfMessageFilter* filter, const MEvent& event, bool success, tf2_ros::FilterFailureReason reason)
    : event_(event)
    , filter_(filter)
    , reason_(reason)
    , success_(success)
    {}


    virtual CallResult call()
    {
      if (success_)
      {
        filter_->signalMessage(event_);
      }
      else
      {
        filter_->signalFailure(event_, reason_);
      }

      return Success;
    }

  private:
    MEvent event_;
    TfMessageFilter* filter_;
    tf2_ros::FilterFailureReason reason_;
    bool success_;
  };

  void messageDropped(const MEvent& evt, tf2_ros::FilterFailureReason reason)
  {
    if (callback_queue_)
    {
      ros::CallbackInterfacePtr cb(new CBQueueCallback(this, evt, false, reason));
      callback_queue_->addCallback(cb, (uint64_t)this);
    }
    else
    {
      signalFailure(evt, reason);
    }
  }

  void messageReady(const MEvent& evt)
  {
    if (callback_queue_)
    {
      ros::CallbackInterfacePtr cb(new CBQueueCallback(this, evt, true, tf2_ros::filter_failure_reasons::Unknown));
      callback_queue_->addCallback(cb, (uint64_t)this);
    }
    else
    {
      this->signalMessage(evt);
    }
  }

  void disconnectFailure(const message_filters::Connection& c)
  {
    boost::mutex::scoped_lock lock(failure_signal_mutex_);
    c.getBoostConnection().disconnect();
  }

  void signalFailure(const MEvent& evt, tf2_ros::FilterFailureReason reason)
  {
    boost::mutex::scoped_lock lock(failure_signal_mutex_);
    failure_signal_(evt.getMessage(), reason);
  }

  static
  std::string stripSlash(const std::string& in)
  {
    if ( !in.empty() && (in[0] == '/'))
    {
      std::string out = in;
      out.erase(0, 1);
      return out;
    }
    return in;
  }

  tf2::BufferCore& bc_;  ///< The Transformer used to determine if transformation data is available
  V_string target_frames_;  ///< The frames we need to be able to transform to before a message is ready
  std::string target_frames_string_;
  boost::mutex target_frames_mutex_;  ///< A mutex to protect access to target_frames_ and target_frames_string_.
  uint32_t queue_size_;  ///< The maximum number of messages we queue up
  tf2::TransformableCallbackHandle callback_handle_;

  typedef std::vector<tf2::TransformableRequestHandle> V_TransformableRequestHandle;
  struct MessageInfo
  {
    MessageInfo()
    : success_count(0)
    {}

    MEvent event;
    V_TransformableRequestHandle handles;
    uint32_t success_count;
  };
  typedef std::list<MessageInfo> L_MessageInfo;
  L_MessageInfo messages_;
  uint32_t message_count_;  ///< The number of messages in the list.
  boost::shared_mutex messages_mutex_;  ///< The mutex used for locking message list operations
  uint32_t expected_success_count_;

  bool warned_about_empty_frame_id_;

  uint64_t successful_transform_count_;
  uint64_t failed_out_the_back_count_;
  uint64_t transform_message_count_;
  uint64_t incoming_message_count_;
  uint64_t dropped_message_count_;

  ros::Time last_out_the_back_stamp_;
  std::string last_out_the_back_frame_;

  ros::WallTime next_failure_warning_;

  //! Provide additional tolerance on time for messages which are stamped but can have associated duration
  ros::Duration time_tolerance_;

  message_filters::Connection message_connection_;

  FailureSignal failure_signal_;
  boost::mutex failure_signal_mutex_;

  ros::CallbackQueueInterface* callback_queue_;
};

}
