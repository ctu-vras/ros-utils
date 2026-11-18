// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 */

#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/any_subscription_callback.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>

// Topic statistics were removed in rclcpp 33.2.0
#if __has_include(<rclcpp/topic_statistics/subscription_topic_statistics.hpp>)
#define CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
#include <rclcpp/topic_statistics/subscription_topic_statistics.hpp>
#endif

namespace cras
{

class GenericSubscription : public ::rclcpp::GenericSubscription
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GenericSubscription)

  /**
   * \brief Constructor
   *
   * In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \sa cras::create_generic_subscription() for creating an instance of this class and adding it to the
   *     node_topic_interface.
   *
   * \param[in] node_base Pointer to parent node's NodeBaseInterface
   * \param[in] ts_lib Type support library, needs to correspond to topic_type
   * \param[in] topic_name Topic name
   * \param[in] topic_type Topic type
   * \param[in] qos QoS settings
   * \param[in] callback Callback for new messages of serialized form
   * \param[in] options Subscription options.
   * \param[in] subscription_topic_stats Optional pointer to a topic statistics subscription.
   * Not all subscription options are currently respected, the only relevant options for this
   * subscription are `event_callbacks`, `use_default_callbacks`, `ignore_local_publications`, `topic_statistics` and
   * `callback_group`.
   */
  template<typename AllocatorT = ::std::allocator<void>>
  GenericSubscription(::rclcpp::node_interfaces::NodeBaseInterface* node_base,
    const ::std::shared_ptr<::rcpputils::SharedLibrary> ts_lib, const ::std::string& topic_name,
    const ::std::string& topic_type, const ::rclcpp::QoS& qos,
    ::rclcpp::AnySubscriptionCallback<::rclcpp::SerializedMessage, AllocatorT> callback,
    const ::rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
    , ::std::shared_ptr<::rclcpp::topic_statistics::SubscriptionTopicStatistics> subscription_topic_stats = nullptr
#endif
    )
  : ::rclcpp::GenericSubscription(node_base, ts_lib, topic_name, topic_type, qos, callback, options),
    any_callback(callback)
  {
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
    if (subscription_topic_stats != nullptr)
      this->subscription_topic_statistics = ::std::move(subscription_topic_stats);
#endif

#ifndef TRACETOOLS_DISABLED
    this->any_callback.register_callback_for_tracing();
#endif
  }

  void handle_serialized_message(const ::std::shared_ptr<::rclcpp::SerializedMessage>& serialized_message,
    const ::rclcpp::MessageInfo& message_info) override
  {
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
    ::std::chrono::time_point<::std::chrono::system_clock> now;
    if (this->subscription_topic_statistics)
      now = ::std::chrono::system_clock::now();
#endif

    this->any_callback.dispatch(serialized_message, message_info);

#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
    if (this->subscription_topic_statistics)
    {
      const auto nanos = ::std::chrono::time_point_cast<::std::chrono::nanoseconds>(now);
      const auto time = ::rclcpp::Time(nanos.time_since_epoch().count());
      this->subscription_topic_statistics->handle_message(message_info.get_rmw_message_info(), time);
    }
#endif
  }

protected:
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
  //! \brief Component which computes and publishes topic statistics for this subscriber.
  ::std::shared_ptr<::rclcpp::topic_statistics::SubscriptionTopicStatistics> subscription_topic_statistics {nullptr};
#endif

  //! \brief The callback to call when a message is received.
  ::rclcpp::AnySubscriptionCallback<::rclcpp::SerializedMessage, ::std::allocator<void>> any_callback;

private:
  RCLCPP_DISABLE_COPY(GenericSubscription)
};

/**
 * \brief Create and return a GenericSubscription which can handle topic statistics.
 *
 * The returned pointer will never be empty, but this function can throw various exceptions, for
 * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
 *
 * \param[in] parameters_interface NodeParametersInterface pointer used in parts of the setup.
 * \param[in] topics_interface NodeTopicsInterface pointer used in parts of the setup.
 * \param[in] topic_name Topic name
 * \param[in] topic_type Topic type
 * \param[in] qos QoS settings
 * \param[in] callback Callback for new messages of serialized form
 * \param[in] options Publisher options.
 * Not all publisher options are currently respected, the only relevant options for this
 * publisher are `event_callbacks`, `use_default_callbacks`, `topic_statistics` and `%callback_group`.
 */
template<typename CallbackT, typename AllocatorT = ::std::allocator<void>>
::std::shared_ptr<::cras::GenericSubscription> create_generic_subscription(
  ::rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  ::rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface, const ::std::string& topic_name,
  const ::std::string& topic_type, const ::rclcpp::QoS& qos, CallbackT&& callback,
  const ::rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options = {})
{
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
  ::std::shared_ptr<::rclcpp::topic_statistics::SubscriptionTopicStatistics> subscription_topic_stats {nullptr};
  if (::rclcpp::detail::resolve_enable_topic_statistics(options, *topics_interface->get_node_base_interface()))
  {
    if (options.topic_stats_options.publish_period <= ::std::chrono::milliseconds(0))
    {
      throw ::std::invalid_argument(
        "topic_stats_options.publish_period must be greater than 0, specified value of " +
        ::std::to_string(options.topic_stats_options.publish_period.count()) + " ms");
    }

    auto publisher = ::rclcpp::detail::create_publisher<::statistics_msgs::msg::MetricsMessage>(
      parameters_interface, topics_interface, options.topic_stats_options.publish_topic,
      options.topic_stats_options.qos);

    subscription_topic_stats = ::std::make_shared<::rclcpp::topic_statistics::SubscriptionTopicStatistics>(
      topics_interface->get_node_base_interface()->get_name(), publisher);

    ::std::weak_ptr<::rclcpp::topic_statistics::SubscriptionTopicStatistics> weak_subscription_topic_stats(
      subscription_topic_stats);
    const auto sub_call_back = [weak_subscription_topic_stats]
    {
      auto subscription_topic_stats = weak_subscription_topic_stats.lock();
      if (subscription_topic_stats)
        subscription_topic_stats->publish_message_and_reset_measurements();
    };

    const auto node_timer_interface = topics_interface->get_node_timers_interface();

    const auto timer = ::rclcpp::create_wall_timer(
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(options.topic_stats_options.publish_period),
      sub_call_back, options.callback_group, topics_interface->get_node_base_interface(), node_timer_interface);

    subscription_topic_stats->set_publisher_timer(timer);
  }
#endif

  auto ts_lib = ::rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  const auto allocator = options.get_allocator();

  ::rclcpp::AnySubscriptionCallback<::rclcpp::SerializedMessage, AllocatorT>
  any_subscription_callback(*allocator);
  any_subscription_callback.set(::std::forward<CallbackT>(callback));

  auto subscription = ::std::make_shared<::cras::GenericSubscription>(topics_interface->get_node_base_interface(),
    ::std::move(ts_lib), topic_name, topic_type, qos, any_subscription_callback, options
#ifdef CRAS_HAS_SUBSCRIPTION_TOPIC_STATISTICS
    , subscription_topic_stats
#endif
    );

  topics_interface->add_subscription(subscription, options.callback_group);

  return subscription;
}

/**
 * \brief Create and return a GenericSubscription which can handle topic statistics.
 *
 * The returned pointer will never be empty, but this function can throw various exceptions, for
 * instance when the message's package can not be found on the AMENT_PREFIX_PATH.
 *
 * \param[in] node The node to create subscription for.
 * \param[in] topic_name Topic name (relative to node namespace).
 * \param[in] topic_type Topic type
 * \param[in] qos QoS settings
 * \param[in] callback Callback for new messages of serialized form
 * \param[in] options Publisher options.
 * Not all publisher options are currently respected, the only relevant options for this
 * publisher are `event_callbacks`, `use_default_callbacks`, `topic_statistics` and `%callback_group`.
 */
template<typename CallbackT, typename AllocatorT = ::std::allocator<void>>
::std::shared_ptr<::cras::GenericSubscription> create_generic_subscription(
  ::rclcpp::Node& node, const ::std::string& topic_name, const ::std::string& topic_type,
  const ::rclcpp::QoS& qos, CallbackT&& callback,
  const ::rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options = {})
{
  const auto topic = ::rclcpp::extend_name_with_sub_namespace(topic_name, node.get_sub_namespace());
  return create_generic_subscription(node.get_node_parameters_interface(), node.get_node_topics_interface(),
    topic, topic_type, qos, callback, options);
}

}