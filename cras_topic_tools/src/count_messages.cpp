// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Count messages on a topic.
 * \author Martin Pecka
 */

#include <functional>
#include <mutex>

#include <cxxopts.hpp>

#include <rclcpp/utilities.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/qos.hpp>
#include <cras_topic_tools/count_messages.hpp>
#include <cras_topic_tools/generic_subscription.hpp>

namespace cras
{

void CountMessagesComponent::cb(const std::shared_ptr<const rclcpp::SerializedMessage>& message)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->count++;
  this->countSinceLastReport++;
  this->bytes += message->size();
  if (this->useParams)
  {
    this->set_parameter(rclcpp::Parameter("count", static_cast<int>(this->count)));
    this->set_parameter(rclcpp::Parameter("bytes", static_cast<int>(this->bytes)));
  }
}

void CountMessagesComponent::reportCb()
{
  if (!this->lastReportStamp.has_value())
    return;

  std::lock_guard<std::mutex> lock(this->mutex);
  RCLCPP_INFO(this->get_logger(), "Received %zu messages in %f s (total %zu messages, %zu bytes).",
    this->countSinceLastReport, (this->get_clock()->now() - *this->lastReportStamp).seconds(),
    this->count, this->bytes);
  this->lastReportStamp = this->get_clock()->now();
  this->countSinceLastReport = 0;
}

void CountMessagesComponent::discoverTopicAndSubscribe()
{
  if (this->sub != nullptr)
  {
    this->discoveryTimer->cancel();
    return;
  }

  std::string type;
  for (const auto [topic, types] : this->get_topic_names_and_types())
  {
    if (!types.empty() && topic == this->resolvedTopic)
      type = types[0];
  }
  if (type.empty())
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Waiting for topic %s", this->resolvedTopic.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Subscribing to topic %s [%s]", this->resolvedTopic.c_str(), type.c_str());

  rclcpp::SubscriptionOptions opts;
  opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  if (this->intraprocessComms)
    opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  if (this->topicStats)
    opts.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

  this->sub = ::cras::create_generic_subscription(*this, this->topic, type, this->qosProfile,
    std::bind(&CountMessagesComponent::cb, this, std::placeholders::_1), opts);

  this->lastReportStamp = this->get_clock()->now();

  this->discoveryTimer->cancel();
}

void CountMessagesComponent::discoverResetTopicAndSubscribe()
{
  if (this->resetSub != nullptr)
  {
    this->resetDiscoveryTimer->cancel();
    return;
  }

  std::string type;
  for (const auto [topic, types] : this->get_topic_names_and_types())
  {
    if (!types.empty() && topic == this->resolvedResetTopic)
      type = types[0];
  }
  if (type.empty())
    return;

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to topic %s [%s]", this->resolvedResetTopic.c_str(), type.c_str());

  this->resetSub = this->create_generic_subscription(
    "reset", type, rclcpp::ServicesQoS(), std::bind(&CountMessagesComponent::resetCb, this, std::placeholders::_1));

  this->resetDiscoveryTimer->cancel();
}

CountMessagesComponent::CountMessagesComponent(const ::rclcpp::NodeOptions& options)
  : rclcpp::Node("count_messages", options)
{
  this->useParams = this->declare_parameter("use_parameters", this->useParams);
  this->intraprocessComms = this->declare_parameter("intraprocess_comms", this->intraprocessComms);
  this->topicStats = this->declare_parameter("topic_statistics", this->topicStats);
  auto reportIntervalDouble = this->declare_parameter("report_interval", 0.0);

  auto qosProfileStr = this->declare_parameter("qos_profile", "");
  auto qosDepthParam = maybeParam<int>(this->declare_parameter("qos_depth", rclcpp::PARAMETER_INTEGER));
  auto qosHistoryParam = maybeParam<std::string>(this->declare_parameter("qos_history", rclcpp::PARAMETER_STRING));
  auto qosReliabilityParam = maybeParam<std::string>(
    this->declare_parameter("qos_reliability", rclcpp::PARAMETER_STRING));
  auto qosDurabilityParam = maybeParam<std::string>(
    this->declare_parameter("qos_durability", rclcpp::PARAMETER_STRING));
  auto qosLivelinessParam = maybeParam<std::string>(
    this->declare_parameter("qos_liveliness", rclcpp::PARAMETER_STRING));
  auto qosLivelinessLeaseDurationSecondsParam =
    maybeParam<double>(this->declare_parameter("qos_liveliness_lease_duration_seconds", rclcpp::PARAMETER_DOUBLE));

  // Support direct command-line usage
  if (options.arguments().size() > 1)
  {
    cxxopts::Options parser("count_messages", "Count ROS messages");
    parser.add_options()
    ("topic", "Topic", cxxopts::value<std::string>()->default_value("input"))
    ("qos-profile,p", "QoS Profile", cxxopts::value<std::string>()->default_value(""))
    ("report,r", "Reporting interval", cxxopts::value<double>()->default_value("0")->implicit_value("1"))
    ("help,h", "Help");
    parser.parse_positional({"topic", "qos-profile"});
    parser.positional_help("[TOPIC [QOS-PROFILE]]");
    parser.show_positional_help();

    // Convert all args to argc/argv, remove ROS-specific args and convert to argc/argv again
    std::vector<const char*> argv;
    for (const auto& arg : options.arguments())
      argv.push_back(arg.c_str());
    const auto tmpArgs = rclcpp::remove_ros_arguments(argv.size(), argv.data());
    argv.clear();
    for (const auto& arg : tmpArgs)
      argv.push_back(arg.c_str());

    if (argv.size() > 0)
    {
      const auto args = parser.parse(argv.size(), argv.data());

      if (args.count("help"))
        throw std::invalid_argument(parser.help());

      this->topic = args["topic"].as<std::string>();

      if (args.count("qos-profile") == 1 && !args["qos-profile"].as<std::string>().empty())
        qosProfileStr = args["qos-profile"].as<std::string>();

      if (args.count("report") == 1)
        reportIntervalDouble = args["report"].as<double>();
    }
  }

  const auto reportInterval = std::chrono::round<std::chrono::nanoseconds>(
    std::chrono::duration<float>(reportIntervalDouble));

  const auto defaultQoS = rclcpp::BestAvailableQoS().keep_last(1000);
  this->qosProfile = qosProfileStr.empty() ? defaultQoS : parseQoSPreset(qosProfileStr);
  cras::configureQoSProfile(this->qosProfile, qosDepthParam, qosHistoryParam, qosReliabilityParam, qosDurabilityParam,
    qosLivelinessParam, qosLivelinessLeaseDurationSecondsParam);

  this->resolvedTopic = this->get_node_base_interface()->resolve_topic_or_service_name(this->topic, false);
  this->resolvedResetTopic = this->get_node_base_interface()->resolve_topic_or_service_name("reset", false);

  if (this->useParams)
  {
    this->declare_parameter("bytes", 0);
    this->declare_parameter("count", 0);
  }

  this->discoveryTimer = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&CountMessagesComponent::discoverTopicAndSubscribe, this));

  this->resetDiscoveryTimer = this->create_wall_timer(std::chrono::seconds(1),
    std::bind(&CountMessagesComponent::discoverResetTopicAndSubscribe, this));

  if (reportInterval != std::chrono::seconds(0))
    this->reportTimer = this->create_timer(reportInterval, std::bind(&CountMessagesComponent::reportCb, this));

  this->discoverTopicAndSubscribe();
  this->discoverResetTopicAndSubscribe();
}

void CountMessagesComponent::resetCb(const std::shared_ptr<const rclcpp::SerializedMessage>&)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->count = this->bytes = this->countSinceLastReport = 0;
  if (this->useParams)
  {
    this->set_parameter(rclcpp::Parameter("count", 0));
    this->set_parameter(rclcpp::Parameter("bytes", 0));
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras::CountMessagesComponent)
