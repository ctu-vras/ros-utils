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

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>

#include <cras_cpp_common/param_utils.hpp>
#include <cras_cpp_common/qos.hpp>
#include <cras_topic_tools/count_messages.hpp>
#include <cras_topic_tools/generic_subscription.hpp>

namespace cras {

void CountMessagesComponent::cb(const std::shared_ptr<const rclcpp::SerializedMessage>& message) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->count_++;
  this->count_since_last_report_++;
  this->bytes_ += message->size();
  if (this->use_params_) {
    this->set_parameter(rclcpp::Parameter("count", static_cast<int>(this->count_)));
    this->set_parameter(rclcpp::Parameter("bytes", static_cast<int>(this->bytes_)));
  }
}

void CountMessagesComponent::reportCb() {
  if (!this->last_report_stamp_.has_value()) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  RCLCPP_INFO(
    this->get_logger(), "Received %zu messages in %f s (total %zu messages, %zu bytes).",
    this->count_since_last_report_, (this->get_clock()->now() - *this->last_report_stamp_).seconds(), this->count_,
    this->bytes_);
  this->last_report_stamp_ = this->get_clock()->now();
  this->count_since_last_report_ = 0;
}

void CountMessagesComponent::discoverTopicAndSubscribe() {
  if (this->sub_ != nullptr) {
    this->discovery_timer_->cancel();
    return;
  }

  std::string type;
  for (const auto& [topic, types] : this->get_topic_names_and_types()) {
    if (!types.empty() && topic == this->resolved_topic_) {
      type = types[0];
    }
  }
  if (type.empty()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Waiting for topic %s", this->resolved_topic_.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Subscribing to topic %s [%s]", this->resolved_topic_.c_str(), type.c_str());

  rclcpp::SubscriptionOptions opts;
  opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  if (this->intraprocess_comms_) {
    opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  }
  if (this->topic_stats_) {
    opts.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
  }

  this->sub_ = ::cras::create_generic_subscription(
    *this, this->topic_, type, this->qos_profile_, std::bind(&CountMessagesComponent::cb, this, std::placeholders::_1),
    opts);

  this->last_report_stamp_ = this->get_clock()->now();

  this->discovery_timer_->cancel();
}

void CountMessagesComponent::discoverResetTopicAndSubscribe() {
  if (this->reset_sub_ != nullptr) {
    this->reset_discovery_timer_->cancel();
    return;
  }

  std::string type;
  for (const auto& [topic, types] : this->get_topic_names_and_types()) {
    if (!types.empty() && topic == this->resolved_reset_topic_) {
      type = types[0];
    }
  }
  if (type.empty()) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to topic %s [%s]", this->resolved_reset_topic_.c_str(), type.c_str());

  this->reset_sub_ = this->create_generic_subscription(
    "reset", type, rclcpp::ServicesQoS(), std::bind(&CountMessagesComponent::resetCb, this, std::placeholders::_1));

  this->reset_discovery_timer_->cancel();
}

CountMessagesComponent::CountMessagesComponent(const ::rclcpp::NodeOptions& options)
    : rclcpp::Node("count_messages", options) {
  this->use_params_ = this->declare_parameter("use_parameters", this->use_params_);
  this->intraprocess_comms_ = this->declare_parameter("intraprocess_comms", this->intraprocess_comms_);
  this->topic_stats_ = this->declare_parameter("topic_statistics", this->topic_stats_);
  auto report_interval_double = this->declare_parameter("report_interval", 0.0);

  auto qos_profile_str = this->declare_parameter("qos_profile", "");
  auto qos_depth_param = maybeParam<int>(this->declare_parameter("qos_depth", rclcpp::PARAMETER_INTEGER));
  auto qos_history_param = maybeParam<std::string>(this->declare_parameter("qos_history", rclcpp::PARAMETER_STRING));
  auto qos_reliability_param = maybeParam<std::string>(
    this->declare_parameter("qos_reliability", rclcpp::PARAMETER_STRING));
  auto qos_durability_param = maybeParam<std::string>(
    this->declare_parameter("qos_durability", rclcpp::PARAMETER_STRING));
  auto qos_liveliness_param = maybeParam<std::string>(
    this->declare_parameter("qos_liveliness", rclcpp::PARAMETER_STRING));
  auto qos_liveliness_lease_duration_seconds_param =
    maybeParam<double>(this->declare_parameter("qos_liveliness_lease_duration_seconds", rclcpp::PARAMETER_DOUBLE));

  // Support direct command-line usage
  if (options.arguments().size() > 1) {
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
    for (const auto& arg : options.arguments()) {
      argv.push_back(arg.c_str());
    }
    const auto tmp_args = rclcpp::remove_ros_arguments(argv.size(), argv.data());
    argv.clear();
    for (const auto& arg : tmp_args) {
      argv.push_back(arg.c_str());
    }

    if (!argv.empty()) {
      const auto args = parser.parse(argv.size(), argv.data());

      if (args.count("help")) {
        throw std::invalid_argument(parser.help());
      }

      this->topic_ = args["topic"].as<std::string>();

      if (args.count("qos-profile") == 1 && !args["qos-profile"].as<std::string>().empty()) {
        qos_profile_str = args["qos-profile"].as<std::string>();
      }

      if (args.count("report") == 1) {
        report_interval_double = args["report"].as<double>();
      }
    }
  }

  const auto report_interval = std::chrono::round<std::chrono::nanoseconds>(
    std::chrono::duration<float>(report_interval_double));

  const auto default_qos = rclcpp::BestAvailableQoS().keep_last(1000);
  this->qos_profile_ = qos_profile_str.empty() ? default_qos : parseQoSPreset(qos_profile_str);
  cras::configureQoSProfile(
    this->qos_profile_, qos_depth_param, qos_history_param, qos_reliability_param, qos_durability_param,
    qos_liveliness_param, qos_liveliness_lease_duration_seconds_param);

  this->resolved_topic_ = this->get_node_base_interface()->resolve_topic_or_service_name(this->topic_, false);
  this->resolved_reset_topic_ = this->get_node_base_interface()->resolve_topic_or_service_name("reset", false);

  if (this->use_params_) {
    this->declare_parameter("bytes", 0);
    this->declare_parameter("count", 0);
  }

  this->discovery_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&CountMessagesComponent::discoverTopicAndSubscribe, this));

  this->reset_discovery_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&CountMessagesComponent::discoverResetTopicAndSubscribe, this));

  if (report_interval != std::chrono::seconds(0)) {
    this->report_timer_ = this->create_timer(report_interval, std::bind(&CountMessagesComponent::reportCb, this));
  }

  this->discoverTopicAndSubscribe();
  this->discoverResetTopicAndSubscribe();
}

void CountMessagesComponent::resetCb(const std::shared_ptr<const rclcpp::SerializedMessage>&) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->count_ = this->bytes_ = this->count_since_last_report_ = 0;
  if (this->use_params_) {
    this->set_parameter(rclcpp::Parameter("count", 0));
    this->set_parameter(rclcpp::Parameter("bytes", 0));
  }
}

}  // namespace cras

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cras::CountMessagesComponent)
