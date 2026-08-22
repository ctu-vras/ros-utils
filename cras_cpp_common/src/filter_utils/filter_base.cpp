// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Base for filters that eases getting parameters and more node interfaces.
 * \author Martin Pecka
 */


/* HACK HACK HACK */
/* We want to access private members of some Node implementations. */
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private public  // NOLINT
#include <rclcpp/node_interfaces/node_clock.hpp>
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/node_interfaces/node_topics.hpp>
#undef private
/* HACK END HACK */

#include <memory>
#include <string>
#include <vector>

#include <cras_cpp_common/filter_utils/filter_base.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

namespace cras {

using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  rclcpp::node_interfaces::NodeBaseInterface,
  rclcpp::node_interfaces::NodeClockInterface,
  rclcpp::node_interfaces::NodeLoggingInterface,
  rclcpp::node_interfaces::NodeParametersInterface,
  rclcpp::node_interfaces::NodeServicesInterface,
  rclcpp::node_interfaces::NodeTopicsInterface
>;

struct FilterNodeInterfaces::Impl {
  Impl(
    const std::string& name, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging_interface,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params_interface)
    : name_(name), params_(params_interface), logging_(logging_interface) {}

  std::string name_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr params_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_;

  volatile bool should_stop_ {false};

  RequiredInterfaces& getNodeInterfaces() {
    if (node_interfaces_ == nullptr) {
      initNodeInterfaces();
    }
    return *node_interfaces_;
  }

private:
  void initNodeInterfaces() {
    // If the params interface given to the filter from FilterChain is the standard rclcpp::n_i::NodeParameters class,
    // misuse it to get the other missing interfaces we need (they are hidden inside of it under private members which
    // we extract using the "#define private public" hack at the top of this file.
    {
      const auto params = std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeParameters>(params_);
      if (params != nullptr) {
        const auto clock = std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeClock>(params->node_clock_);
        if (clock != nullptr) {
          const auto topics = std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeTopics>(clock->node_topics_);
          if (topics != nullptr) {
            RCLCPP_INFO(
              logging_->get_logger(),
              "The filter ROS interface is initialized using a hack via its parameters interface.");
            node_interfaces_ = std::make_unique<RequiredInterfaces>(
              clock->node_base_,
              params->node_clock_,
              logging_,
              params_,
              clock->node_services_,
              clock->node_topics_
            );
            return;
          }
        }
      }
    }

    // If the params interface is something different, we create our own nodehandle. This has a lot of downsides, but
    // it's the best we can do given the circumstances.
    own_node_handle_ = std::make_shared<rclcpp::Node>(name_);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(own_node_handle_);

    node_interfaces_ = std::make_unique<RequiredInterfaces>(
      own_node_handle_->get_node_base_interface(),
      own_node_handle_->get_node_clock_interface(),
      logging_,
      params_,
      own_node_handle_->get_node_services_interface(),
      own_node_handle_->get_node_topics_interface()
    );

    executor_thread_ = std::make_unique<std::thread>(
      [this] {
        while (!should_stop_) {
          executor_->spin_all(std::chrono::milliseconds(100));
        }
      });
  }

  rclcpp::Node::SharedPtr own_node_handle_;
  rclcpp::Executor::UniquePtr executor_;  //!< Executor handling the topics required by this filter.
  std::unique_ptr<std::thread> executor_thread_;  //!< Thread running the internal executor.

  std::unique_ptr<RequiredInterfaces> node_interfaces_;
};

FilterNodeInterfaces::FilterNodeInterfaces(
  const std::string& name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& params,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& logging) : impl_(new Impl{name, logging, params}) {}

FilterNodeInterfaces::~FilterNodeInterfaces() = default;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr FilterNodeInterfaces::get_node_base_interface() const {
  return impl_->getNodeInterfaces().get_node_base_interface();
}

rclcpp::node_interfaces::NodeClockInterface::SharedPtr FilterNodeInterfaces::get_node_clock_interface() const {
  return impl_->getNodeInterfaces().get_node_clock_interface();
}

rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr FilterNodeInterfaces::get_node_logging_interface() const {
  return impl_->logging_;
}

rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
FilterNodeInterfaces::get_node_parameters_interface() const {
  return impl_->params_;
}

rclcpp::node_interfaces::NodeServicesInterface::SharedPtr FilterNodeInterfaces::get_node_services_interface() const {
  return impl_->getNodeInterfaces().get_node_services_interface();
}

rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr FilterNodeInterfaces::get_node_topics_interface() const {
  return impl_->getNodeInterfaces().get_node_topics_interface();
}

}  // namespace cras
