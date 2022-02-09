/**
 * \file
 * \brief Helpers for setting up diagnostics for nodelets.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <memory>
#include <string>
#include <type_traits>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/message_traits.h>
#include <ros/node_handle.h>
#include <ros/rate.h>

#include <cras_cpp_common/diag_utils.hpp>
#include <cras_cpp_common/diagnostics/ImprovedUpdater.h>
#include <cras_cpp_common/node_utils.hpp>

namespace cras
{

namespace impl
{
// forward declaration
struct NodeletWithDiagnosticsPrivate;
}

/**
 * \brief Nodelet mixin that provides helper functions for running a diagnostics updater.
 * \tparam NodeletType Type of the base nodelet.
 */
template <typename NodeletType>
struct NodeletWithDiagnostics : public virtual NodeletType
{
public:
  NodeletWithDiagnostics();
  virtual ~NodeletWithDiagnostics();  // we need to be polymorphic
  
protected:
  /**
   * \brief Get a diagnostic updater to be used with this nodelet.
   * \return The updater.
   */
  ::diagnostic_updater::Updater& getDiagUpdater() const;
  
  /**
   * \brief Start periodic updates of the diagnostics updater.
   * \param[in] nh The node handle on which the timer should be started.
   */
  void startDiagTimer(const ::ros::NodeHandle& nh) const;
  
  /**
   * \brief Stop the automatic updates of the diagnostic updater.
   */
  void stopDiagTimer() const;

  /**
   * \brief Create a diagnosed publisher for a message type without header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size.
   * \param[in] defaultRate Default expected rate.
   * \param[in] defaultMinRate Default minimum rate.
   * \param[in] defaultMaxRate Default maximum rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<!::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate, const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate)
  {
    return ::std::make_unique<::cras::HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace),
      defaultRate, defaultMinRate, defaultMaxRate);
  }

  /**
   * \brief Create a diagnosed publisher for a message type without header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size.
   * \param[in] defaultRate Default expected rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<!::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate)
  {
    return ::std::make_unique<::cras::HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace),
      defaultRate);
  }

  /**
   * \brief Create a diagnosed publisher for a message type without header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<!::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::HeaderlessDiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace)
  {
    return ::std::make_unique<::cras::HeaderlessDiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace));
  }

  /**
   * \brief Create a diagnosed publisher for a message type with header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size,
   *                           delay/min, delay/max.
   * \param[in] defaultRate Default expected rate.
   * \param[in] defaultMinRate Default minimum rate.
   * \param[in] defaultMaxRate Default maximum rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate, const ::ros::Rate& defaultMinRate, const ::ros::Rate& defaultMaxRate)
  {
    return ::std::make_unique<::cras::DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace),
      defaultRate, defaultMinRate, defaultMaxRate);
  }

  /**
   * \brief Create a diagnosed publisher for a message type with header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size,
   *                           delay/min, delay/max.
   * \param[in] defaultRate Default expected rate.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace,
    const ::ros::Rate& defaultRate)
  {
    return ::std::make_unique<::cras::DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace),
      defaultRate);
  }

  /**
   * \brief Create a diagnosed publisher for a message type with header.
   * \tparam T Type of the published data.
   * \param[in] nh Node handle for which the publisher should be created.
   * \param[in] topic Topic to publish to.
   * \param[in] queueSize Queue size.
   * \param[in] paramNamespace Namespace in private parameters of this nodelet where configuration of the diagnostics is
   *                           stored. Params are: rate/desired, rate/max, rate/min, rate/tolerance, rate/window_size,
   *                           delay/min, delay/max.
   * \return The publisher with diagnostics.
   */
  template<typename T>
  typename ::std::enable_if<::ros::message_traits::HasHeader<T>::value,
    ::std::unique_ptr<::cras::DiagnosedPublisher<T>>>::type
  createDiagnosedPublisher(
    ::ros::NodeHandle nh, const ::std::string& topic, size_t queueSize, const ::std::string& paramNamespace)
  {
    return ::std::make_unique<::cras::DiagnosedPublisher<T>>(
      nh.template advertise<T>(topic, queueSize), this->getDiagUpdater(),
      this->getDiagnosedPublisherParams(nh, paramNamespace));
  }

  /**
   * \brief Get the parameters configuring a diagnosed publisher.
   * \param[in] nh Node handle of the publisher.
   * \param[in] paramNamespace Parameter sub-namespace inside the node handle's namespace.
   * \return Param helper with the parameters.
   */
  ::cras::BoundParamHelperPtr getDiagnosedPublisherParams(
    const ::ros::NodeHandle& nh, const ::std::string& paramNamespace);

private:
  //! \brief PIMPL
  ::std::unique_ptr<::cras::impl::NodeletWithDiagnosticsPrivate> data;
};

}

#include "impl/nodelet_with_diagnostics.hpp"