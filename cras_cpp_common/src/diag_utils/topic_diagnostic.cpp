/**
 * \file
 * \brief This file is for backwards compatibility only.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>

#include <cras_cpp_common/diag_utils/deprecated/topic_diagnostic.h>

namespace cras
{

TopicDiagnostic::TopicDiagnostic(
  const std::string&, diagnostic_updater::Updater& updater, const BoundParamHelperPtr& params) :
    DiagnosedPubSub(params)
{
  updater.add(*this->diag);
}

TopicDiagnostic::TopicDiagnostic(
  const std::string&, diagnostic_updater::Updater& updater, const BoundParamHelperPtr& params,
  const ros::Rate& defaultRate) :
    DiagnosedPubSub(params, {frequency(defaultRate), frequency(defaultRate)})
{
  updater.add(*this->diag);
}

TopicDiagnostic::TopicDiagnostic(
  const std::string&, diagnostic_updater::Updater& updater, const BoundParamHelperPtr& params,
  const ros::Rate& defaultRate, const ros::Rate& defaultMinRate, const ros::Rate& defaultMaxRate) :
    DiagnosedPubSub(params, {frequency(defaultMinRate), frequency(defaultMaxRate)})
{
  updater.add(*this->diag);
}

void TopicDiagnostic::tick(const ros::Time& stamp)
{
  this->diag->tick(stamp);
}

std::string TopicDiagnostic::getName() const
{
  return this->diag->getName();
}

}
