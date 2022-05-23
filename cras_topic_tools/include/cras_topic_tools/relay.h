#pragma once

/**
 * \file
 * \brief This is a simple implementation of a relay nodelet. It can process the messages on a single topic in parallel
 *        allowing for maximum throughput.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <cras_cpp_common/nodelet_utils.hpp>

#include <cras_topic_tools/generic_lazy_pubsub.hpp>

namespace cras
{

class RelayNodelet : public ::cras::Nodelet
{
protected:
  ::std::unique_ptr<::cras::GenericLazyPubSub<>> pubSub;
  
  void onInit() override;
};

}
