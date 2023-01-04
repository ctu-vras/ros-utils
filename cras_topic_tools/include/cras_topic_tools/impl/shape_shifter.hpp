#pragma once

/**
 * \file
 * \brief Tools for more convenient working with ShapeShifter objects (implementation details, do not include directly).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

#include <cras_topic_tools/shape_shifter.h>

namespace cras
{

template<typename T, typename EnableT>
void msgToShapeShifter(const T& msg, ::topic_tools::ShapeShifter& shifter)
{
  // Serialize the message into a byte buffer
  const auto length = ::ros::serialization::serializationLength(msg);
  ::cras::resizeBuffer(shifter, length);
  ::ros::serialization::OStream ostream(::cras::getBuffer(shifter), length);
  ::ros::serialization::serialize(ostream, msg);
  shifter.morph(::ros::message_traits::MD5Sum<T>::value(), ::ros::message_traits::DataType<T>::value(),
                ::ros::message_traits::Definition<T>::value(), "0");
}

}
