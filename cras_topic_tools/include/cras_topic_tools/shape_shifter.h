#pragma once

/**
 * \file
 * \brief Tools for more convenient working with ShapeShifter objects.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <std_msgs/Header.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/optional.hpp>

namespace cras
{

/**
 * \brief Get the internal buffer with serialized message data.
 * \param[in] msg The shape shifter object.
 * \return The internal buffer.
 * \note The buffer can stop being valid after a ShapeShifter::read() operation or when the shifter is destroyed.
 */
const uint8_t* getBuffer(const ::topic_tools::ShapeShifter& msg);

/**
 * \brief Get the length of the internal buffer with serialized message data.
 * \param[in] msg The shape shifter object.
 * \return The internal buffer length.
 * \note The buffer can stop being valid after a ShapeShifter::read() operation or when the shifter is destroyed.
 */
size_t getBufferLength(const ::topic_tools::ShapeShifter& msg);

/**
 * \brief Get the `header` field of the given message, if it has any.
 * \param[in] msg The shape shifter object.
 * \return The `header` of the represented message. If there is no header, `nullopt` is returned.
 * \note This function can have some "false positives", as it doesn't actually know whether the message has a header or
 *       not. There is a high chance that if the message does not have a header, the reading of Header data will fail,
 *       but there are corner cases when the message data will be interpretable as a Header. It is advised to check
 *       whether the message has a header via introspection or some other kind of information.
 */
::cras::optional<::std_msgs::Header> getHeader(const ::topic_tools::ShapeShifter& msg);

}