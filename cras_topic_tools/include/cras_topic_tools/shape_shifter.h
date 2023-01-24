#pragma once

/**
 * \file
 * \brief Tools for more convenient working with ShapeShifter objects.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <type_traits>

#include <ros/message_traits.h>
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
uint8_t* getBuffer(::topic_tools::ShapeShifter& msg);

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

/**
 * \brief Change the `header` field of the given message, if it has any.
 * \param[in,out] msg The message to change.
 * \param[in] header The header to set.
 * \return Whether setting the new header has succeeded. It will fail if either the message does not seem to have a
 *         `header` field, or if some serialization/deserialization error occurs. If the function returns false,
 *         the message has to be considered invalid and has to be discarded.
 * \note This function does not (de)serialize the whole message, it only serializes its header part.
 * \note If the new header is shorter or same length as the old one, there is no memory allocation happenning.
 * \note If the message is the "false positive" of getHeader() (i.e. it does not really hold a `header` field although
 *       getHeader() thinks it does), data corruption is very probable.
 */
bool setHeader(::topic_tools::ShapeShifter& msg, ::std_msgs::Header& header);

/**
 * \brief Resize the internal buffer of the message.
 * \param[in,out] msg The message to change.
 * \param[in] newLength New length of the internal buffer.
 * \note If the new size is the same as the old one, nothing happens. If the new one is longer, the buffer is
 *       reallocated and the contents of the old buffer are copied to it. If the new one is shorter, the contained data
 *       are cropped to the new length.
 * \note Use this function with care. After calling it, the message object becomes invalid until you fix it.
 */
void resizeBuffer(::topic_tools::ShapeShifter& msg, size_t newLength);

/**
 * \brief Copy `in` ShapeShifter to `out`.
 * \param[in] in Input message.
 * \param[in] out Output message.
 * \note This is a workaround for https://github.com/ros/ros_comm/pull/1722 which has not been merged into Melodic.
 */
void copyShapeShifter(const ::topic_tools::ShapeShifter& in, ::topic_tools::ShapeShifter& out);

/**
 * \brief Copy the message instance into the given ShapeShifter.
 * \tparam T Type of the message.
 * \param[in] msg The message to copy.
 * \param[out] shifter The ShapeShifter to copy to.
 * \note All old references to the shifter's internal buffer have to be treated as invalid after calling this function.
 */
template<typename T, typename EnableT = ::std::enable_if_t<::ros::message_traits::IsMessage<::std::decay_t<T>>::value>>
void msgToShapeShifter(const T& msg, ::topic_tools::ShapeShifter& shifter);

#if ROS_VERSION_MINIMUM(1, 15, 0)
using ::topic_tools::ShapeShifter;
#else

/**
 * \brief `ShapeShifter` class with fixed behavior on copy/move on Melodic. Use this class everywhere possible to
 *        prevent memory corruption. It seamlessly converts to `topic_tools::ShapeShifter`;
 */
class ShapeShifter : public ::topic_tools::ShapeShifter
{
public:
  ShapeShifter();
  ~ShapeShifter() override;
  ShapeShifter(const ::topic_tools::ShapeShifter& other);
  ShapeShifter(::topic_tools::ShapeShifter&& other) noexcept;
  ShapeShifter& operator=(const ::topic_tools::ShapeShifter& other);
  ShapeShifter& operator=(::topic_tools::ShapeShifter&& other) noexcept;
  ShapeShifter(const ShapeShifter& other);
  ShapeShifter(ShapeShifter&& other) noexcept;
  ShapeShifter& operator=(const ShapeShifter& other);
  ShapeShifter& operator=(ShapeShifter&& other) noexcept;
};
#endif

}

#include "impl/shape_shifter.hpp"
