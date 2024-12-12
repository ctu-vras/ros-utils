#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief NodeletWithSharedTfBuffer allows you to use a tf2_ros::Buffer provided by the nodelet manager.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <rosconsole/macros_generated.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cras_cpp_common/nodelet_utils/nodelet_aware_tf_buffer.h>
#include <cras_cpp_common/resettable.h>

namespace cras
{

namespace impl
{
// forward declaration
struct NodeletWithSharedTfBufferPrivate;
}

/**
 * \brief Public non-template API of NodeletWithSharedTfBuffer. Dynamic_cast a nodelet to this type if you need to
 * access this API publicly.
 */
struct NodeletWithSharedTfBufferInterface
{
public:
  /**
   * \brief Set the TF buffer to be used by the nodelet. If this method is not called, a standalone buffer is created.
   * \param[in] buffer The buffer to use.
   */
  virtual void setBuffer(const ::std::shared_ptr<::tf2_ros::Buffer>& buffer) = 0;

  /**
   * \brief Get the TF buffer used by the nodelet. If none has been set by `setBuffer()`, a buffer is automatically
   * created.
   * \return The buffer.
   * \note This buffer only offers the timeouting versions of canTransform() and lookupTransform().
   *       It does not offer setTransform() and many other functions. User getRawBuffer() on the returned instance to
   *       get a tf2::BufferCore that offers the missing non-time-aware functionality.
   */
  virtual ::cras::NodeletAwareTFBuffer& getBuffer() const = 0;

  /**
   * \brief Whether the buffer set using `setBuffer()` is used or a standalone buffer has been automatically created.
   * \return Whether the buffer set using `setBuffer()` is used or a standalone buffer has been automatically created.
   */
  virtual bool usesSharedBuffer() const = 0;

  /**
   * \brief Reset the TF buffer. If a shared buffer is used, it is not reset by this call and its owner is responsible.
   */
  virtual void reset() = 0;
};

/**
 * \brief A nodelet mixin that allows to use a tf2_ros::Buffer provided by the nodelet manager (which should save some
 * computations). If this nodelet has also the `StatefulNodelet` mixin, the automatically created non-shared buffer
 * is nodelet-aware (you can also pass a `NodeletAwareTfBuffer` to `setBuffer()`). That means any TF lookups done via
 * `this->getBuffer()` will be able to correctly end when the nodelet is being unloaded (which normally hangs:
 * https://github.com/ros/geometry2/issues/381).
 * \tparam NodeletType Type of the base nodelet.
 */
template <typename NodeletType = ::nodelet::Nodelet>
struct NodeletWithSharedTfBuffer : public virtual NodeletType, public ::cras::NodeletWithSharedTfBufferInterface,
  protected ::cras::TimeJumpResettable
{
public:
  NodeletWithSharedTfBuffer();
  ~NodeletWithSharedTfBuffer() override;

  void setBuffer(const ::std::shared_ptr<::tf2_ros::Buffer>& buffer) override;
  ::cras::NodeletAwareTFBuffer& getBuffer() const override;
  ::std::shared_ptr<::cras::NodeletAwareTFBuffer> getBufferPtr() const;
  bool usesSharedBuffer() const override;

  virtual void onInit();

  void reset() override;

protected:
  using NodeletType::getName;

private:
  //! \brief PIMPL
  ::std::unique_ptr<::cras::impl::NodeletWithSharedTfBufferPrivate> data;
};

}

#include "impl/nodelet_with_shared_tf_buffer.hpp"
