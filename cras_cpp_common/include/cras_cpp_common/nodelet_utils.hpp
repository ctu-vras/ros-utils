#pragma once

/**
 * \file
 * \brief This file contains a set of classes that make work with nodelets easier.
 * 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 * 
 * \details
 * There are a few helper "mixins":
 *
 * NodeletParamHelper provides the conveniently templatized getParam() methods similar to those in node_utils.
 *
 * StatefulNodelet adds state to the nodelets, so that you can query isOk() to determine whether the nodelet is not
 * shutting down.
 *
 * ThreadNameUpdatingNodelet adds function setThreadName() which you can call in your callbacks to signal to the OS
 * which nodelet is currently being run in the particular nodelet manager's thread.
 *
 * The NodeletSharingTfBuffer adds functions setBuffer(), getBuffer() and usesSharedBuffer(), which allow the nodelet
 * manager to pass a pointer to a shared TF buffer. Note that standard nodelet manager cannot make use of this feature,
 * so to utilize it, you should launch nodelet_manager_sharing_tf_buffer from this package instead.
 *
 * Finally, NodeletBase<BaseNodelet> template class adds all these mixins to the provided BaseNodelet class to create
 * a nodelet where you can use all the features added by the mentioned mixins. Use this template if you need to
 * derive your nodelet from a different class than nodelet::Nodelet (like PCLNodelet).
 *
 * If you just want to build your nodelet from scratch, you should start by deriving it from the cras::Nodelet class
 * which has all the mixins and is based on nodelet::Nodelet.
 * 
 * Here's an example how your custom nodelet class should be started:
 * 
 * <code>
 * class MyClass : public cras\:\:Nodelet
 * {
 * ...
 * }
 * </code>
 * 
 * In case you need to base your class on something else than nodelet::Nodelet, here's an example with a PCLNodelet:
 * 
 * <code>
 * class MyClass : public cras\:\:NodeletBase<pcl_ros\:\:PCLNodelet>
 * {
 * ...
 * }
 * </code>
 *
 * NodeletAwareTFBuffer is a class that is able to correctly end canTransform() calls in case the nodelet is asked to
 * unload while the canTransform() call is waiting.
 * 
 * \note The "mixin" classes are templated so that each of them has a virtual base class BaseNodelet. It is important
 * that this base class is virtual, otherwise NodeletBase and other classes combining multiple mixins would contain
 * multiple "copies" of BaseNodelet, and that would lead to ambiguous function calls etc. Virtual inheritance instead
 * creates just a single copy of BaseNodelet shared by all the mixins.
 * 
 * \note The mixins can call methods from other mixins by dynamic_casting this to OtherMixin<BaseNodelet>*. It is not
 * guaranteed that all mixins will be always available, so be sure to correctly handle the case when the dynamic_cast
 * yields nullptr.
 * 
 * \note If the mixins need to expose a public API (as is the case of NodeletWithSharedTfBuffer), the public API must
 * be extracted to a separate interface class that is not templated (otherwise, the users of the public API would need
 * to dynamic cast the nodelet to Mixin<NodeletType>, but NodeletType could be anything from the view of a public user).
 * The interface class does not depend on NodeletType, which allows public users to dynamic_cast the nodelet to this
 * interface. To use the implementations from the mixin for the interface class, the interface functions should be
 * declared pure virtual and the mixin function should override these. See NodeletWithSharedTfBuffer for an example.
 */

#include <nodelet/nodelet.h>

#include "nodelet_utils/log_macros.h"
#include "nodelet_utils/param_helper.hpp"
#include "nodelet_utils/nodelet_aware_tf_buffer.h"
#include "nodelet_utils/nodelet_with_diagnostics.hpp"
#include "nodelet_utils/nodelet_with_shared_tf_buffer.hpp"
#include "nodelet_utils/stateful_nodelet.hpp"
#include "nodelet_utils/thread_name_updating_nodelet.hpp"

namespace cras
{

/**
 * \brief Base template which adds all defined mixins to BaseNodelet class.
 * \tparam BaseNodelet Type of the nodelet that should be the parent class of this one.
 */
template <typename BaseNodelet>
class NodeletBase :
  public virtual BaseNodelet,  // has to be virtual, most mixins also have BaseNodelet as virtual base class
  public ::cras::NodeletWithDiagnostics<BaseNodelet>,
  public ::cras::NodeletWithSharedTfBuffer<BaseNodelet>,
  public ::cras::ThreadNameUpdatingNodelet<BaseNodelet>,
  public ::cras::NodeletParamHelper<BaseNodelet>,
  public ::cras::StatefulNodelet<BaseNodelet>
{
public:
  ~NodeletBase() override = default;
protected:
  using BaseNodelet::getName;  // for disambiguation because some mixins contain a "using getName()"
};

/**
 * A convenient base class for all nodelets.
 */
class Nodelet : public ::cras::NodeletBase<::nodelet::Nodelet>
{
public:
  ~Nodelet() override = default;
};

}
