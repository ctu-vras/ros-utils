/**
 * \file
 * \brief TF buffer whose functions with timeout can be interrupted.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>

#include <cras_cpp_common/time_utils/interruptible_sleep_interface.h>
#include <cras_cpp_common/thread_utils/semaphore.hpp>

namespace cras
{

/**
 * \brief Provides overrides of canTransform() that can be interrupted. Normally, canTransform() waits until transform
 * is available, timeout is reached (which may be never in paused simulation), time jumps backwards or ros::ok() returns
 * false. This buffer adds another option of interrupting the call. It also allows "wrapping around" another buffer
 * that does not support this interruptibility and add this ability to it.
 * \note In the "wrapping around" mode, do not call any other functions than canTransform() and lookupTransform(). Other
 * functions will return wrong results.
 * \note The buffer is partly concurrent. This means canTransform() and lookupTransform() calls can be interleaved, but
 * in the end all of them share the same tf2::BufferCore which locks the frames mutex in
 * canTransform()/lookupTransform(). But they access the BufferCore only from time to time in some polling intervals,
 * so concurrency should be good in the end.
 * \note The destructor of this class does not finish until no canTransform() or lookupTransform() call is being
 * executed. Call requestStop() to instruct the buffer to reject any new requests and finish the ongoing ones as soon as
 * possible, most probably resulting in failures.
 */
class InterruptibleTFBuffer : protected ::tf2_ros::Buffer, public ::cras::InterruptibleSleepInterface
{
public:
  /**
   * \brief Create the buffer.
   * \param[in] cacheTime How long to keep a history of transforms
   */
  explicit InterruptibleTFBuffer(const ::ros::Duration& cacheTime={::tf2::BufferCore::DEFAULT_CACHE_TIME, 0});
  
  /**
   * \brief Create the buffer that relays lookups to the given parentBuffer and adds the interruptible behavior to it.
   *        Cache duration is the same as in parentBuffer. Only getCacheLength(), canTransform() and lookupTransform()
   *        methods are valid on this buffer in this mode. Any modifications/tf listeners should be done on the parent
   *        buffer.
   * \param[in] parentBuffer The buffer to relay lookups to.
   */
  explicit InterruptibleTFBuffer(const ::std::shared_ptr<::tf2::BufferCore>& parentBuffer);

  /**
   * \brief Destroys the class. Waits until a running `canTransform()` call is finished.
   */
  ~InterruptibleTFBuffer() override;
  
  /**
   * \brief Whether it is OK to continue. If false, all pending lookups should stop as soon as possible.
   * \return Whether it is OK to continue.
   */
  bool ok() const override;
  
  /**
   * \brief Request all pending lookups to stop. After calling this, `ok()` should return false.
   */
  virtual void requestStop();

  bool canTransform(const ::std::string& target_frame, const ::std::string& source_frame,
    const ::ros::Time& time, ::ros::Duration timeout, ::std::string* errstr) const override;

  /**
   * \brief Test if a transform is possible.
   * \param[in] target_frame The frame into which to transform.
   * \param[in] source_frame The frame from which to transform.
   * \param[in] target_time The time at which to transform.
   * \param[in] timeout How long to block before failing.
   * \return True if the transform is possible, false otherwise. 
   */
  bool canTransform(const ::std::string& target_frame, const ::std::string& source_frame,
    const ::ros::Time& time, ::ros::Duration timeout) const;

  bool canTransform(const ::std::string& target_frame, const ::ros::Time& target_time,
    const ::std::string& source_frame, const ::ros::Time& source_time,
    const ::std::string& fixed_frame, ::ros::Duration timeout,
    ::std::string* errstr) const override;

  /**
   * \brief Test if a transform is possible.
   * \param[in] target_frame The frame into which to transform.
   * \param[in] target_time The time into which to transform.
   * \param[in] source_frame The frame from which to transform.
   * \param[in] source_time The time from which to transform.
   * \param[in] fixed_frame The frame in which to treat the transform as constant in time.
   * \param[in] timeout How long to block before failing.
   * \return True if the transform is possible, false otherwise. 
   */
  bool canTransform(const ::std::string& target_frame, const ::ros::Time& target_time,
    const ::std::string& source_frame, const ::ros::Time& source_time,
    const ::std::string& fixed_frame, ::ros::Duration timeout) const;

  ::geometry_msgs::TransformStamped lookupTransform(
    const ::std::string& target_frame, const ::std::string& source_frame,
    const ::ros::Time& time, const ::ros::Duration timeout) const override;  // NOLINT

  ::geometry_msgs::TransformStamped lookupTransform(
    const ::std::string& target_frame, const ::ros::Time& target_time,
    const ::std::string& source_frame, const ::ros::Time& source_time,
    const ::std::string& fixed_frame, const ::ros::Duration timeout) const override;  // NOLINT

  /**
   * \brief Set the scale by which `canTransform()` timeout is multiplied to determine the transform polling rate.
   * \param scale The scale to set. Must be between 0 and 1.
   * \return Whether the given scale was set. Returns false if an invalid scale was provided.
   */
  bool setCanTransformPollingScale(double scale);
  
  /**
   * \brief Set minimum duration of a pause between two transform polls. 
   * \param[in] duration The minimum duration. It has to be positive.
   * \return Whether the given duration is valid and was set.
   */
  bool setMinPollingDuration(const ::ros::Duration& duration);
	
	/**
	 * \brief Get the raw tf2::BufferCore that actually stores the transforms.
	 * \return The raw buffer.
	 */
	::tf2::BufferCore& getRawBuffer();
	
	/**
	 * \brief Get the raw tf2::BufferCore that actually stores the transforms.
	 * \return The raw buffer.
	 */
	const ::tf2::BufferCore& getRawBuffer() const;

	// The transform() functions internally use the overridden lookupTransform(), so it is okay to just call the parent
	// implementation.
	
	/** \brief Transform an input into the target frame.
	 * This function is templated and can take as input any valid mathematical object that tf knows
	 * how to apply a transform to, by way of the templated math conversions interface.
	 * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
	 * type (as defined in geometry_msgs).
	 * \tparam T The type of the object to transform.
	 * \param[in] in The object to transform
	 * \param[out] out The transformed output, preallocated by the caller.
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 */
	template <class T>
	T& transform(const T& in, T& out, const ::std::string& target_frame, ::ros::Duration timeout={0, 0}) const
	{
		return ::tf2_ros::Buffer::transform(in, out, target_frame, timeout);
	}

	/** \brief Transform an input into the target frame.
	 * This function is templated and can take as input any valid mathematical object that tf knows
	 * how to apply a transform to, by way of the templated math conversions interface.
	 * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
	 * type (as defined in geometry_msgs).
	 * \tparam T The type of the object to transform.
	 * \param[in] in The object to transform.
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 * \return The transformed output.
	 */
	template <class T>
	T transform(const T& in, const ::std::string& target_frame, ::ros::Duration timeout={0, 0}) const
	{
		T out;
		return ::tf2_ros::Buffer::transform(in, target_frame, timeout);
	}

	/** \brief Transform an input into the target frame and convert to a specified output type.
	 * It is templated on two types: the type of the input object and the type of the
	 * transformed output.
	 * For example, the template types could be Transform, Pose, Vector, or Quaternion messages
	 * type (as defined in geometry_msgs).
	 * The function will calculate the transformation and then convert the result into the
	 * specified output type.
	 * Compilation will fail if a known conversion does not exist bewteen the two template
	 * parameters.
	 * \tparam A The type of the object to transform.
	 * \tparam B The type of the transformed output.
	 * \param[in] in The object to transform
	 * \param[out] out The transformed output, converted to the specified type.
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 * \return The transformed output, converted to the specified type.
	 */
	template <class A, class B>
	B& transform(const A& in, B& out, const ::std::string& target_frame, ::ros::Duration timeout={0, 0}) const
	{
		return ::tf2_ros::Buffer::transform(in, out, target_frame, timeout);
	}

	/** \brief Transform an input into the target frame (advanced).
	 * This function is templated and can take as input any valid mathematical object that tf knows
	 * how to apply a transform to, by way of the templated math conversions interface.
	 * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
	 * type (as defined in geometry_msgs).
	 * This function follows the advanced API, which allows transforming between different time
	 * points, and specifying a fixed frame that does not varying in time.
	 * \tparam T The type of the object to transform.
	 * \param[in] in The object to transform
	 * \param[out] out The transformed output, preallocated by the caller.
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] target_time The time into which to transform
	 * \param[in] fixed_frame The frame in which to treat the transform as constant in time.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 */
	template <class T>
	T& transform(const T& in, T& out, const ::std::string& target_frame, const ::ros::Time& target_time,
		const ::std::string& fixed_frame, ::ros::Duration timeout={0, 0}) const
	{
		return ::tf2_ros::Buffer::transform(in, out, target_frame, target_time, fixed_frame, timeout);
	}


	/** \brief Transform an input into the target frame (advanced).
	 * This function is templated and can take as input any valid mathematical object that tf knows
	 * how to apply a transform to, by way of the templated math conversions interface.
	 * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
	 * type (as defined in geometry_msgs).
	 * This function follows the advanced API, which allows transforming between different time
	 * points, and specifying a fixed frame that does not varying in time.
	 * \tparam T The type of the object to transform.
	 * \param[in] in The object to transform
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] target_time The time into which to transform
	 * \param[in] fixed_frame The frame in which to treat the transform as constant in time.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 * \return The transformed output.
	 */
	template <class T>
	T transform(const T& in, const ::std::string& target_frame, const ::ros::Time& target_time,
		const ::std::string& fixed_frame, ::ros::Duration timeout={0, 0}) const
	{
		return ::tf2_ros::Buffer::transform(in, target_frame, target_time, fixed_frame, timeout);
	}

	/** \brief Transform an input into the target frame and convert to a specified output type (advanced).
	 * It is templated on two types: the type of the input object and the type of the
	 * transformed output.
	 * For example, the template type could be a Transform, Pose, Vector, or Quaternion message
	 * type (as defined in geometry_msgs).
	 * The function will calculate the transformation and then convert the result into the
	 * specified output type.
	 * Compilation will fail if a known conversion does not exist bewteen the two template
	 * parameters.
	 * This function follows the advanced API, which allows transforming between different time
	 * points, and specifying a fixed frame that does not varying in time.
	 * \tparam A The type of the object to transform.
	 * \tparam B The type of the transformed output.
	 * \param[in] in The object to transform
	 * \param[in] out The transformed output, converted to the specified output type.
	 * \param[in] target_frame The string identifer for the frame to transform into.
	 * \param[in] target_time The time into which to transform
	 * \param[in] fixed_frame The frame in which to treat the transform as constant in time.
	 * \param[in] timeout How long to wait for the target frame. Default value is zero (no blocking).
	 * \return The transformed output, converted to the specified output type.
	 */
	template <class A, class B>
	B& transform(const A& in, B& out, const ::std::string& target_frame, const ::ros::Time& target_time,
		const ::std::string& fixed_frame, ::ros::Duration timeout={0, 0}) const
	{
		return ::tf2_ros::Buffer::transform(in, out, target_frame, target_time, fixed_frame, timeout);
	}
	
	// Methods copied from BufferCore, they do not rely on time.

	/**
	 * \brief Get the duration over which this transformer will cache.
	 */
	::ros::Duration getCacheLength();
	
	/**
	 * \brief Clear all data.
   */
	void clear();

	/**
	 * \brief Add transform information to the tf data structure
	 * \param[in] transform The transform to store
	 * \param[in] authority The source of the information for this transform
	 * \param[in] is_static Record this transform as a static transform.  It will be good across all time.
	 * 										  (This cannot be changed after the first call.)
	 * \return True unless an error occurred.
	 */
	bool setTransform(const ::geometry_msgs::TransformStamped& transform, const ::std::string& authority,
		bool is_static = false);

	/**
	 * \brief Get the transform between two frames by frame ID.
	 * \param[in] target_frame The frame to which data should be transformed
	 * \param[in] source_frame The frame where the data originated
	 * \param[in] time The time at which the value of the transform is desired. (0 will get the latest)
	 * \return The transform between the frames.
	 * \throws tf2::LookupException, tf2::ConnectivityException, tf2::ExtrapolationException,
	 *         tf2::InvalidArgumentException
	 */
	::geometry_msgs::TransformStamped lookupTransform(const ::std::string& target_frame,
		const ::std::string& source_frame, const ::ros::Time& time) const;

	/**
	 * \brief Get the transform between two frames by frame ID assuming fixed frame.
	 * \param[in] target_frame The frame to which data should be transformed
	 * \param[in] target_time The time to which the data should be transformed. (0 will get the latest)
	 * \param[in] source_frame The frame where the data originated
	 * \param[in] source_time The time at which the source_frame should be evaluated. (0 will get the latest)
	 * \param[in] fixed_frame The frame in which to assume the transform is constant in time. 
	 * \return The transform between the frames.
	 * \throws tf2::LookupException, tf2::ConnectivityException, tf2::ExtrapolationException,
	 *         tf2::InvalidArgumentException
	 */
	::geometry_msgs::TransformStamped lookupTransform(const ::std::string& target_frame, const ::ros::Time& target_time,
		const ::std::string& source_frame, const ::ros::Time& source_time, const ::std::string& fixed_frame) const;

	/**
	 * \brief Test if a transform is possible
	 * \param[in] target_frame The frame into which to transform
	 * \param[in] source_frame The frame from which to transform
	 * \param[in] time The time at which to transform
	 * \param[out] error_msg A pointer to a string which will be filled with why the transform failed, if not NULL.
	 * \return True if the transform is possible, false otherwise.
	 */
	bool canTransform(const ::std::string& target_frame, const ::std::string& source_frame, const ::ros::Time& time,
		::std::string* error_msg = nullptr) const;

	/**
	 * \brief Test if a transform is possible
	 * \param[in] target_frame The frame into which to transform
	 * \param[in] target_time The time into which to transform
	 * \param[in] source_frame The frame from which to transform
	 * \param[in] source_time The time from which to transform
	 * \param[in] fixed_frame The frame in which to treat the transform as constant in time
	 * \param[out] error_msg A pointer to a string which will be filled with why the transform failed, if not NULL.
	 * \return True if the transform is possible, false otherwise.
	 */
	bool canTransform(const ::std::string& target_frame, const ::ros::Time& target_time,
		const ::std::string& source_frame, const ::ros::Time& source_time, const ::std::string& fixed_frame,
		::std::string* error_msg = nullptr) const;

	/**
	 * \brief A way to see what frames have been cached in yaml format. Useful for debugging tools.
	 */
	::std::string allFramesAsYAML(double current_time) const;

	/**
	 * \brief Backwards compatibility for #84.
	 */
	::std::string allFramesAsYAML() const;

	/**
	 * \brief A way to see what frames have been cached. Useful for debugging.
	 */
	::std::string allFramesAsString() const;

protected:
  //! \brief If not null, this class relays all lookups to parentBuffer.
  const ::std::shared_ptr<::tf2::BufferCore> parentBuffer {nullptr};
  
  //! \brief `canTransform()` timeout is multiplied by this scale and polling for the transform is done in these time
  //!         steps.  
  double canTransformPollingScale {0.01};
  
  //! \brief Minimum duration of a pause between two transform polls. If timeout * canTransformPollingScale would be
  //!        smaller than this value, this value will be used instead to prevent too frequent queries.
  ::ros::Duration minPollingDuration {0, 1000000};
  
private:
  //! \brief True until `requestStop()` is called.
  bool isOk {true};

  //! \brief If parentBuffer is set and it is an InterruptibleTFBuffer, its dynamic cast to this type is stored here.
  const ::std::shared_ptr<::cras::InterruptibleTFBuffer> interruptibleParentBuffer {nullptr};
 
  //! \brief Reverse semaphore guarding that the object is not destroyed before all pending callbacks finish. At the
  //! start of each callback, the semaphore is increased, and at the end of the callback, it is decreased. The
  //! destructor of this class will block until the semaphore reaches zero.
  mutable ::cras::ReverseSemaphore callbackSemaphore;
};

}