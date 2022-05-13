#pragma once

/**
 * \file
 * \brief Definitions of parameters for a TopicStatus diagnostic task.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <limits>
#include <type_traits>

#include <diagnostic_updater/update_functions.h>
#include <ros/message_traits.h>

namespace cras
{

/**
 * \brief Helper struct for easy brace-initialization of TopicStatusParam objects. On supported compilers, you can also
 * use designated braced initialization, i.e. `param = {.maxRate = 10}`. Supported is e.g. GCC 8+ in any mode or any
 * compiler in C++20 mode.
 * \note Version for message types without a header (cannot measure delay).
 */
struct SimpleTopicStatusParamNoHeader
{
  typedef ::std::remove_pointer_t<decltype(::diagnostic_updater::FrequencyStatusParam::max_freq_)> RateType;
  
  //! \brief Minimum allowed frequency.
  RateType minRate {0};
  
  //! \brief Maximum allowed frequency.
  RateType maxRate {::std::numeric_limits<RateType>::infinity()};

  //! \brief Tolerance of the rate.
  decltype(::diagnostic_updater::FrequencyStatusParam::tolerance_) rateTolerance {0.1};

  //! \brief Number of updates during which the frequency is computed.
  decltype(::diagnostic_updater::FrequencyStatusParam::window_size_) rateWindowSize {5};
};

/**
 * \brief Helper struct for easy brace-initialization of TopicStatusParam objects. On supported compilers, you can also
 * use designated braced initialization, i.e. `param = {.maxRate = 10}`. Supported is e.g. GCC 8+ in any mode or any
 * compiler in C++20 mode.
 * \note Version for message types with a header (can measure delay).
 */
struct SimpleTopicStatusParamWithHeader
{
  typedef ::std::remove_pointer_t<decltype(::diagnostic_updater::FrequencyStatusParam::max_freq_)> RateType;
  
  //! \brief Minimum allowed frequency.
  RateType minRate {0};
  
  //! \brief Maximum allowed frequency.
  RateType maxRate {::std::numeric_limits<RateType>::infinity()};
  
  //! \brief Tolerance of the rate.
  decltype(::diagnostic_updater::FrequencyStatusParam::tolerance_) rateTolerance {0.1};
  
  //! \brief Number of updates during which the frequency is computed.
  decltype(::diagnostic_updater::FrequencyStatusParam::window_size_) rateWindowSize {5};
  
  //! \brief Min acceptable delay (in s). It can be negative if timestamps in future are expected.
  decltype(::diagnostic_updater::TimeStampStatusParam::min_acceptable_) minDelay {-1.0};
  
  //! \brief Max acceptable delay (in s). It can be negative if timestamps in future are expected.
  decltype(::diagnostic_updater::TimeStampStatusParam::max_acceptable_) maxDelay {5.0};
};

/**
 * \brief Helper struct for easy brace-initialization of TopicStatusParam objects.
 * \tparam Message Type of the message for which parameters are constructed.
 * \note This typedef automatically chooses WithHeader or NoHeader variant of the struct based on the type of the
 * message.
 */
template <typename Message>
using SimpleTopicStatusParam = ::std::conditional_t<::ros::message_traits::HasHeader<Message>::value,
  ::cras::SimpleTopicStatusParamWithHeader, ::cras::SimpleTopicStatusParamNoHeader>;

/**
 * \brief Utility class with various predefined topic diagnostic parameters.
 */
struct TopicStatusParams
{
  //! \brief The type of parameters for topics without a header
  typedef ::cras::SimpleTopicStatusParamNoHeader NoHeaderParam;
  //! \brief The type of parameters for topics with header
  typedef ::cras::SimpleTopicStatusParamWithHeader HeaderParam;

  //! \brief 1 Hz headerless topic, rate tolerance 10%, window size 5 updates
  static constexpr NoHeaderParam _1Hz {1, 1};
  //! \brief 1 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 5 secs
  static constexpr HeaderParam delayed1Hz {1, 1};
  //! \brief 1 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 0.1 secs
  static constexpr HeaderParam fast1Hz {1, 1, 0.1, 5, -1, 0.1};

  //! \brief 10 Hz headerless topic, rate tolerance 10%, window size 5 updates
  static constexpr NoHeaderParam _10Hz {10, 10};
  //! \brief 10 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 5 secs
  static constexpr HeaderParam delayed10Hz {10, 10};
  //! \brief 10 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 0.1 secs
  static constexpr HeaderParam fast10Hz {10, 10, 0.1, 5, -1, 0.1};

  //! \brief 100 Hz headerless topic, rate tolerance 10%, window size 5 updates
  static constexpr NoHeaderParam _100Hz {100, 100};
  //! \brief 100 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 5 secs
  static constexpr HeaderParam delayed100Hz {100, 100};
  //! \brief 100 Hz topic with header, rate tolerance 10%, window size 5 updates, delay -1 to 0.1 secs
  static constexpr HeaderParam fast100Hz {100, 100, 0.1, 5, -1, 0.1};
};

/**
 * \brief An extension of diagnostic_updater::FrequencyStatusParam that allows passing non-pointer min/max frequency
 * values. If the internal pointers are used, they are correctly handled when copying this object (the new copy will
 * have its own copy of the internal pointers).
 */
class FrequencyStatusParam : public ::diagnostic_updater::FrequencyStatusParam
{
public:
  /**
   * \brief The original constructor from diagnostic_updater::FrequencyStatusParam.
   * \param[in] min_freq Pointer to a double containing the minimum allowed frequency. Cannot be nullptr.
   * \param[in] max_freq Pointer to a double containing the maximum allowed frequency. Cannot be nullptr.
   * \param[in] tolerance Tolerance of the rate.
   * \param[in] window_size Number of updates during which the frequency is computed.
   */
  FrequencyStatusParam(double *min_freq, double *max_freq, double tolerance = 0.1, int window_size = 5);

  /**
   * \brief The added constructor not requiring pointers.
   * \param[in] min_freq Minimum allowed frequency.
   * \param[in] max_freq Maximum allowed frequency.
   * \param[in] tolerance Tolerance of the rate.
   * \param[in] window_size Number of updates during which the frequency is computed.
   * \note Internal pointers for min/max rate are used.
   */
  explicit FrequencyStatusParam(
    double min_freq = 0.0, double max_freq = ::std::numeric_limits<double>::infinity(),
    double tolerance = 0.1, int window_size = 5);
  
  /**
   * \brief Copy values from another object of this type. If it uses the internal pointers, their values are copied and
   * a new set of internal pointers is created for this object, making it independent of the other one. If the other
   * object uses explicit pointer, this object will use pointers to the same memory.
   * \param[in] other The object to copy from.
   */
  FrequencyStatusParam(const ::cras::FrequencyStatusParam& other);
  
  /**
   * \brief Copy values from the original `diagnostic_updater::FrequencyStatusParam`. This object will use pointers to
   * the same memory for min/max rate as the other one.
   * \param[in] other The object to copy from.
   */
  FrequencyStatusParam(const ::diagnostic_updater::FrequencyStatusParam& other);  // NOLINT(google-explicit-constructor)
  
  /**
   * \brief Initialize this object from `SimpleTopicStatusParam`. Internal pointers for min/max rate will be used, so
   * the `params` object may cease to exist.
   * \param[in] params The parameters to initialize with.
   */
  FrequencyStatusParam(const ::cras::SimpleTopicStatusParamNoHeader& params);  // NOLINT(google-explicit-constructor)

  /**
   * \brief Copy values from another object of this type. If it uses the internal pointers, their values are copied and
   * a new set of internal pointers is created for this object, making it independent of the other one. If the other
   * object uses explicit pointer, this object will use pointers to the same memory.
   * \param[in] other The object to copy from.
   */
  ::cras::FrequencyStatusParam& operator=(const ::cras::FrequencyStatusParam& other);

  /**
   * \brief Copy values from the original `diagnostic_updater::FrequencyStatusParam`. This object will use pointers to
   * the same memory for min/max rate as the other one.
   * \param[in] other The object to copy from.
   */
  ::cras::FrequencyStatusParam& operator=(const ::diagnostic_updater::FrequencyStatusParam& other);
  
  /**
   * \brief Get the expected/average rate. If min and max are the same, their value will be returned. If min rate is
   * non-positive, the max rate is returned. Otherwise, if max rate is infinite, the min rate will be returned. If min
   * is positive and max is finite, their arithmetic mean is returned.
   * \return The expected rate (in Hz).
   */
  double getExpectedRate() const;

private:
  /**
   * \brief This method handles copying of the internal pointers if they are used.
   * \param[in] other 
   */
  void copyAdditionalData(const ::cras::FrequencyStatusParam& other);

  //! \brief Minimum frequency. This field is used by the internal pointers if they are used, otherwise it is ignored.
  double minFreq {0.0};

  //! \brief Maximum frequency. This field is used by the internal pointers if they are used, otherwise it is ignored.
  double maxFreq {::std::numeric_limits<double>::infinity()};
  
  //! \brief Whether internal pointers are used or not (depends on whether this object was constructed with explicit
  //! rate pointers or not).
  bool usesInternalPointers {false};
};

/**
 * \brief A combined parameter defining both rate diagnostics and delay diagnostics (for messages with a header).
 * \note This is a combination of `FrequencyStatusParam` and `TimeStampStatusParam`.
 */
class TopicStatusParamWithHeader :
  public ::cras::FrequencyStatusParam, public ::diagnostic_updater::TimeStampStatusParam
{
public:
  /**
   * \brief Construct the param object.
   * \param[in] min_freq Pointer to a double containing the minimum allowed frequency. Cannot be nullptr.
   * \param[in] max_freq Pointer to a double containing the maximum allowed frequency. Cannot be nullptr.
   * \param[in] tolerance Tolerance of the rate.
   * \param[in] window_size Number of updates during which the frequency is computed.
   * \param[in] min_acceptable Min acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \param[in] max_acceptable Max acceptable delay (in s). It can be negative if timestamps in future are expected.
   */
  TopicStatusParamWithHeader(
    double* min_freq, double* max_freq, double tolerance = 0.1, int window_size = 5,
    double min_acceptable = -1.0, double max_acceptable = 5.0);

  /**
   * \brief Construct the param object.
   * \param[in] min_freq Minimum allowed frequency.
   * \param[in] max_freq Maximum allowed frequency.
   * \param[in] tolerance Tolerance of the rate.
   * \param[in] window_size Number of updates during which the frequency is computed.
   * \param[in] min_acceptable Min acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \param[in] max_acceptable Max acceptable delay (in s). It can be negative if timestamps in future are expected.
   * \note Internal pointers for min/max rate are used.
   */
  TopicStatusParamWithHeader(  // NOLINT(google-explicit-constructor)
    double min_freq = 0.0, double max_freq = ::std::numeric_limits<double>::infinity(),
    double tolerance = 0.1, int window_size = 5, double min_acceptable = -1.0, double max_acceptable = 5.0);
  
  /**
   * \brief Construct the object from a `FrequencyStatusParam` using default timestamp delay values.
   * \param[in] freqParam The param to initialize from. 
   */
  explicit TopicStatusParamWithHeader(const ::diagnostic_updater::FrequencyStatusParam& freqParam);

  /**
   * \brief Construct the object from a `TimeStampStatusParam` using default rate values.
   * \param[in] stampParam The param to initialize from. 
   */
  explicit TopicStatusParamWithHeader(const ::diagnostic_updater::TimeStampStatusParam& stampParam);

  /**
   * \brief Construct the object from a `FrequencyStatusParam` and `TimeStampStatusParam`.
   * \param[in] freqParam The rate param to initialize from. 
   * \param[in] stampParam The timestamp delay param to initialize from. 
   */
  TopicStatusParamWithHeader(
    const ::diagnostic_updater::FrequencyStatusParam& freqParam,
    const ::diagnostic_updater::TimeStampStatusParam& stampParam);

  /**
   * \brief Initialize this object from `SimpleTopicStatusParam`. Internal pointers for min/max rate will be used, so
   * the `params` object may cease to exist.
   * \param[in] params The parameters to initialize with.
   */
  TopicStatusParamWithHeader(  // NOLINT(google-explicit-constructor)
    const ::cras::SimpleTopicStatusParamWithHeader& params);
};

/**
 * \brief Helper struct for automatic choosing of the correct topic status parameter object based on whether the message
 * type has a header or not.
 * \tparam Message Type of the message for which parameters are constructed.
 * \note This typedef automatically chooses either `cras::FrequencyStatusParam` or `cras::TopicStatusParamWithHeader`
 * based on the type of the message.
 */
template <typename Message>
using TopicStatusParam = ::std::conditional_t<::ros::message_traits::HasHeader<Message>::value,
  ::cras::TopicStatusParamWithHeader, ::cras::FrequencyStatusParam>;

}