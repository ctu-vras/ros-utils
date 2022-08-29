/**
 * \file
 * \brief Unit tests for diag_utils.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <cras_cpp_common/diag_utils/diagnosed_pub_sub.hpp>
#include <cras_cpp_common/diag_utils/duration_status.h>
#include <cras_cpp_common/diag_utils/duration_status_param.h>
#include <cras_cpp_common/diag_utils/topic_status.hpp>
#include <cras_cpp_common/diag_utils/topic_status_param.hpp>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <cras_cpp_common/time_utils.hpp>

using namespace cras;

TEST(SimpleTopicStatusParam, Types)  // NOLINT
{
  EXPECT_EQ(typeid(SimpleTopicStatusParamNoHeader), typeid(SimpleTopicStatusParam<std_msgs::Header>));
  EXPECT_EQ(typeid(SimpleTopicStatusParamWithHeader), typeid(SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray>));
}

TEST(SimpleTopicStatusParam, BracedInit)  // NOLINT
{
  SimpleTopicStatusParam<std_msgs::Header> h = {1, 2, 3, 4};
  EXPECT_EQ(1.0, h.minRate);
  EXPECT_EQ(2.0, h.maxRate);
  EXPECT_EQ(3.0, h.rateTolerance);
  EXPECT_EQ(4, h.rateWindowSize);

  SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray> d = {1, 2, 3, 4, 5, 6};
  EXPECT_EQ(1.0, d.minRate);
  EXPECT_EQ(2.0, d.maxRate);
  EXPECT_EQ(3.0, d.rateTolerance);
  EXPECT_EQ(4, d.rateWindowSize);
  EXPECT_EQ(5.0, d.minDelay);
  EXPECT_EQ(6.0, d.maxDelay);
}

TEST(SimpleTopicStatusParam, EmptyBracedInit)  // NOLINT
{
  SimpleTopicStatusParam<std_msgs::Header> h = {};
  EXPECT_EQ(0.0, h.minRate);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), h.maxRate);
  EXPECT_EQ(0.1, h.rateTolerance);
  EXPECT_EQ(5, h.rateWindowSize);

  SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray> d = {};
  EXPECT_EQ(0.0, d.minRate);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), d.maxRate);
  EXPECT_EQ(0.1, d.rateTolerance);
  EXPECT_EQ(5, d.rateWindowSize);
  EXPECT_EQ(-1.0, d.minDelay);
  EXPECT_EQ(5.0, d.maxDelay);
}

#ifdef HAS_DESIGNATED_INITIALIZERS
TEST(SimpleTopicStatusParam, DesignatedBracedInit)  // NOLINT
{
  SimpleTopicStatusParam<std_msgs::Header> h = {.rateWindowSize = 10};
  EXPECT_EQ(0.0, h.minRate);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), h.maxRate);
  EXPECT_EQ(0.1, h.rateTolerance);
  EXPECT_EQ(10, h.rateWindowSize);

  SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray> d = {.rateWindowSize = 10, .maxDelay = 10};
  EXPECT_EQ(0.0, d.minRate);
  EXPECT_EQ(std::numeric_limits<double>::infinity(), d.maxRate);
  EXPECT_EQ(0.1, d.rateTolerance);
  EXPECT_EQ(10, d.rateWindowSize);
  EXPECT_EQ(-1.0, d.minDelay);
  EXPECT_EQ(10.0, d.maxDelay);
}
#endif

TEST(TopicStatusParams, Values)  // NOLINT
{
  EXPECT_EQ(1.0, TopicStatusParams::_1Hz.minRate);
  EXPECT_EQ(1.0, TopicStatusParams::_1Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::_1Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::_1Hz.rateWindowSize);

  EXPECT_EQ(10.0, TopicStatusParams::_10Hz.minRate);
  EXPECT_EQ(10.0, TopicStatusParams::_10Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::_10Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::_10Hz.rateWindowSize);

  EXPECT_EQ(100.0, TopicStatusParams::_100Hz.minRate);
  EXPECT_EQ(100.0, TopicStatusParams::_100Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::_100Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::_100Hz.rateWindowSize);

  EXPECT_EQ(1.0, TopicStatusParams::delayed1Hz.minRate);
  EXPECT_EQ(1.0, TopicStatusParams::delayed1Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::delayed1Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::delayed1Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::delayed1Hz.minDelay);
  EXPECT_EQ(5, TopicStatusParams::delayed1Hz.maxDelay);

  EXPECT_EQ(10.0, TopicStatusParams::delayed10Hz.minRate);
  EXPECT_EQ(10.0, TopicStatusParams::delayed10Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::delayed10Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::delayed10Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::delayed10Hz.minDelay);
  EXPECT_EQ(5, TopicStatusParams::delayed10Hz.maxDelay);

  EXPECT_EQ(100.0, TopicStatusParams::delayed100Hz.minRate);
  EXPECT_EQ(100.0, TopicStatusParams::delayed100Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::delayed100Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::delayed100Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::delayed100Hz.minDelay);
  EXPECT_EQ(5, TopicStatusParams::delayed100Hz.maxDelay);

  EXPECT_EQ(1.0, TopicStatusParams::fast1Hz.minRate);
  EXPECT_EQ(1.0, TopicStatusParams::fast1Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::fast1Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::fast1Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::fast1Hz.minDelay);
  EXPECT_EQ(0.1, TopicStatusParams::fast1Hz.maxDelay);

  EXPECT_EQ(10.0, TopicStatusParams::fast10Hz.minRate);
  EXPECT_EQ(10.0, TopicStatusParams::fast10Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::fast10Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::fast10Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::fast10Hz.minDelay);
  EXPECT_EQ(0.1, TopicStatusParams::fast10Hz.maxDelay);

  EXPECT_EQ(100.0, TopicStatusParams::fast100Hz.minRate);
  EXPECT_EQ(100.0, TopicStatusParams::fast100Hz.maxRate);
  EXPECT_EQ(0.1, TopicStatusParams::fast100Hz.rateTolerance);
  EXPECT_EQ(5, TopicStatusParams::fast100Hz.rateWindowSize);
  EXPECT_EQ(-1, TopicStatusParams::fast100Hz.minDelay);
  EXPECT_EQ(0.1, TopicStatusParams::fast100Hz.maxDelay);
}

TEST(TopicStatusParam, NoHeaderConstructors)  // NOLINT
{
  EXPECT_EQ(typeid(cras::FrequencyStatusParam), typeid(TopicStatusParam<std_msgs::Header>));

  double minFreq = 1.0, maxFreq = 2.0;
  TopicStatusParam<std_msgs::Header> p1(&minFreq, &maxFreq);
  TopicStatusParam<std_msgs::Header> p2(&minFreq, &maxFreq, 3.0);
  TopicStatusParam<std_msgs::Header> p3(&minFreq, &maxFreq, 3.0, 4);
  TopicStatusParam<std_msgs::Header> p4(p3);
  TopicStatusParam<std_msgs::Header> p5 = p3;

  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
  EXPECT_EQ(0.1, p1.tolerance_);
  EXPECT_EQ(5, p1.window_size_);
  EXPECT_EQ(1.5, p1.getExpectedRate());
  EXPECT_EQ(1.0, *p2.min_freq_);
  EXPECT_EQ(2.0, *p2.max_freq_);
  EXPECT_EQ(3.0, p2.tolerance_);
  EXPECT_EQ(5, p2.window_size_);
  EXPECT_EQ(1.5, p2.getExpectedRate());
  EXPECT_EQ(1.0, *p3.min_freq_);
  EXPECT_EQ(2.0, *p3.max_freq_);
  EXPECT_EQ(3.0, p3.tolerance_);
  EXPECT_EQ(4, p3.window_size_);
  EXPECT_EQ(1.5, p3.getExpectedRate());
  EXPECT_EQ(1.0, *p4.min_freq_);
  EXPECT_EQ(2.0, *p4.max_freq_);
  EXPECT_EQ(3.0, p4.tolerance_);
  EXPECT_EQ(4, p4.window_size_);
  EXPECT_EQ(1.5, p4.getExpectedRate());
  EXPECT_EQ(1.0, *p5.min_freq_);
  EXPECT_EQ(2.0, *p5.max_freq_);
  EXPECT_EQ(3.0, p5.tolerance_);
  EXPECT_EQ(4, p5.window_size_);
  EXPECT_EQ(1.5, p5.getExpectedRate());

  minFreq = 5.0; maxFreq = 6.0;
  EXPECT_EQ(5.0, *p1.min_freq_);
  EXPECT_EQ(6.0, *p1.max_freq_);
  EXPECT_EQ(5.5, p1.getExpectedRate());
  EXPECT_EQ(5.0, *p2.min_freq_);
  EXPECT_EQ(6.0, *p2.max_freq_);
  EXPECT_EQ(5.5, p2.getExpectedRate());
  EXPECT_EQ(5.0, *p3.min_freq_);
  EXPECT_EQ(6.0, *p3.max_freq_);
  EXPECT_EQ(5.5, p3.getExpectedRate());
  EXPECT_EQ(5.0, *p4.min_freq_);
  EXPECT_EQ(6.0, *p4.max_freq_);
  EXPECT_EQ(5.5, p4.getExpectedRate());
  EXPECT_EQ(5.0, *p5.min_freq_);
  EXPECT_EQ(6.0, *p5.max_freq_);
  EXPECT_EQ(5.5, p5.getExpectedRate());

  minFreq = 1.0; maxFreq = 2.0;

  TopicStatusParam<std_msgs::Header> q1(minFreq, maxFreq);
  TopicStatusParam<std_msgs::Header> q2(minFreq, maxFreq, 3.0);
  TopicStatusParam<std_msgs::Header> q3(minFreq, maxFreq, 3.0, 4);
  TopicStatusParam<std_msgs::Header> q4(q3);
  TopicStatusParam<std_msgs::Header> q5 = q3;

  EXPECT_EQ(1.0, *q1.min_freq_);
  EXPECT_EQ(2.0, *q1.max_freq_);
  EXPECT_EQ(0.1, q1.tolerance_);
  EXPECT_EQ(5, q1.window_size_);
  EXPECT_EQ(1.5, q1.getExpectedRate());
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(3.0, q2.tolerance_);
  EXPECT_EQ(5, q2.window_size_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(3.0, q3.tolerance_);
  EXPECT_EQ(4, q3.window_size_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(3.0, q4.tolerance_);
  EXPECT_EQ(4, q4.window_size_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(3.0, q5.tolerance_);
  EXPECT_EQ(4, q5.window_size_);
  EXPECT_EQ(1.5, q5.getExpectedRate());

  minFreq = 5.0; maxFreq = 6.0;
  EXPECT_EQ(1.0, *q1.min_freq_);
  EXPECT_EQ(2.0, *q1.max_freq_);
  EXPECT_EQ(1.5, q1.getExpectedRate());
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(1.5, q5.getExpectedRate());

  *q1.min_freq_ = 5.0;
  *q1.max_freq_ = 6.0;
  EXPECT_EQ(5.5, q1.getExpectedRate());
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(1.5, q5.getExpectedRate());

  EXPECT_EQ(0.0, *TopicStatusParam<std_msgs::Header>().min_freq_);
  EXPECT_FALSE(std::isfinite(*TopicStatusParam<std_msgs::Header>().max_freq_));

  EXPECT_FALSE(std::isfinite(TopicStatusParam<std_msgs::Header>().getExpectedRate()));
  EXPECT_EQ(5.0, TopicStatusParam<std_msgs::Header>(5.0).getExpectedRate());
  EXPECT_EQ(5.0, TopicStatusParam<std_msgs::Header>(0.0, 5.0).getExpectedRate());
}

TEST(TopicStatusParam, WithHeaderConstructors)  // NOLINT
{
  EXPECT_EQ(typeid(cras::TopicStatusParamWithHeader), typeid(TopicStatusParam<diagnostic_msgs::DiagnosticArray>));

  double minFreq = 1.0, maxFreq = 2.0;
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p1(&minFreq, &maxFreq);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p2(&minFreq, &maxFreq, 3.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p3(&minFreq, &maxFreq, 3.0, 4);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p4(&minFreq, &maxFreq, 3.0, 4, 5.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p5(&minFreq, &maxFreq, 3.0, 4, 5.0, 6.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p6(p5);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p7 = p5;

  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
  EXPECT_EQ(0.1, p1.tolerance_);
  EXPECT_EQ(5, p1.window_size_);
  EXPECT_EQ(1.5, p1.getExpectedRate());
  EXPECT_EQ(-1.0, p1.min_acceptable_);
  EXPECT_EQ(5.0, p1.max_acceptable_);
  EXPECT_EQ(1.0, *p2.min_freq_);
  EXPECT_EQ(2.0, *p2.max_freq_);
  EXPECT_EQ(3.0, p2.tolerance_);
  EXPECT_EQ(5, p2.window_size_);
  EXPECT_EQ(1.5, p2.getExpectedRate());
  EXPECT_EQ(-1.0, p2.min_acceptable_);
  EXPECT_EQ(5.0, p2.max_acceptable_);
  EXPECT_EQ(1.0, *p3.min_freq_);
  EXPECT_EQ(2.0, *p3.max_freq_);
  EXPECT_EQ(3.0, p3.tolerance_);
  EXPECT_EQ(4, p3.window_size_);
  EXPECT_EQ(1.5, p3.getExpectedRate());
  EXPECT_EQ(-1.0, p3.min_acceptable_);
  EXPECT_EQ(5.0, p3.max_acceptable_);
  EXPECT_EQ(1.0, *p4.min_freq_);
  EXPECT_EQ(2.0, *p4.max_freq_);
  EXPECT_EQ(3.0, p4.tolerance_);
  EXPECT_EQ(4, p4.window_size_);
  EXPECT_EQ(1.5, p4.getExpectedRate());
  EXPECT_EQ(5.0, p4.min_acceptable_);
  EXPECT_EQ(5.0, p4.max_acceptable_);
  EXPECT_EQ(1.0, *p5.min_freq_);
  EXPECT_EQ(2.0, *p5.max_freq_);
  EXPECT_EQ(3.0, p5.tolerance_);
  EXPECT_EQ(4, p5.window_size_);
  EXPECT_EQ(1.5, p5.getExpectedRate());
  EXPECT_EQ(5.0, p5.min_acceptable_);
  EXPECT_EQ(6.0, p5.max_acceptable_);
  EXPECT_EQ(1.0, *p6.min_freq_);
  EXPECT_EQ(2.0, *p6.max_freq_);
  EXPECT_EQ(3.0, p6.tolerance_);
  EXPECT_EQ(4, p6.window_size_);
  EXPECT_EQ(1.5, p6.getExpectedRate());
  EXPECT_EQ(5.0, p6.min_acceptable_);
  EXPECT_EQ(6.0, p6.max_acceptable_);
  EXPECT_EQ(1.0, *p7.min_freq_);
  EXPECT_EQ(2.0, *p7.max_freq_);
  EXPECT_EQ(3.0, p7.tolerance_);
  EXPECT_EQ(4, p7.window_size_);
  EXPECT_EQ(1.5, p7.getExpectedRate());
  EXPECT_EQ(5.0, p7.min_acceptable_);
  EXPECT_EQ(6.0, p7.max_acceptable_);

  minFreq = 5.0; maxFreq = 6.0;
  EXPECT_EQ(5.0, *p1.min_freq_);
  EXPECT_EQ(6.0, *p1.max_freq_);
  EXPECT_EQ(5.5, p1.getExpectedRate());
  EXPECT_EQ(5.0, *p2.min_freq_);
  EXPECT_EQ(6.0, *p2.max_freq_);
  EXPECT_EQ(5.5, p2.getExpectedRate());
  EXPECT_EQ(5.0, *p3.min_freq_);
  EXPECT_EQ(6.0, *p3.max_freq_);
  EXPECT_EQ(5.5, p3.getExpectedRate());
  EXPECT_EQ(5.0, *p4.min_freq_);
  EXPECT_EQ(6.0, *p4.max_freq_);
  EXPECT_EQ(5.5, p4.getExpectedRate());
  EXPECT_EQ(5.0, *p5.min_freq_);
  EXPECT_EQ(6.0, *p5.max_freq_);
  EXPECT_EQ(5.5, p5.getExpectedRate());
  EXPECT_EQ(5.0, *p6.min_freq_);
  EXPECT_EQ(6.0, *p6.max_freq_);
  EXPECT_EQ(5.5, p6.getExpectedRate());
  EXPECT_EQ(5.0, *p7.min_freq_);
  EXPECT_EQ(6.0, *p7.max_freq_);
  EXPECT_EQ(5.5, p7.getExpectedRate());

  minFreq = 1.0; maxFreq = 2.0;

  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q1(minFreq, maxFreq);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q2(minFreq, maxFreq, 3.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q3(minFreq, maxFreq, 3.0, 4);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q4(minFreq, maxFreq, 3.0, 4, 5.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q5(minFreq, maxFreq, 3.0, 4, 5.0, 6.0);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q6(q5);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> q7 = q5;

  EXPECT_EQ(1.0, *q1.min_freq_);
  EXPECT_EQ(2.0, *q1.max_freq_);
  EXPECT_EQ(0.1, q1.tolerance_);
  EXPECT_EQ(5, q1.window_size_);
  EXPECT_EQ(1.5, q1.getExpectedRate());
  EXPECT_EQ(-1.0, q1.min_acceptable_);
  EXPECT_EQ(5.0, q1.max_acceptable_);
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(3.0, q2.tolerance_);
  EXPECT_EQ(5, q2.window_size_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(-1.0, q2.min_acceptable_);
  EXPECT_EQ(5.0, q2.max_acceptable_);
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(3.0, q3.tolerance_);
  EXPECT_EQ(4, q3.window_size_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(-1.0, q3.min_acceptable_);
  EXPECT_EQ(5.0, q3.max_acceptable_);
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(3.0, q4.tolerance_);
  EXPECT_EQ(4, q4.window_size_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(5.0, q4.min_acceptable_);
  EXPECT_EQ(5.0, q4.max_acceptable_);
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(3.0, q5.tolerance_);
  EXPECT_EQ(4, q5.window_size_);
  EXPECT_EQ(1.5, q5.getExpectedRate());
  EXPECT_EQ(5.0, q5.min_acceptable_);
  EXPECT_EQ(6.0, q5.max_acceptable_);
  EXPECT_EQ(1.0, *q6.min_freq_);
  EXPECT_EQ(2.0, *q6.max_freq_);
  EXPECT_EQ(3.0, q6.tolerance_);
  EXPECT_EQ(4, q6.window_size_);
  EXPECT_EQ(1.5, q6.getExpectedRate());
  EXPECT_EQ(5.0, q6.min_acceptable_);
  EXPECT_EQ(6.0, q6.max_acceptable_);
  EXPECT_EQ(1.0, *q7.min_freq_);
  EXPECT_EQ(2.0, *q7.max_freq_);
  EXPECT_EQ(3.0, q7.tolerance_);
  EXPECT_EQ(4, q7.window_size_);
  EXPECT_EQ(1.5, q7.getExpectedRate());
  EXPECT_EQ(5.0, q7.min_acceptable_);
  EXPECT_EQ(6.0, q7.max_acceptable_);

  minFreq = 5.0; maxFreq = 6.0;
  EXPECT_EQ(1.0, *q1.min_freq_);
  EXPECT_EQ(2.0, *q1.max_freq_);
  EXPECT_EQ(1.5, q1.getExpectedRate());
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(1.5, q5.getExpectedRate());
  EXPECT_EQ(1.0, *q6.min_freq_);
  EXPECT_EQ(2.0, *q6.max_freq_);
  EXPECT_EQ(1.5, q6.getExpectedRate());
  EXPECT_EQ(1.0, *q7.min_freq_);
  EXPECT_EQ(2.0, *q7.max_freq_);
  EXPECT_EQ(1.5, q7.getExpectedRate());

  *q1.min_freq_ = 5.0;
  *q1.max_freq_ = 6.0;
  EXPECT_EQ(5.5, q1.getExpectedRate());
  EXPECT_EQ(1.0, *q2.min_freq_);
  EXPECT_EQ(2.0, *q2.max_freq_);
  EXPECT_EQ(1.5, q2.getExpectedRate());
  EXPECT_EQ(1.0, *q3.min_freq_);
  EXPECT_EQ(2.0, *q3.max_freq_);
  EXPECT_EQ(1.5, q3.getExpectedRate());
  EXPECT_EQ(1.0, *q4.min_freq_);
  EXPECT_EQ(2.0, *q4.max_freq_);
  EXPECT_EQ(1.5, q4.getExpectedRate());
  EXPECT_EQ(1.0, *q5.min_freq_);
  EXPECT_EQ(2.0, *q5.max_freq_);
  EXPECT_EQ(1.5, q5.getExpectedRate());
  EXPECT_EQ(1.0, *q6.min_freq_);
  EXPECT_EQ(2.0, *q6.max_freq_);
  EXPECT_EQ(1.5, q6.getExpectedRate());
  EXPECT_EQ(1.0, *q7.min_freq_);
  EXPECT_EQ(2.0, *q7.max_freq_);
  EXPECT_EQ(1.5, q7.getExpectedRate());

  EXPECT_EQ(0.0, *TopicStatusParam<diagnostic_msgs::DiagnosticArray>().min_freq_);
  EXPECT_FALSE(std::isfinite(*TopicStatusParam<diagnostic_msgs::DiagnosticArray>().max_freq_));

  EXPECT_FALSE(std::isfinite(TopicStatusParam<diagnostic_msgs::DiagnosticArray>().getExpectedRate()));
  EXPECT_EQ(5.0, TopicStatusParam<diagnostic_msgs::DiagnosticArray>(5.0).getExpectedRate());
  EXPECT_EQ(5.0, TopicStatusParam<diagnostic_msgs::DiagnosticArray>(0.0, 5.0).getExpectedRate());
}

TEST(TopicStatusParam, NoHeaderFromSimple)  // NOLINT
{
  SimpleTopicStatusParam<std_msgs::Header> param({1.0, 2.0, 3.0, 4});
  TopicStatusParam<std_msgs::Header> p1(param);
  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
  EXPECT_EQ(3.0, p1.tolerance_);
  EXPECT_EQ(4, p1.window_size_);

  param.minRate = 5.0;
  param.maxRate = 6.0;
  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
}

TEST(TopicStatusParam, WithHeaderFromSimple)  // NOLINT
{
  SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray> param({1.0, 2.0, 3.0, 4, 5.0, 6.0});
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> p1(param);
  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
  EXPECT_EQ(3.0, p1.tolerance_);
  EXPECT_EQ(4, p1.window_size_);
  EXPECT_EQ(5.0, p1.min_acceptable_);
  EXPECT_EQ(6.0, p1.max_acceptable_);

  param.minRate = 5.0;
  param.maxRate = 6.0;
  EXPECT_EQ(1.0, *p1.min_freq_);
  EXPECT_EQ(2.0, *p1.max_freq_);
}

TEST(TopicStatus, NoHeaderConstructors)  // NOLINT
{
  TopicStatus<std_msgs::Header> s1("a", 1.0, 2.0, 3.0, 4);
  TopicStatus<std_msgs::Header> s2("a", TopicStatusParam<std_msgs::Header>(1.0, 2.0, 3.0, 4));
  TopicStatus<std_msgs::Header> s3("a", SimpleTopicStatusParam<std_msgs::Header>({1.0, 2.0, 3.0, 4}));
  TopicStatus<std_msgs::Header> s4("a", {1.0, 2.0, 3.0, 4});
#ifdef HAS_DESIGNATED_INITIALIZERS
  TopicStatus<std_msgs::Header> s5("a", {.maxRate = 10.0, .rateTolerance = 1.0});
#endif

  EXPECT_EQ("a", s1.getName());
  EXPECT_EQ(ros::Rate(1.0), s1.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s1.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s1.getExpectedRate());
  EXPECT_EQ(3.0, s1.getRateTolerance());
  EXPECT_EQ(4, s1.getRateWindowSize());

  EXPECT_EQ("a", s2.getName());
  EXPECT_EQ(ros::Rate(1.0), s2.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s2.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s2.getExpectedRate());
  EXPECT_EQ(3.0, s2.getRateTolerance());
  EXPECT_EQ(4, s2.getRateWindowSize());

  EXPECT_EQ("a", s3.getName());
  EXPECT_EQ(ros::Rate(1.0), s3.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s3.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s3.getExpectedRate());
  EXPECT_EQ(3.0, s3.getRateTolerance());
  EXPECT_EQ(4, s3.getRateWindowSize());

  EXPECT_EQ("a", s4.getName());
  EXPECT_EQ(ros::Rate(1.0), s4.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s4.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s4.getExpectedRate());
  EXPECT_EQ(3.0, s4.getRateTolerance());
  EXPECT_EQ(4, s4.getRateWindowSize());

#ifdef HAS_DESIGNATED_INITIALIZERS
  EXPECT_EQ("a", s5.getName());
  EXPECT_EQ(ros::Rate(ros::DURATION_MAX), s5.getMinRate());
  EXPECT_EQ(ros::Rate(10.0), s5.getMaxRate());
  EXPECT_EQ(ros::Rate(10.0), s5.getExpectedRate());
  EXPECT_EQ(1.0, s5.getRateTolerance());
  EXPECT_EQ(5, s5.getRateWindowSize());
#endif
}

TEST(TopicStatus, WithHeaderConstructors)  // NOLINT
{
  TopicStatus<diagnostic_msgs::DiagnosticArray> s1("a", 1.0, 2.0, 3.0, 4, 5.0, 6.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> s2("a",
    TopicStatusParam<diagnostic_msgs::DiagnosticArray>(1.0, 2.0, 3.0, 4, 5.0, 6.0));
  TopicStatus<diagnostic_msgs::DiagnosticArray> s3("a",
    SimpleTopicStatusParam<diagnostic_msgs::DiagnosticArray>({1.0, 2.0, 3.0, 4, 5.0, 6.0}));
  TopicStatus<diagnostic_msgs::DiagnosticArray> s4("a", {1.0, 2.0, 3.0, 4, 5.0, 6.0});
#ifdef HAS_DESIGNATED_INITIALIZERS
  TopicStatus<diagnostic_msgs::DiagnosticArray> s5("a", {.maxRate = 10.0, .rateTolerance = 1.0});
#endif

  EXPECT_EQ("a", s1.getName());
  EXPECT_EQ(ros::Rate(1.0), s1.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s1.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s1.getExpectedRate());
  EXPECT_EQ(3.0, s1.getRateTolerance());
  EXPECT_EQ(4, s1.getRateWindowSize());
  EXPECT_EQ(ros::Duration(5.0), s1.getMinDelay());
  EXPECT_EQ(ros::Duration(6.0), s1.getMaxDelay());

  EXPECT_EQ("a", s2.getName());
  EXPECT_EQ(ros::Rate(1.0), s2.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s2.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s2.getExpectedRate());
  EXPECT_EQ(3.0, s2.getRateTolerance());
  EXPECT_EQ(4, s2.getRateWindowSize());
  EXPECT_EQ(ros::Duration(5.0), s2.getMinDelay());
  EXPECT_EQ(ros::Duration(6.0), s2.getMaxDelay());

  EXPECT_EQ("a", s3.getName());
  EXPECT_EQ(ros::Rate(1.0), s3.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s3.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s3.getExpectedRate());
  EXPECT_EQ(3.0, s3.getRateTolerance());
  EXPECT_EQ(4, s3.getRateWindowSize());
  EXPECT_EQ(ros::Duration(5.0), s3.getMinDelay());
  EXPECT_EQ(ros::Duration(6.0), s3.getMaxDelay());

  EXPECT_EQ("a", s4.getName());
  EXPECT_EQ(ros::Rate(1.0), s4.getMinRate());
  EXPECT_EQ(ros::Rate(2.0), s4.getMaxRate());
  EXPECT_EQ(ros::Rate(1.5), s4.getExpectedRate());
  EXPECT_EQ(3.0, s4.getRateTolerance());
  EXPECT_EQ(4, s4.getRateWindowSize());
  EXPECT_EQ(ros::Duration(5.0), s4.getMinDelay());
  EXPECT_EQ(ros::Duration(6.0), s4.getMaxDelay());

#ifdef HAS_DESIGNATED_INITIALIZERS
  EXPECT_EQ("a", s5.getName());
  EXPECT_EQ(ros::Rate(ros::DURATION_MAX), s5.getMinRate());
  EXPECT_EQ(ros::Rate(10.0), s5.getMaxRate());
  EXPECT_EQ(ros::Rate(10.0), s5.getExpectedRate());
  EXPECT_EQ(1.0, s5.getRateTolerance());
  EXPECT_EQ(5, s5.getRateWindowSize());
  EXPECT_EQ(ros::Duration(-1.0), s5.getMinDelay());
  EXPECT_EQ(ros::Duration(5.0), s5.getMaxDelay());
#endif
}

TEST(TopicStatus, TickAndUpdateOk)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 10.0, 10.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 10.0, 10.0, 0.1, 5, 0.0, 0.2);

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateDelayed)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 10.0, 10.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 10.0, 10.0, 0.1, 5, 0.0, 0.2);

  auto prevTime = ros::Time(9);
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime += ros::Duration(0.1);
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("Timestamps too far in past seen.", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateLowRate)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 20.0, 20.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 20.0, 20.0, 0.1, 5, 0.0, 0.2);

  auto prevTime = ros::Time(10);
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime += ros::Duration(0.1);
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, withHeaderWrapper.level);
  EXPECT_EQ("Frequency too low.", noHeaderWrapper.message);
  EXPECT_EQ("Frequency too low.", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateHighRate)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 5.0, 5.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 5.0, 5.0, 0.1, 5, 0.0, 0.2);

  auto prevTime = ros::Time(10);
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime += ros::Duration(0.1);
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, withHeaderWrapper.level);
  EXPECT_EQ("Frequency too high.", noHeaderWrapper.message);
  EXPECT_EQ("Frequency too high.", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateLowRateDelayed)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 20.0, 20.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 20.0, 20.0, 0.1, 5, 0.0, 0.2);

  auto prevTime = ros::Time(9);
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime += ros::Duration(0.1);
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, withHeaderWrapper.level);
  EXPECT_EQ("Frequency too low.", noHeaderWrapper.message);
  EXPECT_EQ("Frequency too low.; Timestamps too far in past seen.", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateWithMessage)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 10.0, 10.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 10.0, 10.0, 0.1, 5, 0.0, 0.2);

  std_msgs::Header header;
  diagnostic_msgs::DiagnosticArray diagMsg;

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    header.stamp = prevTime;
    diagMsg.header = header;
    noHeader.tick(header);
    withHeader.tick(diagMsg);
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateWithMessagePtr)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 10.0, 10.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 10.0, 10.0, 0.1, 5, 0.0, 0.2);

  auto header = boost::make_shared<std_msgs::Header>();
  auto diagMsg = boost::make_shared<diagnostic_msgs::DiagnosticArray>();

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    header->stamp = prevTime;
    diagMsg->header = *header;
    noHeader.tick(header);
    withHeader.tick(diagMsg);
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateWithMessageEvent)  // NOLINT
{
  ros::Time::setNow({10, 0});

  TopicStatus<std_msgs::Header> noHeader("a", 10.0, 10.0);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", 10.0, 10.0, 0.1, 5, 0.0, 0.2);

  auto header = boost::make_shared<std_msgs::Header>();
  auto diagMsg = boost::make_shared<diagnostic_msgs::DiagnosticArray>();

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    header->stamp = prevTime;
    diagMsg->header = *header;
    noHeader.tick(ros::MessageEvent<std_msgs::Header>(header));
    withHeader.tick(ros::MessageEvent<diagnostic_msgs::DiagnosticArray>(diagMsg));
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("", withHeaderWrapper.message);
}

TEST(TopicStatus, TickAndUpdateParamsUsePointer)  // NOLINT
{
  ros::Time::setNow({10, 0});

  double minFreq = 20.0;
  double maxFreq = 20.0;
  TopicStatusParam<std_msgs::Header> noHeaderParam(&minFreq, &maxFreq);
  TopicStatusParam<diagnostic_msgs::DiagnosticArray> withHeaderParam(&minFreq, &maxFreq, 0.1, 5, 0.0, 0.2);
  TopicStatus<std_msgs::Header> noHeader("a", noHeaderParam);
  TopicStatus<diagnostic_msgs::DiagnosticArray> withHeader("a", withHeaderParam);

  minFreq = maxFreq = 10.0;

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    noHeader.tick(prevTime);
    withHeader.tick(prevTime);
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper noHeaderWrapper;
  diagnostic_updater::DiagnosticStatusWrapper withHeaderWrapper;
  noHeader.run(noHeaderWrapper);
  withHeader.run(withHeaderWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, noHeaderWrapper.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, withHeaderWrapper.level);
  EXPECT_EQ("", noHeaderWrapper.message);
  EXPECT_EQ("", withHeaderWrapper.message);
}

TEST(SimpleDurationStatusParam, BracedInit)  // NOLINT
{
  SimpleDurationStatusParam p = {{1, 0}, {2, 0}, 3, 4, true};
  EXPECT_EQ(ros::Duration(1, 0), p.minDuration);
  EXPECT_EQ(ros::Duration(2, 0), p.maxDuration);
  EXPECT_EQ(3.0, p.tolerance);
  EXPECT_EQ(4, p.windowSize);
  EXPECT_EQ(true, p.noEventsIsOk);
}

TEST(SimpleDurationStatusParam, EmptyBracedInit)  // NOLINT
{
  SimpleDurationStatusParam p = {};
  EXPECT_EQ(ros::Duration(0, 0), p.minDuration);
  EXPECT_EQ(ros::DURATION_MAX, p.maxDuration);
  EXPECT_EQ(0.1, p.tolerance);
  EXPECT_EQ(5u, p.windowSize);
  EXPECT_EQ(true, p.noEventsIsOk);
}

#ifdef HAS_DESIGNATED_INITIALIZERS
TEST(SimpleDurationStatusParam, DesignatedBracedInit)  // NOLINT
{
  SimpleDurationStatusParam p = {.windowSize = 10};
  EXPECT_EQ(ros::Duration(0, 0), p.minDuration);
  EXPECT_EQ(ros::DURATION_MAX, p.maxDuration);
  EXPECT_EQ(0.1, p.tolerance);
  EXPECT_EQ(10u, p.windowSize);
  EXPECT_EQ(true, p.noEventsIsOk);
}
#endif

TEST(DurationStatusParam, Init)  // NOLINT
{
  DurationStatusParam p({1, 0}, {2, 0}, 3, 4, true);
  EXPECT_EQ(ros::Duration(1, 0), p.minDuration);
  EXPECT_EQ(ros::Duration(2, 0), p.maxDuration);
  EXPECT_EQ(3.0, p.tolerance);
  EXPECT_EQ(4, p.windowSize);
  EXPECT_EQ(true, p.noEventsIsOk);
}

TEST(DurationStatusParam, EmptyInit)  // NOLINT
{
  DurationStatusParam p;
  EXPECT_EQ(ros::Duration(0, 0), p.minDuration);
  EXPECT_EQ(ros::DURATION_MAX, p.maxDuration);
  EXPECT_EQ(0.1, p.tolerance);
  EXPECT_EQ(5u, p.windowSize);
  EXPECT_EQ(true, p.noEventsIsOk);
}

TEST(DurationStatus, Constructors)  // NOLINT
{
  XmlRpc::XmlRpcValue xmlParams;
  xmlParams.begin();  // morph into a struct
  xmlParams["min_duration"] = 1.0;
  xmlParams["max_duration"] = 2.0;
  xmlParams["tolerance"] = 3.0;
  xmlParams["window_size"] = 4;
  xmlParams["no_events_is_ok"] = true;
  auto paramsAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(xmlParams, "");
  auto params = std::make_shared<cras::BoundParamHelper>(std::make_shared<cras::NodeLogHelper>(), paramsAdapter);

  DurationStatus s1("a", ros::Duration(1, 0), ros::Duration(2, 0), 3.0, 4, true);
  DurationStatus s2("a", DurationStatusParam(ros::Duration(1, 0), ros::Duration(2, 0), 3.0, 4, true));
  DurationStatus s3("a", SimpleDurationStatusParam({ros::Duration(1, 0), ros::Duration(2, 0), 3.0, 4, true}));
  DurationStatus s4("a", params, DurationStatusParam());
  DurationStatus s5("a", params, SimpleDurationStatusParam());

  EXPECT_EQ("a", s1.getName());
  EXPECT_EQ(ros::Duration(1, 0), s1.getMinDuration());
  EXPECT_EQ(ros::Duration(2, 0), s1.getMaxDuration());
  EXPECT_EQ(3.0, s1.getTolerance());
  EXPECT_EQ(4, s1.getWindowSize());

  EXPECT_EQ("a", s2.getName());
  EXPECT_EQ(ros::Duration(1, 0), s2.getMinDuration());
  EXPECT_EQ(ros::Duration(2, 0), s2.getMaxDuration());
  EXPECT_EQ(3.0, s2.getTolerance());
  EXPECT_EQ(4, s2.getWindowSize());

  EXPECT_EQ("a", s3.getName());
  EXPECT_EQ(ros::Duration(1, 0), s3.getMinDuration());
  EXPECT_EQ(ros::Duration(2, 0), s3.getMaxDuration());
  EXPECT_EQ(3.0, s3.getTolerance());
  EXPECT_EQ(4, s3.getWindowSize());

  EXPECT_EQ("a", s4.getName());
  EXPECT_EQ(ros::Duration(1, 0), s4.getMinDuration());
  EXPECT_EQ(ros::Duration(2, 0), s4.getMaxDuration());
  EXPECT_EQ(3.0, s4.getTolerance());
  EXPECT_EQ(4, s4.getWindowSize());

  EXPECT_EQ("a", s5.getName());
  EXPECT_EQ(ros::Duration(1, 0), s5.getMinDuration());
  EXPECT_EQ(ros::Duration(2, 0), s5.getMaxDuration());
  EXPECT_EQ(3.0, s5.getTolerance());
  EXPECT_EQ(4, s5.getWindowSize());
}

TEST(DurationStatus, TickAndUpdateOk)  // NOLINT
{
  ros::Time::setNow({10, 0});

  DurationStatus status("a", {10, 0}, {12, 0});

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    status.start(prevTime);
    status.stop(ros::Time(time.toSec() * 2));
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper statusWrapper;
  status.run(statusWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, statusWrapper.level);
  EXPECT_EQ("Duration within limits.", statusWrapper.message);
  EXPECT_EQ(10, statusWrapper.values.size());
  EXPECT_EQ("Events in window", statusWrapper.values[0].key);
  EXPECT_EQ("10", statusWrapper.values[0].value);
  EXPECT_EQ("Events since startup", statusWrapper.values[1].key);
  EXPECT_EQ("10", statusWrapper.values[1].value);
  EXPECT_EQ("Duration of window (s)", statusWrapper.values[2].key);
  EXPECT_EQ("1.000000", statusWrapper.values[2].value);
  EXPECT_EQ("Minimum observed duration (s)", statusWrapper.values[3].key);
  EXPECT_EQ("10.200000000", statusWrapper.values[3].value);
  EXPECT_EQ("Maximum observed duration (s)", statusWrapper.values[4].key);
  EXPECT_EQ("11.100000000", statusWrapper.values[4].value);
  EXPECT_EQ("Mean observed duration (s)", statusWrapper.values[5].key);
  EXPECT_EQ("10.650000000", statusWrapper.values[5].value);
  EXPECT_EQ("Observed duration standard deviation (s)", statusWrapper.values[6].key);
  EXPECT_EQ("0.302765036", statusWrapper.values[6].value);
  EXPECT_EQ("Minimum acceptable duration (s)", statusWrapper.values[7].key);
  EXPECT_EQ("9.000000000", statusWrapper.values[7].value);
  EXPECT_EQ("Maximum acceptable duration (s)", statusWrapper.values[8].key);
  EXPECT_EQ("13.200000000", statusWrapper.values[8].value);
  EXPECT_EQ("Time mode", statusWrapper.values[9].key);
  EXPECT_EQ("Sim time", statusWrapper.values[9].value);
}

TEST(DurationStatus, TickAndUpdateOkEqualMinMax)  // NOLINT
{
  ros::Time::setNow({10, 0});

  DurationStatus status("a", {11, 0}, {11, 0});

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    status.start(prevTime);
    status.stop(ros::Time(time.toSec() * 2));
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper statusWrapper;
  status.run(statusWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, statusWrapper.level);
  EXPECT_EQ("Duration within limits.", statusWrapper.message);
  EXPECT_EQ(11, statusWrapper.values.size());
  EXPECT_EQ("Events in window", statusWrapper.values[0].key);
  EXPECT_EQ("10", statusWrapper.values[0].value);
  EXPECT_EQ("Events since startup", statusWrapper.values[1].key);
  EXPECT_EQ("10", statusWrapper.values[1].value);
  EXPECT_EQ("Duration of window (s)", statusWrapper.values[2].key);
  EXPECT_EQ("1.000000", statusWrapper.values[2].value);
  EXPECT_EQ("Minimum observed duration (s)", statusWrapper.values[3].key);
  EXPECT_EQ("10.200000000", statusWrapper.values[3].value);
  EXPECT_EQ("Maximum observed duration (s)", statusWrapper.values[4].key);
  EXPECT_EQ("11.100000000", statusWrapper.values[4].value);
  EXPECT_EQ("Mean observed duration (s)", statusWrapper.values[5].key);
  EXPECT_EQ("10.650000000", statusWrapper.values[5].value);
  EXPECT_EQ("Observed duration standard deviation (s)", statusWrapper.values[6].key);
  EXPECT_EQ("0.302765036", statusWrapper.values[6].value);
  EXPECT_EQ("Target duration (s)", statusWrapper.values[7].key);
  EXPECT_EQ("11.000000000", statusWrapper.values[7].value);
  EXPECT_EQ("Minimum acceptable duration (s)", statusWrapper.values[8].key);
  EXPECT_EQ("9.900000000", statusWrapper.values[8].value);
  EXPECT_EQ("Maximum acceptable duration (s)", statusWrapper.values[9].key);
  EXPECT_EQ("12.100000000", statusWrapper.values[9].value);
  EXPECT_EQ("Time mode", statusWrapper.values[10].key);
  EXPECT_EQ("Sim time", statusWrapper.values[10].value);
}

TEST(DurationStatus, TickAndUpdateTooShort)  // NOLINT
{
  ros::Time::setNow({10, 0});

  DurationStatus status("a", {20, 0}, {30, 0});

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    status.start(prevTime);
    status.stop(ros::Time(time.toSec() * 2));
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper statusWrapper;
  status.run(statusWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, statusWrapper.level);
  EXPECT_EQ("Duration too short.", statusWrapper.message);
  EXPECT_EQ(10, statusWrapper.values.size());
  EXPECT_EQ("Events in window", statusWrapper.values[0].key);
  EXPECT_EQ("10", statusWrapper.values[0].value);
  EXPECT_EQ("Events since startup", statusWrapper.values[1].key);
  EXPECT_EQ("10", statusWrapper.values[1].value);
  EXPECT_EQ("Duration of window (s)", statusWrapper.values[2].key);
  EXPECT_EQ("1.000000", statusWrapper.values[2].value);
  EXPECT_EQ("Minimum observed duration (s)", statusWrapper.values[3].key);
  EXPECT_EQ("10.200000000", statusWrapper.values[3].value);
  EXPECT_EQ("Maximum observed duration (s)", statusWrapper.values[4].key);
  EXPECT_EQ("11.100000000", statusWrapper.values[4].value);
  EXPECT_EQ("Mean observed duration (s)", statusWrapper.values[5].key);
  EXPECT_EQ("10.650000000", statusWrapper.values[5].value);
  EXPECT_EQ("Observed duration standard deviation (s)", statusWrapper.values[6].key);
  EXPECT_EQ("0.302765036", statusWrapper.values[6].value);
  EXPECT_EQ("Minimum acceptable duration (s)", statusWrapper.values[7].key);
  EXPECT_EQ("18.000000000", statusWrapper.values[7].value);
  EXPECT_EQ("Maximum acceptable duration (s)", statusWrapper.values[8].key);
  EXPECT_EQ("33.000000000", statusWrapper.values[8].value);
  EXPECT_EQ("Time mode", statusWrapper.values[9].key);
  EXPECT_EQ("Sim time", statusWrapper.values[9].value);
}

TEST(DurationStatus, TickAndUpdateTooLong)  // NOLINT
{
  ros::Time::setNow({10, 0});

  DurationStatus status("a", {0, 0}, ros::Duration(0.01));

  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    status.start(prevTime);
    status.stop(ros::Time(time.toSec() * 2));
    prevTime = time;
  }

  diagnostic_updater::DiagnosticStatusWrapper statusWrapper;
  status.run(statusWrapper);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, statusWrapper.level);
  EXPECT_EQ("Duration too long.", statusWrapper.message);
  EXPECT_EQ(10, statusWrapper.values.size());
  EXPECT_EQ("Events in window", statusWrapper.values[0].key);
  EXPECT_EQ("10", statusWrapper.values[0].value);
  EXPECT_EQ("Events since startup", statusWrapper.values[1].key);
  EXPECT_EQ("10", statusWrapper.values[1].value);
  EXPECT_EQ("Duration of window (s)", statusWrapper.values[2].key);
  EXPECT_EQ("1.000000", statusWrapper.values[2].value);
  EXPECT_EQ("Minimum observed duration (s)", statusWrapper.values[3].key);
  EXPECT_EQ("10.200000000", statusWrapper.values[3].value);
  EXPECT_EQ("Maximum observed duration (s)", statusWrapper.values[4].key);
  EXPECT_EQ("11.100000000", statusWrapper.values[4].value);
  EXPECT_EQ("Mean observed duration (s)", statusWrapper.values[5].key);
  EXPECT_EQ("10.650000000", statusWrapper.values[5].value);
  EXPECT_EQ("Observed duration standard deviation (s)", statusWrapper.values[6].key);
  EXPECT_EQ("0.302765036", statusWrapper.values[6].value);
  EXPECT_EQ("Minimum acceptable duration (s)", statusWrapper.values[7].key);
  EXPECT_EQ("0.000000000", statusWrapper.values[7].value);
  EXPECT_EQ("Maximum acceptable duration (s)", statusWrapper.values[8].key);
  EXPECT_EQ("0.011000000", statusWrapper.values[8].value);
  EXPECT_EQ("Time mode", statusWrapper.values[9].key);
  EXPECT_EQ("Sim time", statusWrapper.values[9].value);
}

TEST(DiagnosedPubSub, ConstructorsNoHeader)  // NOLINT
{
  auto diag1 = std::make_shared<TopicStatus<std_msgs::Header>>("a");
  DiagnosedPubSub<std_msgs::Header> pubSub1(diag1);
  ASSERT_NE(nullptr, pubSub1.getDiagnosticTask().get());
  EXPECT_EQ(diag1.get(), pubSub1.getDiagnosticTask().get());

  TopicStatusParam<std_msgs::Header> param1;
  DiagnosedPubSub<std_msgs::Header> pubSub2("a", param1);
  ASSERT_NE(nullptr, pubSub2.getDiagnosticTask().get());
  EXPECT_EQ(*param1.min_freq_, frequency(pubSub2.getDiagnosticTask()->getMinRate(), true));
  EXPECT_EQ(*param1.max_freq_, frequency(pubSub2.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(param1.tolerance_, pubSub2.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(param1.window_size_, pubSub2.getDiagnosticTask()->getRateWindowSize());

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  ros::NodeHandle nh;
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub3(params);
  ASSERT_NE(nullptr, pubSub3.getDiagnosticTask().get());
  EXPECT_EQ(0.0, frequency(pubSub3.getDiagnosticTask()->getMinRate(), true));
  EXPECT_EQ(std::numeric_limits<double>::infinity(), frequency(pubSub3.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(0.1, pubSub3.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(5u, pubSub3.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub4(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub4.getDiagnosticTask().get());
  EXPECT_EQ(1.0, frequency(pubSub4.getDiagnosticTask()->getMinRate()));
  EXPECT_EQ(2.0, frequency(pubSub4.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(3.0, pubSub4.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(4u, pubSub4.getDiagnosticTask()->getRateWindowSize());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub5(params);
  ASSERT_NE(nullptr, pubSub5.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub5.getDiagnosticTask()->getMinRate(), true));
  EXPECT_NEAR(11.0, frequency(pubSub5.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub5.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub5.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub6(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub6.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub6.getDiagnosticTask()->getMinRate()));
  EXPECT_NEAR(11.0, frequency(pubSub6.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub6.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub6.getDiagnosticTask()->getRateWindowSize());

  nodeParams["rate"].clear();
  nodeParams["rate"]["desired"] = 9.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;
  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub7(params);
  ASSERT_NE(nullptr, pubSub7.getDiagnosticTask().get());
  EXPECT_NEAR(9.0, frequency(pubSub7.getDiagnosticTask()->getMinRate(), true), 1e-6);
  EXPECT_NEAR(9.0, frequency(pubSub7.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub7.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub7.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub8(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub8.getDiagnosticTask().get());
  EXPECT_NEAR(9.0, frequency(pubSub8.getDiagnosticTask()->getMinRate()), 1e-6);
  EXPECT_NEAR(9.0, frequency(pubSub8.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub8.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub8.getDiagnosticTask()->getRateWindowSize());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub9(params);
  ASSERT_NE(nullptr, pubSub9.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub9.getDiagnosticTask()->getMinRate(), true));
  EXPECT_NEAR(11.0, frequency(pubSub9.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub9.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub9.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub10(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub10.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub10.getDiagnosticTask()->getMinRate()));
  EXPECT_NEAR(11.0, frequency(pubSub10.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub10.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub10.getDiagnosticTask()->getRateWindowSize());
}

TEST(DiagnosedPubSub, ConstructorsWithHeader)  // NOLINT
{
  auto diag1 = std::make_shared<TopicStatus<diagnostic_msgs::DiagnosticArray>>("a");
  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub1(diag1);
  ASSERT_NE(nullptr, pubSub1.getDiagnosticTask().get());
  EXPECT_EQ(diag1.get(), pubSub1.getDiagnosticTask().get());

  TopicStatusParam<diagnostic_msgs::DiagnosticArray> param1;
  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub2("a", param1);
  ASSERT_NE(nullptr, pubSub2.getDiagnosticTask().get());
  EXPECT_EQ(*param1.min_freq_, frequency(pubSub2.getDiagnosticTask()->getMinRate(), true));
  EXPECT_EQ(*param1.max_freq_, frequency(pubSub2.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(param1.tolerance_, pubSub2.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(param1.window_size_, pubSub2.getDiagnosticTask()->getRateWindowSize());
  EXPECT_EQ(ros::Duration(param1.min_acceptable_), pubSub2.getDiagnosticTask()->getMinDelay());
  EXPECT_EQ(ros::Duration(param1.max_acceptable_), pubSub2.getDiagnosticTask()->getMaxDelay());

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  ros::NodeHandle nh;
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub3(params);
  ASSERT_NE(nullptr, pubSub3.getDiagnosticTask().get());
  EXPECT_EQ(0.0, frequency(pubSub3.getDiagnosticTask()->getMinRate(), true));
  EXPECT_EQ(std::numeric_limits<double>::infinity(), frequency(pubSub3.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(0.1, pubSub3.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(5u, pubSub3.getDiagnosticTask()->getRateWindowSize());
  EXPECT_EQ(ros::Duration(-1), pubSub3.getDiagnosticTask()->getMinDelay());
  EXPECT_EQ(ros::Duration(5), pubSub3.getDiagnosticTask()->getMaxDelay());

  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub4(params, {1.0, 2.0, 3.0, 4, 5.0, 6.0});
  ASSERT_NE(nullptr, pubSub4.getDiagnosticTask().get());
  EXPECT_EQ(1.0, frequency(pubSub4.getDiagnosticTask()->getMinRate()));
  EXPECT_EQ(2.0, frequency(pubSub4.getDiagnosticTask()->getMaxRate()));
  EXPECT_EQ(3.0, pubSub4.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(4u, pubSub4.getDiagnosticTask()->getRateWindowSize());
  EXPECT_EQ(ros::Duration(5), pubSub4.getDiagnosticTask()->getMinDelay());
  EXPECT_EQ(ros::Duration(6), pubSub4.getDiagnosticTask()->getMaxDelay());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;
  nodeParams["delay"]["min"] = 14.0;
  nodeParams["delay"]["max"] = 15.0;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub5(params);
  ASSERT_NE(nullptr, pubSub5.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub5.getDiagnosticTask()->getMinRate(), true));
  EXPECT_NEAR(11.0, frequency(pubSub5.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub5.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub5.getDiagnosticTask()->getRateWindowSize());
  EXPECT_EQ(ros::Duration(14), pubSub5.getDiagnosticTask()->getMinDelay());
  EXPECT_EQ(ros::Duration(15), pubSub5.getDiagnosticTask()->getMaxDelay());

  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub6(params, {1.0, 2.0, 3.0, 4, 5.0, 6.0});
  ASSERT_NE(nullptr, pubSub6.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub6.getDiagnosticTask()->getMinRate()));
  EXPECT_NEAR(11.0, frequency(pubSub6.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub6.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub6.getDiagnosticTask()->getRateWindowSize());
  EXPECT_EQ(ros::Duration(14), pubSub6.getDiagnosticTask()->getMinDelay());
  EXPECT_EQ(ros::Duration(15), pubSub6.getDiagnosticTask()->getMaxDelay());

  nodeParams["rate"].clear();
  nodeParams["rate"]["desired"] = 9.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;
  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub7(params);
  ASSERT_NE(nullptr, pubSub7.getDiagnosticTask().get());
  EXPECT_NEAR(9.0, frequency(pubSub7.getDiagnosticTask()->getMinRate(), true), 1e-6);
  EXPECT_NEAR(9.0, frequency(pubSub7.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub7.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub7.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub8(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub8.getDiagnosticTask().get());
  EXPECT_NEAR(9.0, frequency(pubSub8.getDiagnosticTask()->getMinRate()), 1e-6);
  EXPECT_NEAR(9.0, frequency(pubSub8.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub8.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub8.getDiagnosticTask()->getRateWindowSize());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedPubSub<std_msgs::Header> pubSub9(params);
  ASSERT_NE(nullptr, pubSub9.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub9.getDiagnosticTask()->getMinRate(), true));
  EXPECT_NEAR(11.0, frequency(pubSub9.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub9.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub9.getDiagnosticTask()->getRateWindowSize());

  DiagnosedPubSub<std_msgs::Header> pubSub10(params, {1.0, 2.0, 3.0, 4});
  ASSERT_NE(nullptr, pubSub10.getDiagnosticTask().get());
  EXPECT_EQ(10.0, frequency(pubSub10.getDiagnosticTask()->getMinRate()));
  EXPECT_NEAR(11.0, frequency(pubSub10.getDiagnosticTask()->getMaxRate()), 1e-6);
  EXPECT_EQ(12.0, pubSub10.getDiagnosticTask()->getRateTolerance());
  EXPECT_EQ(13u, pubSub10.getDiagnosticTask()->getRateWindowSize());
}

TEST(DiagnosedPubSub, Attach)  // NOLINT
{
  ros::NodeHandle nh;
  ros::Time::setNow({10, 0});
  diagnostic_msgs::DiagnosticArrayConstPtr msg;
  size_t numCalled = 0;
  auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10,
    [&msg, &numCalled](const diagnostic_msgs::DiagnosticArrayConstPtr& m) {msg = m; numCalled++;});

  TopicStatusParam<diagnostic_msgs::DiagnosticArray> diag1(10.0, 10.0);
  DiagnosedPubSub<diagnostic_msgs::DiagnosticArray> pubSub1("a", diag1);
  diagnostic_updater::Updater updater(nh);
  pubSub1.attach(updater);

  pubSub1.getDiagnosticTask()->tick(ros::Time(9, 0));

  ros::Time::setNow({11, 0});
  pubSub1.getDiagnosticTask()->tick(ros::Time(10, 0));

  ros::WallDuration(0.1).sleep();
  updater.force_update();

  for (size_t i = 0; i < 10 && numCalled < 2; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }
  EXPECT_EQ(2u, numCalled);
  ASSERT_NE(nullptr, msg);
  ASSERT_EQ(1u, msg->status.size());
  EXPECT_EQ("Frequency too low.", msg->status[0].message);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, msg->status[0].level);
  EXPECT_LE(10u, msg->status[0].values.size());
}

TEST(DiagnosedPublisher, ConstructorsNoHeader)  // NOLINT
{
  ros::NodeHandle nh;
  ros::Time::setNow({10, 0});

  auto diag1 = std::make_shared<TopicStatus<std_msgs::Header>>("a");
  auto pub1 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher diagPub1(pub1, diag1);
  EXPECT_EQ(pub1, diagPub1.getPublisher());

  TopicStatusParam<std_msgs::Header> param1;
  auto pub2 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher<std_msgs::Header> diagPub2(pub2, "a", param1);
  EXPECT_EQ(pub2, diagPub2.getPublisher());

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  auto pub3 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher<std_msgs::Header> diagPub3(pub3, params);
  EXPECT_EQ(pub3, diagPub3.getPublisher());

  auto pub4 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher<std_msgs::Header> diagPub4(pub4, params, {1.0, 2.0, 3.0, 4});
  EXPECT_EQ(pub4, diagPub4.getPublisher());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  auto pub5 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher<std_msgs::Header> diagPub5(pub5, params);
  EXPECT_EQ(pub5, diagPub5.getPublisher());

  auto pub6 = nh.advertise<std_msgs::Header>("a", 10);
  DiagnosedPublisher<std_msgs::Header> diagPub6(pub6, params, {1.0, 2.0, 3.0, 4});
  EXPECT_EQ(pub6, diagPub6.getPublisher());

  size_t numMsgs {0};
  auto sub = nh.subscribe<std_msgs::Header>("a", 100, [&numMsgs](const std_msgs::HeaderConstPtr&) {numMsgs++;});

  ros::WallDuration(0.25).sleep();

  std_msgs::Header msg;
  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    msg.stamp = prevTime;
    diagPub1.publish(msg);
    diagPub2.publish(msg);
    diagPub3.publish(msg);
    diagPub4.publish(msg);
    diagPub5.publish(msg);
    diagPub6.publish(msg);
    prevTime = time;
    ros::spinOnce();
  }

  for (size_t i = 0; i < 10 && numMsgs < 60; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  EXPECT_EQ(60u, numMsgs);

  diagnostic_updater::DiagnosticStatusWrapper wrapper1, wrapper2, wrapper3, wrapper4, wrapper5, wrapper6;
  diagPub1.getDiagnosticTask()->run(wrapper1);
  diagPub2.getDiagnosticTask()->run(wrapper2);
  diagPub3.getDiagnosticTask()->run(wrapper3);
  diagPub4.getDiagnosticTask()->run(wrapper4);
  diagPub5.getDiagnosticTask()->run(wrapper5);
  diagPub6.getDiagnosticTask()->run(wrapper6);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper1.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper2.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper3.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, wrapper4.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper5.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper6.level);
  EXPECT_EQ("", wrapper1.message);
  EXPECT_EQ("", wrapper2.message);
  EXPECT_EQ("", wrapper3.message);
  EXPECT_EQ("Frequency too high.", wrapper4.message);
  EXPECT_EQ("", wrapper5.message);
  EXPECT_EQ("", wrapper6.message);
}

TEST(DiagnosedPublisher, ConstructorsWithHeader)  // NOLINT
{
  ros::NodeHandle nh;
  ros::Time::setNow({10, 0});

  auto diag1 = std::make_shared<TopicStatus<diagnostic_msgs::DiagnosticArray>>("a");
  auto pub1 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher diagPub1(pub1, diag1);
  EXPECT_EQ(pub1, diagPub1.getPublisher());

  TopicStatusParam<diagnostic_msgs::DiagnosticArray> param1;
  auto pub2 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher<diagnostic_msgs::DiagnosticArray> diagPub2(pub2, "a", param1);
  EXPECT_EQ(pub2, diagPub2.getPublisher());

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  auto pub3 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher<diagnostic_msgs::DiagnosticArray> diagPub3(pub3, params);
  EXPECT_EQ(pub3, diagPub3.getPublisher());

  auto pub4 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher<diagnostic_msgs::DiagnosticArray> diagPub4(pub4, params, {1.0, 2.0, 3.0, 4, 5.0, 6.0});
  EXPECT_EQ(pub4, diagPub4.getPublisher());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;
  nodeParams["delay"]["min"] = 14.0;
  nodeParams["delay"]["max"] = 15.0;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  auto pub5 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher<diagnostic_msgs::DiagnosticArray> diagPub5(pub5, params);
  EXPECT_EQ(pub5, diagPub5.getPublisher());

  auto pub6 = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);
  DiagnosedPublisher<diagnostic_msgs::DiagnosticArray> diagPub6(pub6, params, {1.0, 2.0, 3.0, 4, 5.0, 6.0});
  EXPECT_EQ(pub6, diagPub6.getPublisher());

  size_t numMsgs {0};
  auto sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>(
    "a", 100, [&numMsgs](const diagnostic_msgs::DiagnosticArrayConstPtr&) {numMsgs++;});

  ros::WallDuration(0.25).sleep();

  diagnostic_msgs::DiagnosticArray msg;
  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    msg.header.stamp = prevTime;
    diagPub1.publish(msg);
    diagPub2.publish(msg);
    diagPub3.publish(msg);
    diagPub4.publish(msg);
    diagPub5.publish(msg);
    diagPub6.publish(msg);
    prevTime = time;
    ros::spinOnce();
  }

  for (size_t i = 0; i < 10 && numMsgs < 60; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  EXPECT_EQ(60u, numMsgs);

  diagnostic_updater::DiagnosticStatusWrapper wrapper1, wrapper2, wrapper3, wrapper4, wrapper5, wrapper6;
  diagPub1.getDiagnosticTask()->run(wrapper1);
  diagPub2.getDiagnosticTask()->run(wrapper2);
  diagPub3.getDiagnosticTask()->run(wrapper3);
  diagPub4.getDiagnosticTask()->run(wrapper4);
  diagPub5.getDiagnosticTask()->run(wrapper5);
  diagPub6.getDiagnosticTask()->run(wrapper6);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper1.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper2.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper3.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper4.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper5.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper6.level);
  EXPECT_EQ("", wrapper1.message);
  EXPECT_EQ("", wrapper2.message);
  EXPECT_EQ("", wrapper3.message);
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", wrapper4.message);
  EXPECT_EQ("Timestamps too far in future seen.", wrapper5.message);
  EXPECT_EQ("Timestamps too far in future seen.", wrapper6.message);
}

TEST(DiagnosedSubscriber, DiagnosticsNoHeader)  // NOLINT
{
  ros::NodeHandle nh;
  ros::Time::setNow({10, 0});

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  size_t numMsgs1 {0}, numMsgs2 {0}, numMsgs3 {0}, numMsgs4 {0};

  DiagnosedSubscriber<std_msgs::Header> sub1(nh, params, {}, "a", 10,
    [&numMsgs1](const std_msgs::HeaderConstPtr&){++numMsgs1;});
  EXPECT_TRUE(sub1.getSubscriber());

  DiagnosedSubscriber<std_msgs::Header> sub2(nh, params, {1.0, 2.0, 3.0, 4}, "a", 10,
    [&numMsgs2](const std_msgs::HeaderConstPtr&){++numMsgs2;});
  EXPECT_TRUE(sub2.getSubscriber());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedSubscriber<std_msgs::Header> sub3(nh, params, {}, "a", 10,
    [&numMsgs3](const std_msgs::HeaderConstPtr&){++numMsgs3;});
  EXPECT_TRUE(sub3.getSubscriber());

  DiagnosedSubscriber<std_msgs::Header> sub4(nh, params, {1.0, 2.0, 3.0, 4}, "a", 10,
    [&numMsgs4](const std_msgs::HeaderConstPtr&){++numMsgs4;});
  EXPECT_TRUE(sub4.getSubscriber());

  auto pub = nh.advertise<std_msgs::Header>("a", 10);

  ros::WallDuration(0.25).sleep();

  std_msgs::Header msg;
  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    msg.stamp = prevTime;
    pub.publish(msg);
    prevTime = time;
    ros::spinOnce();
  }

  for (size_t i = 0; i < 10 && numMsgs1 + numMsgs2 + numMsgs3 + numMsgs4 < 40; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  EXPECT_EQ(10u, numMsgs1);
  EXPECT_EQ(10u, numMsgs2);
  EXPECT_EQ(10u, numMsgs3);
  EXPECT_EQ(10u, numMsgs4);

  diagnostic_updater::DiagnosticStatusWrapper wrapper1, wrapper2, wrapper3, wrapper4;
  sub1.getDiagnosticTask()->run(wrapper1);
  sub2.getDiagnosticTask()->run(wrapper2);
  sub3.getDiagnosticTask()->run(wrapper3);
  sub4.getDiagnosticTask()->run(wrapper4);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper1.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::WARN, wrapper2.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper3.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper4.level);
  EXPECT_EQ("", wrapper1.message);
  EXPECT_EQ("Frequency too high.", wrapper2.message);
  EXPECT_EQ("", wrapper3.message);
  EXPECT_EQ("", wrapper4.message);
}

TEST(DiagnosedSubscriber, DiagnosticsWithHeader)  // NOLINT
{
  ros::NodeHandle nh;
  ros::Time::setNow({10, 0});

  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  size_t numMsgs1 {0}, numMsgs2 {0}, numMsgs3 {0}, numMsgs4 {0};

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> sub1(nh, params, {}, "a", 10,
    [&numMsgs1](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numMsgs1;});
  EXPECT_TRUE(sub1.getSubscriber());

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> sub2(nh, params, {1.0, 2.0, 3.0, 4, 5.0, 6.0}, "a", 10,
    [&numMsgs2](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numMsgs2;});
  EXPECT_TRUE(sub2.getSubscriber());

  nodeParams["rate"]["min"] = 10.0;
  nodeParams["rate"]["max"] = 11.0;
  nodeParams["rate"]["tolerance"] = 12.0;
  nodeParams["rate"]["window_size"] = 13;
  nodeParams["delay"]["min"] = 14.0;
  nodeParams["delay"]["max"] = 15.0;

  paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> sub3(nh, params, {}, "a", 10,
    [&numMsgs3](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numMsgs3;});
  EXPECT_TRUE(sub3.getSubscriber());

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> sub4(nh, params, {1.0, 2.0, 3.0, 4, 5.0, 6.0}, "a", 10,
    [&numMsgs4](const diagnostic_msgs::DiagnosticArrayConstPtr&){++numMsgs4;});
  EXPECT_TRUE(sub4.getSubscriber());

  auto pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("a", 10);

  ros::WallDuration(0.25).sleep();

  diagnostic_msgs::DiagnosticArray msg;
  auto prevTime = ros::Time::now();
  for (auto time = ros::Time(10.1); time <= ros::Time(11); time += ros::Duration(0.1))
  {
    ros::Time::setNow(time);
    msg.header.stamp = prevTime;
    pub.publish(msg);
    prevTime = time;
    ros::spinOnce();
  }

  for (size_t i = 0; i < 10 && numMsgs1 + numMsgs2 + numMsgs3 + numMsgs4 < 40; ++i)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  EXPECT_EQ(10u, numMsgs1);
  EXPECT_EQ(10u, numMsgs2);
  EXPECT_EQ(10u, numMsgs3);
  EXPECT_EQ(10u, numMsgs4);

  diagnostic_updater::DiagnosticStatusWrapper wrapper1, wrapper2, wrapper3, wrapper4;
  sub1.getDiagnosticTask()->run(wrapper1);
  sub2.getDiagnosticTask()->run(wrapper2);
  sub3.getDiagnosticTask()->run(wrapper3);
  sub4.getDiagnosticTask()->run(wrapper4);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::OK, wrapper1.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper2.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper3.level);
  EXPECT_EQ(diagnostic_msgs::DiagnosticStatus::ERROR, wrapper4.level);
  EXPECT_EQ("", wrapper1.message);
  EXPECT_EQ("Frequency too high.; Timestamps too far in future seen.", wrapper2.message);
  EXPECT_EQ("Timestamps too far in future seen.", wrapper3.message);
  EXPECT_EQ("Timestamps too far in future seen.", wrapper4.message);
}

// The following definitions are used by the subscriber test.

class CbTest
{
public:
  void cbConstPtr(const std_msgs::HeaderConstPtr& m) {this->cbConstPtrFrame = m->frame_id;}
  void cbPtr(const std_msgs::HeaderPtr& m) {this->cbPtrFrame = m->frame_id;}
  void cbConstRef(const std_msgs::Header& m) {this->cbConstRefFrame = m.frame_id;}
  void cbValue(std_msgs::Header m) {this->cbValueFrame = m.frame_id;}  // NOLINT
  void cbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
    this->cbConstEventFrame = m.getConstMessage()->frame_id;}
  void cbEvent(const ros::MessageEvent<std_msgs::Header>& m) {this->cbEventFrame = m.getConstMessage()->frame_id;}
  void constCbConstPtr(const std_msgs::HeaderConstPtr& m) const {this->constCbConstPtrFrame = m->frame_id;}
  void constCbPtr(const std_msgs::HeaderPtr& m) const {this->constCbPtrFrame = m->frame_id;}
  void constCbConstRef(const std_msgs::Header& m) const {this->constCbConstRefFrame = m.frame_id;}
  void constCbValue(std_msgs::Header m) const {this->constCbValueFrame = m.frame_id;}  // NOLINT
  void constCbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) const {
    this->constCbConstEventFrame = m.getConstMessage()->frame_id;}
  void constCbEvent(const ros::MessageEvent<std_msgs::Header>& m) const {
    this->constCbEventFrame = m.getConstMessage()->frame_id;}

  std::string cbConstPtrFrame {};
  std::string cbPtrFrame {};
  std::string cbConstRefFrame {};
  std::string cbValueFrame {};
  std::string cbConstEventFrame {};
  std::string cbEventFrame {};
  mutable std::string constCbConstPtrFrame {};
  mutable std::string constCbPtrFrame {};
  mutable std::string constCbConstRefFrame {};
  mutable std::string constCbValueFrame {};
  mutable std::string constCbConstEventFrame {};
  mutable std::string constCbEventFrame {};
};

class CbTestHeader
{
public:
  void cbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {this->cbConstPtrFrame = m->header.frame_id;}
  void cbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {this->cbPtrFrame = m->header.frame_id;}
  void cbConstRef(const diagnostic_msgs::DiagnosticArray& m) {this->cbConstRefFrame = m.header.frame_id;}
  void cbValue(diagnostic_msgs::DiagnosticArray m) {this->cbValueFrame = m.header.frame_id;}  // NOLINT
  void cbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
    this->cbConstEventFrame = m.getConstMessage()->header.frame_id;}
  void cbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
    this->cbEventFrame = m.getConstMessage()->header.frame_id;}
  void constCbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) const {
    this->constCbConstPtrFrame = m->header.frame_id;}
  void constCbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) const {this->constCbPtrFrame = m->header.frame_id;}
  void constCbConstRef(const diagnostic_msgs::DiagnosticArray& m) const {
    this->constCbConstRefFrame = m.header.frame_id;}
  void constCbValue(diagnostic_msgs::DiagnosticArray m) const {this->constCbValueFrame = m.header.frame_id;}  // NOLINT
  void constCbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) const {
    this->constCbConstEventFrame = m.getConstMessage()->header.frame_id;}
  void constCbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) const {
    this->constCbEventFrame = m.getConstMessage()->header.frame_id;}

  std::string cbConstPtrFrame {};
  std::string cbPtrFrame {};
  std::string cbConstRefFrame {};
  std::string cbValueFrame {};
  std::string cbConstEventFrame {};
  std::string cbEventFrame {};
  mutable std::string constCbConstPtrFrame {};
  mutable std::string constCbPtrFrame {};
  mutable std::string constCbConstRefFrame {};
  mutable std::string constCbValueFrame {};
  mutable std::string constCbConstEventFrame {};
  mutable std::string constCbEventFrame {};
};

std::string cbConstPtrFrame {};
std::string cbPtrFrame {};
std::string cbConstRefFrame {};
std::string cbValueFrame {};
std::string cbConstEventFrame {};
std::string cbEventFrame {};

void cbConstPtr(const std_msgs::HeaderConstPtr& m) {cbConstPtrFrame = m->frame_id;}
void cbPtr(const std_msgs::HeaderPtr& m) {cbPtrFrame = m->frame_id;}
void cbConstRef(const std_msgs::Header& m) {cbConstRefFrame = m.frame_id;}
void cbValue(std_msgs::Header m) {cbValueFrame = m.frame_id;}  // NOLINT
void cbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
  cbConstEventFrame = m.getConstMessage()->frame_id;}
void cbEvent(const ros::MessageEvent<std_msgs::Header>& m) { cbEventFrame = m.getConstMessage()->frame_id;}

void cbHeaderConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {cbConstPtrFrame = m->header.frame_id;}
void cbHeaderPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {cbPtrFrame = m->header.frame_id;}
void cbHeaderConstRef(const diagnostic_msgs::DiagnosticArray& m) {cbConstRefFrame = m.header.frame_id;}
void cbHeaderValue(diagnostic_msgs::DiagnosticArray m) {cbValueFrame = m.header.frame_id;}  // NOLINT
void cbHeaderConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  cbConstEventFrame = m.getConstMessage()->header.frame_id;}
void cbHeaderEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  cbEventFrame = m.getConstMessage()->header.frame_id;}

std::string optionsCbConstPtrFrame {};
std::string optionsCbPtrFrame {};
std::string optionsCbConstRefFrame {};
std::string optionsCbValueFrame {};
std::string optionsCbConstEventFrame {};
std::string optionsCbEventFrame {};

void optionsCbConstPtr(const std_msgs::HeaderConstPtr& m) {optionsCbConstPtrFrame = m->frame_id;}
void optionsCbPtr(const std_msgs::HeaderPtr& m) {optionsCbPtrFrame = m->frame_id;}
void optionsCbConstRef(const std_msgs::Header& m) {optionsCbConstRefFrame = m.frame_id;}
void optionsCbValue(std_msgs::Header m) {optionsCbValueFrame = m.frame_id;}  // NOLINT
void optionsCbConstEvent(const ros::MessageEvent<std_msgs::Header const>& m) {
  optionsCbConstEventFrame = m.getConstMessage()->frame_id;}
void optionsCbEvent(const ros::MessageEvent<std_msgs::Header>& m) {optionsCbEventFrame = m.getConstMessage()->frame_id;}

void optionsHeaderCbConstPtr(const diagnostic_msgs::DiagnosticArrayConstPtr& m) {
  optionsCbConstPtrFrame = m->header.frame_id;}
void optionsHeaderCbPtr(const diagnostic_msgs::DiagnosticArrayPtr& m) {optionsCbPtrFrame = m->header.frame_id;}
void optionsHeaderCbConstRef(const diagnostic_msgs::DiagnosticArray& m) {optionsCbConstRefFrame = m.header.frame_id;}
void optionsHeaderCbValue(diagnostic_msgs::DiagnosticArray m) {optionsCbValueFrame = m.header.frame_id;}  // NOLINT
void optionsHeaderCbConstEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  optionsCbConstEventFrame = m.getConstMessage()->header.frame_id;}
void optionsHeaderCbEvent(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  optionsCbEventFrame = m.getConstMessage()->header.frame_id;}

std::string lambdaCbConstPtrFrame {};
std::string lambdaCbPtrFrame {};
std::string lambdaCbConstRefFrame {};
std::string lambdaCbValueFrame {};
std::string lambdaCbConstEventFrame {};
std::string lambdaCbEventFrame {};

auto lambdaCbConstPtr = [](const std_msgs::HeaderConstPtr& m){lambdaCbConstPtrFrame = m->frame_id;};
auto lambdaCbPtr = [](const std_msgs::HeaderPtr& m){lambdaCbPtrFrame = m->frame_id;};
auto lambdaCbConstRef = [](const std_msgs::Header& m){lambdaCbConstRefFrame = m.frame_id;};
auto lambdaCbValue = [](std_msgs::Header m){lambdaCbValueFrame = m.frame_id;};  // NOLINT
auto lambdaCbConstEvent = [](const ros::MessageEvent<std_msgs::Header const>& m) {
  lambdaCbConstEventFrame = m.getConstMessage()->frame_id;};
auto lambdaCbEvent = [](const ros::MessageEvent<std_msgs::Header>& m) {
  lambdaCbEventFrame = m.getConstMessage()->frame_id;};

auto lambdaHeaderCbConstPtr = [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){
  lambdaCbConstPtrFrame = m->header.frame_id;};
auto lambdaHeaderCbPtr = [](const diagnostic_msgs::DiagnosticArrayPtr& m){lambdaCbPtrFrame = m->header.frame_id;};
auto lambdaHeaderCbConstRef = [](const diagnostic_msgs::DiagnosticArray& m){lambdaCbConstRefFrame = m.header.frame_id;};
auto lambdaHeaderCbValue = [](diagnostic_msgs::DiagnosticArray m){lambdaCbValueFrame = m.header.frame_id;};  // NOLINT
auto lambdaHeaderCbConstEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  lambdaCbConstEventFrame = m.getConstMessage()->header.frame_id;};
auto lambdaHeaderCbEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  lambdaCbEventFrame = m.getConstMessage()->header.frame_id;};

std::string lambdaHintsConstPtrFrame {};
std::string lambdaHintsPtrFrame {};
std::string lambdaHintsConstRefFrame {};
std::string lambdaHintsValueFrame {};
std::string lambdaHintsConstEventFrame {};
std::string lambdaHintsEventFrame {};

auto lambdaHintsConstPtr = [](const std_msgs::HeaderConstPtr& m){lambdaHintsConstPtrFrame = m->frame_id;};
auto lambdaHintsPtr = [](const std_msgs::HeaderPtr& m){lambdaHintsPtrFrame = m->frame_id;};
auto lambdaHintsConstRef = [](const std_msgs::Header& m){lambdaHintsConstRefFrame = m.frame_id;};
auto lambdaHintsValue = [](std_msgs::Header m){lambdaHintsValueFrame = m.frame_id;};  // NOLINT
auto lambdaHintsConstEvent = [](const ros::MessageEvent<std_msgs::Header const>& m) {
  lambdaHintsConstEventFrame = m.getConstMessage()->frame_id;};
auto lambdaHintsEvent = [](const ros::MessageEvent<std_msgs::Header>& m) {
  lambdaHintsEventFrame = m.getConstMessage()->frame_id;};

auto lambdaHeaderHintsConstPtr = [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){
  lambdaHintsConstPtrFrame = m->header.frame_id;};
auto lambdaHeaderHintsPtr = [](const diagnostic_msgs::DiagnosticArrayPtr& m){lambdaHintsPtrFrame = m->header.frame_id;};
auto lambdaHeaderHintsConstRef = [](const diagnostic_msgs::DiagnosticArray& m){
  lambdaHintsConstRefFrame = m.header.frame_id;};
auto lambdaHeaderHintsValue = [](diagnostic_msgs::DiagnosticArray m){  //NOLINT
  lambdaHintsValueFrame = m.header.frame_id;};
auto lambdaHeaderHintsConstEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
  lambdaHintsConstEventFrame = m.getConstMessage()->header.frame_id;};
auto lambdaHeaderHintsEvent = [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
  lambdaHintsEventFrame = m.getConstMessage()->header.frame_id;};

std::string boostLambdaConstPtrFrame {};
std::string boostLambdaPtrFrame {};
std::string boostLambdaConstRefFrame {};
std::string boostLambdaValueFrame {};
std::string boostLambdaConstEventFrame {};
std::string boostLambdaEventFrame {};

boost::function<void(const std_msgs::HeaderConstPtr&)> boostLambdaConstPtr =  // NOLINT
  [](const std_msgs::HeaderConstPtr& m){boostLambdaConstPtrFrame = m->frame_id;};
boost::function<void(const std_msgs::HeaderPtr&)> boostLambdaPtr =  // NOLINT
  [](const std_msgs::HeaderPtr& m){boostLambdaPtrFrame = m->frame_id;};
boost::function<void(const std_msgs::Header&)> boostLambdaConstRef =  // NOLINT
  [](const std_msgs::Header& m){boostLambdaConstRefFrame = m.frame_id;};
boost::function<void(std_msgs::Header)> boostLambdaValue =  // NOLINT
  [](std_msgs::Header m){boostLambdaValueFrame = m.frame_id;};  // NOLINT
boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)> boostLambdaConstEvent =  // NOLINT
  [](const ros::MessageEvent<std_msgs::Header const>& m) {boostLambdaConstEventFrame = m.getConstMessage()->frame_id;};
boost::function<void(const ros::MessageEvent<std_msgs::Header>&)> boostLambdaEvent =  // NOLINT
  [](const ros::MessageEvent<std_msgs::Header>& m) {boostLambdaEventFrame = m.getConstMessage()->frame_id;};

boost::function<void(const diagnostic_msgs::DiagnosticArrayConstPtr&)> boostHeaderLambdaConstPtr =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){boostLambdaConstPtrFrame = m->header.frame_id;};
boost::function<void(const diagnostic_msgs::DiagnosticArrayPtr&)> boostHeaderLambdaPtr =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArrayPtr& m){boostLambdaPtrFrame = m->header.frame_id;};
boost::function<void(const diagnostic_msgs::DiagnosticArray&)> boostHeaderLambdaConstRef =  // NOLINT
  [](const diagnostic_msgs::DiagnosticArray& m){boostLambdaConstRefFrame = m.header.frame_id;};
boost::function<void(diagnostic_msgs::DiagnosticArray)> boostHeaderLambdaValue =  // NOLINT
  [](diagnostic_msgs::DiagnosticArray m){boostLambdaValueFrame = m.header.frame_id;};  // NOLINT
boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>
boostHeaderLambdaConstEvent =  // NOLINT
  [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m) {
    boostLambdaConstEventFrame = m.getConstMessage()->header.frame_id;};
boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)> boostHeaderLambdaEvent =  // NOLINT
  [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m) {
    boostLambdaEventFrame = m.getConstMessage()->header.frame_id;};

std::string noCaptureLambdaConstPtrFrame {};
std::string noCaptureLambdaPtrFrame {};
std::string noCaptureLambdaConstRefFrame {};
std::string noCaptureLambdaValueFrame {};
std::string noCaptureLambdaConstEventFrame {};
std::string noCaptureLambdaEventFrame {};
std::string valueCaptureLambdaConstPtrFrame {};
std::string valueCaptureLambdaPtrFrame {};
std::string valueCaptureLambdaConstRefFrame {};
std::string valueCaptureLambdaValueFrame {};
std::string valueCaptureLambdaConstEventFrame {};
std::string valueCaptureLambdaEventFrame {};

TEST(DiagnosedSubscriber, AllSupportedCallbackSignaturesNoHeader)  // NOLINT
{
  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  ros::NodeHandle nh;
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  CbTest obj1, obj2, obj3;
  auto sharedObj = boost::make_shared<CbTest>();

  std::string refCaptureLambdaConstPtrFrame {};
  std::string refCaptureLambdaPtrFrame {};
  std::string refCaptureLambdaConstRefFrame {};
  std::string refCaptureLambdaValueFrame {};
  std::string refCaptureLambdaConstEventFrame {};
  std::string refCaptureLambdaEventFrame {};

  cbConstPtrFrame = cbPtrFrame = cbConstRefFrame = cbValueFrame = cbConstEventFrame = cbEventFrame = "";
  cbConstPtrFrame = cbPtrFrame = cbConstRefFrame = cbValueFrame = cbConstEventFrame = cbEventFrame = "";
  lambdaHintsConstPtrFrame = lambdaHintsPtrFrame = lambdaHintsConstRefFrame = lambdaHintsValueFrame =
    lambdaHintsConstEventFrame = lambdaHintsEventFrame = "";
  noCaptureLambdaConstPtrFrame = noCaptureLambdaPtrFrame = noCaptureLambdaConstRefFrame = noCaptureLambdaValueFrame =
    noCaptureLambdaConstEventFrame = noCaptureLambdaEventFrame = "";
  valueCaptureLambdaConstPtrFrame = valueCaptureLambdaPtrFrame = valueCaptureLambdaConstRefFrame =
    valueCaptureLambdaValueFrame = valueCaptureLambdaConstEventFrame = valueCaptureLambdaEventFrame = "";

  DiagnosedSubscriber subConstPtr(nh, params, {9.0, 10}, "a", 0, &cbConstPtr);
  DiagnosedSubscriber subPtr(nh, params, {}, "a", 0, &cbPtr);
  DiagnosedSubscriber subConstRef(nh, params, {}, "a", 0, &cbConstRef);
  DiagnosedSubscriber subValue(nh, params, {}, "a", 0, &cbValue);
  DiagnosedSubscriber subConstEvent(nh, params, {}, "a", 0, &cbConstEvent);
  DiagnosedSubscriber subEvent(nh, params, {}, "a", 0, &cbEvent);
  EXPECT_TRUE((::std::is_same_v<decltype(subConstPtr), DiagnosedSubscriber<std_msgs::Header>>));

  DiagnosedSubscriber subClassConstPtr(nh, params, {}, "a", 0, &CbTest::cbConstPtr, &obj1);
  DiagnosedSubscriber subClassPtr(nh, params, {}, "a", 0, &CbTest::cbPtr, &obj1);
  DiagnosedSubscriber subClassConstRef(nh, params, {}, "a", 0, &CbTest::cbConstRef, &obj1);
  DiagnosedSubscriber subClassValue(nh, params, {}, "a", 0, &CbTest::cbValue, &obj1);
  DiagnosedSubscriber subClassConstEvent(nh, params, {}, "a", 0, &CbTest::cbConstEvent, &obj1);
  DiagnosedSubscriber subClassEvent(nh, params, {}, "a", 0, &CbTest::cbEvent, &obj1);

  DiagnosedSubscriber subConstClassConstPtr(nh, params, {}, "a", 0, &CbTest::constCbConstPtr, &obj1);
  DiagnosedSubscriber subConstClassPtr(nh, params, {}, "a", 0, &CbTest::constCbPtr, &obj1);
  DiagnosedSubscriber subConstClassConstRef(nh, params, {}, "a", 0, &CbTest::constCbConstRef, &obj1);
  DiagnosedSubscriber subConstClassValue(nh, params, {}, "a", 0, &CbTest::constCbValue, &obj1);
  DiagnosedSubscriber subConstClassConstEvent(nh, params, {}, "a", 0, &CbTest::constCbConstEvent, &obj1);
  DiagnosedSubscriber subConstClassEvent(nh, params, {}, "a", 0, &CbTest::constCbEvent, &obj1);

  DiagnosedSubscriber subSharedClassConstPtr(nh, params, {}, "a", 0, &CbTest::cbConstPtr, sharedObj);
  DiagnosedSubscriber subSharedClassPtr(nh, params, {}, "a", 0, &CbTest::cbPtr, sharedObj);
  DiagnosedSubscriber subSharedClassConstRef(nh, params, {}, "a", 0, &CbTest::cbConstRef, sharedObj);
  DiagnosedSubscriber subSharedClassValue(nh, params, {}, "a", 0, &CbTest::cbValue, sharedObj);
  DiagnosedSubscriber subSharedClassConstEvent(nh, params, {}, "a", 0, &CbTest::cbConstEvent, sharedObj);
  DiagnosedSubscriber subSharedClassEvent(nh, params, {}, "a", 0, &CbTest::cbEvent, sharedObj);

  DiagnosedSubscriber subConstSharedClassConstPtr(nh, params, {}, "a", 0, &CbTest::constCbConstPtr, sharedObj);
  DiagnosedSubscriber subConstSharedClassPtr(nh, params, {}, "a", 0, &CbTest::constCbPtr, sharedObj);
  DiagnosedSubscriber subConstSharedClassConstRef(nh, params, {}, "a", 0, &CbTest::constCbConstRef, sharedObj);
  DiagnosedSubscriber subConstSharedClassValue(nh, params, {}, "a", 0, &CbTest::constCbValue, sharedObj);
  DiagnosedSubscriber subConstSharedClassConstEvent(nh, params, {}, "a", 0, &CbTest::constCbConstEvent, sharedObj);
  DiagnosedSubscriber subConstSharedClassEvent(nh, params, {}, "a", 0, &CbTest::constCbEvent, sharedObj);

  DiagnosedSubscriber<std_msgs::Header> subNoCaptureLambdaConstPtr(nh, params, {}, "a", 0,
    [](const std_msgs::HeaderConstPtr& m){noCaptureLambdaValueFrame = m->frame_id;});
  DiagnosedSubscriber<std_msgs::Header> subNoCaptureLambdaPtr(nh, params, {}, "a", 0,
    [](const std_msgs::HeaderPtr& m){noCaptureLambdaConstRefFrame = m->frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaConstRef(nh, params, {}, "a", 0,
    (void(*)(const std_msgs::Header&)) [](const std_msgs::Header& m){noCaptureLambdaPtrFrame = m.frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaValue(nh, params, {}, "a", 0,
    (void(*)(std_msgs::Header)) [](std_msgs::Header m){noCaptureLambdaConstPtrFrame = m.frame_id;});  // NOLINT
  DiagnosedSubscriber subNoCaptureLambdaConstEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header const>&)) [](const ros::MessageEvent<std_msgs::Header const>& m){
      noCaptureLambdaEventFrame = m.getConstMessage()->frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header>&)) [](const ros::MessageEvent<std_msgs::Header>& m){
      noCaptureLambdaConstEventFrame = m.getConstMessage()->frame_id;});

  DiagnosedSubscriber<std_msgs::Header> subRefCaptureLambdaConstPtr(nh, params, {}, "a", 0,
    [&](const std_msgs::HeaderConstPtr& m){refCaptureLambdaValueFrame = m->frame_id;});
  DiagnosedSubscriber<std_msgs::Header> subRefCaptureLambdaPtr(nh, params, {}, "a", 0,
    [&](const std_msgs::HeaderPtr& m){refCaptureLambdaConstRefFrame = m->frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)
      [&](const std_msgs::Header& m){refCaptureLambdaPtrFrame = m.frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)
      [&](std_msgs::Header m){refCaptureLambdaConstPtrFrame = m.frame_id;});  // NOLINT
  DiagnosedSubscriber subRefCaptureLambdaConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      [&](const ros::MessageEvent<std_msgs::Header const>& m){
        refCaptureLambdaEventFrame = m.getConstMessage()->frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      [&](const ros::MessageEvent<std_msgs::Header>& m){
        refCaptureLambdaConstEventFrame = m.getConstMessage()->frame_id;});

  DiagnosedSubscriber<std_msgs::Header> subValueCaptureLambdaConstPtr(nh, params, {}, "a", 0,
    [=](const std_msgs::HeaderConstPtr& m){valueCaptureLambdaValueFrame = m->frame_id;});
  DiagnosedSubscriber<std_msgs::Header> subValueCaptureLambdaPtr(nh, params, {}, "a", 0,
    [=](const std_msgs::HeaderPtr& m){valueCaptureLambdaConstRefFrame = m->frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)
      [=](const std_msgs::Header& m){valueCaptureLambdaPtrFrame = m.frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)
      [=](std_msgs::Header m){valueCaptureLambdaConstPtrFrame = m.frame_id;});  // NOLINT
  DiagnosedSubscriber subValueCaptureLambdaConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      [=](const ros::MessageEvent<std_msgs::Header const>& m){
        valueCaptureLambdaEventFrame = m.getConstMessage()->frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      [=](const ros::MessageEvent<std_msgs::Header>& m){
        valueCaptureLambdaConstEventFrame = m.getConstMessage()->frame_id;});

  DiagnosedSubscriber<std_msgs::Header> subLambdaConstPtr(nh, params, {}, "a", 0, lambdaCbConstPtr);
  DiagnosedSubscriber<std_msgs::Header> subLambdaPtr(nh, params, {}, "a", 0, lambdaCbPtr);
  DiagnosedSubscriber subLambdaConstRef(nh, params, {}, "a", 0, (void(*)(const std_msgs::Header&))lambdaCbConstRef);
  DiagnosedSubscriber subLambdaValue(nh, params, {}, "a", 0, (void(*)(std_msgs::Header))lambdaCbValue);
  DiagnosedSubscriber subLambdaConstEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header const>&))lambdaCbConstEvent);
  DiagnosedSubscriber subLambdaEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header>&))lambdaCbEvent);

  DiagnosedSubscriber<std_msgs::Header> subLambdaHintsConstPtr(nh, params, {}, "a", 0,
    lambdaHintsConstPtr, ros::TransportHints());
  DiagnosedSubscriber<std_msgs::Header> subLambdaHintsPtr(nh, params, {}, "a", 0,
    lambdaHintsPtr, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsConstRef(nh, params, {}, "a", 0,
    (void(*)(const std_msgs::Header&))lambdaHintsConstRef, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsValue(nh, params, {}, "a", 0,
    (void(*)(std_msgs::Header))lambdaHintsValue, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsConstEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header const>&))lambdaHintsConstEvent, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsEvent(nh, params, {}, "a", 0,
    (void(*)(const ros::MessageEvent<std_msgs::Header>&))lambdaHintsEvent, ros::TransportHints());

  DiagnosedSubscriber<std_msgs::Header> subClassBindConstPtr(nh, params, {}, "a", 0,
    boost::bind(&CbTest::cbConstPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber<std_msgs::Header> subClassBindPtr(nh, params, {}, "a", 0,
    boost::bind(&CbTest::cbPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)boost::bind(&CbTest::cbConstRef, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)boost::bind(&CbTest::cbValue, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      boost::bind(&CbTest::cbConstEvent, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      boost::bind(&CbTest::cbEvent, &obj2, boost::placeholders::_1));

  DiagnosedSubscriber<std_msgs::Header> subConstClassBindConstPtr(nh, params, {}, "a", 0,
    boost::bind(&CbTest::constCbConstPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber<std_msgs::Header> subConstClassBindPtr(nh, params, {}, "a", 0,
    boost::bind(&CbTest::constCbPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)
      boost::bind(&CbTest::constCbConstRef, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)
      boost::bind(&CbTest::constCbValue, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      boost::bind(&CbTest::constCbConstEvent, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      boost::bind(&CbTest::constCbEvent, &obj2, boost::placeholders::_1));

  DiagnosedSubscriber<std_msgs::Header> subSharedClassBindConstPtr(nh, params, {}, "a", 0,
    std::bind(&CbTest::cbConstPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber<std_msgs::Header> subSharedClassBindPtr(nh, params, {}, "a", 0,
    std::bind(&CbTest::cbPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)std::bind(&CbTest::cbConstRef, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)std::bind(&CbTest::cbValue, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      std::bind(&CbTest::cbConstEvent, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      std::bind(&CbTest::cbEvent, &obj3, std::placeholders::_1));

  DiagnosedSubscriber<std_msgs::Header> subSharedConstClassBindConstPtr(nh, params, {}, "a", 0,
    std::bind(&CbTest::constCbConstPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber<std_msgs::Header> subSharedConstClassBindPtr(nh, params, {}, "a", 0,
    std::bind(&CbTest::constCbPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindConstRef(nh, params, {}, "a", 0,
    (boost::function<void(const std_msgs::Header&)>)std::bind(&CbTest::constCbConstRef, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindValue(nh, params, {}, "a", 0,
    (boost::function<void(std_msgs::Header)>)std::bind(&CbTest::constCbValue, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindConstEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header const>&)>)
      std::bind(&CbTest::constCbConstEvent, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindEvent(nh, params, {}, "a", 0,
    (boost::function<void(const ros::MessageEvent<std_msgs::Header>&)>)
      std::bind(&CbTest::constCbEvent, &obj3, std::placeholders::_1));

  DiagnosedSubscriber subBoostLambdaConstPtr(nh, params, {}, "a", 0, boostLambdaConstPtr);
  DiagnosedSubscriber subBoostLambdaPtr(nh, params, {}, "a", 0, boostLambdaPtr);
  DiagnosedSubscriber subBoostLambdaConstRef(nh, params, {}, "a", 0, boostLambdaConstRef);
  DiagnosedSubscriber subBoostLambdaValue(nh, params, {}, "a", 0, boostLambdaValue);
  DiagnosedSubscriber subBoostLambdaConstEvent(nh, params, {}, "a", 0, boostLambdaConstEvent);
  DiagnosedSubscriber subBoostLambdaEvent(nh, params, {}, "a", 0, boostLambdaEvent);

  ros::SubscribeOptions ops;
  ops.initByFullCallbackType<const std_msgs::HeaderConstPtr &>("a", 10, &optionsCbConstPtr);
  DiagnosedSubscriber<std_msgs::Header> subOptionsConstPtr(nh, params, {}, ops);
  ops.initByFullCallbackType<const std_msgs::HeaderPtr &>("a", 10, &optionsCbPtr);
  DiagnosedSubscriber<std_msgs::Header> subOptionsPtr(nh, params, {}, ops);
  ops.initByFullCallbackType<const std_msgs::Header&>("a", 10, &optionsCbConstRef);
  DiagnosedSubscriber<std_msgs::Header> subOptionsConstRef(nh, params, {}, ops);
  ops.initByFullCallbackType<std_msgs::Header>("a", 10, &optionsCbValue);
  DiagnosedSubscriber<std_msgs::Header> subOptionsValue(nh, params, {}, ops);
  ops.initByFullCallbackType<const ros::MessageEvent<std_msgs::Header const>&>("a", 10, &optionsCbConstEvent);
  DiagnosedSubscriber<std_msgs::Header> subOptionsConstEvent(nh, params, {}, ops);
  ops.initByFullCallbackType<const ros::MessageEvent<std_msgs::Header>&>("a", 10, &optionsCbEvent);
  DiagnosedSubscriber<std_msgs::Header> subOptionsEvent(nh, params, {}, ops);

  auto pub = nh.advertise<std_msgs::Header>("a", 100);
  auto message = std_msgs::Header();
  message.frame_id = "test";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  while (ros::WallTime::now() < end)
    ros::spinOnce();

  EXPECT_EQ(message.frame_id, cbConstPtrFrame);
  EXPECT_EQ(message.frame_id, cbPtrFrame);
  EXPECT_EQ(message.frame_id, cbConstRefFrame);
  EXPECT_EQ(message.frame_id, cbValueFrame);
  EXPECT_EQ(message.frame_id, cbConstEventFrame);
  EXPECT_EQ(message.frame_id, cbEventFrame);

  EXPECT_EQ(message.frame_id, optionsCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, optionsCbPtrFrame);
  EXPECT_EQ(message.frame_id, optionsCbConstRefFrame);
  EXPECT_EQ(message.frame_id, optionsCbValueFrame);
  EXPECT_EQ(message.frame_id, optionsCbConstEventFrame);
  EXPECT_EQ(message.frame_id, optionsCbEventFrame);

  EXPECT_EQ(message.frame_id, obj1.cbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj1.cbPtrFrame);
  EXPECT_EQ(message.frame_id, obj1.cbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj1.cbValueFrame);
  EXPECT_EQ(message.frame_id, obj1.cbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj1.cbEventFrame);

  EXPECT_EQ(message.frame_id, obj1.constCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj1.constCbPtrFrame);
  EXPECT_EQ(message.frame_id, obj1.constCbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj1.constCbValueFrame);
  EXPECT_EQ(message.frame_id, obj1.constCbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj1.constCbEventFrame);

  EXPECT_EQ(message.frame_id, sharedObj->cbConstPtrFrame);
  EXPECT_EQ(message.frame_id, sharedObj->cbPtrFrame);
  EXPECT_EQ(message.frame_id, sharedObj->cbConstRefFrame);
  EXPECT_EQ(message.frame_id, sharedObj->cbValueFrame);
  EXPECT_EQ(message.frame_id, sharedObj->cbConstEventFrame);
  EXPECT_EQ(message.frame_id, sharedObj->cbEventFrame);

  EXPECT_EQ(message.frame_id, sharedObj->constCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, sharedObj->constCbPtrFrame);
  EXPECT_EQ(message.frame_id, sharedObj->constCbConstRefFrame);
  EXPECT_EQ(message.frame_id, sharedObj->constCbValueFrame);
  EXPECT_EQ(message.frame_id, sharedObj->constCbConstEventFrame);
  EXPECT_EQ(message.frame_id, sharedObj->constCbEventFrame);

  EXPECT_EQ(message.frame_id, obj2.cbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj2.cbPtrFrame);
  EXPECT_EQ(message.frame_id, obj2.cbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj2.cbValueFrame);
  EXPECT_EQ(message.frame_id, obj2.cbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj2.cbEventFrame);

  EXPECT_EQ(message.frame_id, obj2.constCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj2.constCbPtrFrame);
  EXPECT_EQ(message.frame_id, obj2.constCbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj2.constCbValueFrame);
  EXPECT_EQ(message.frame_id, obj2.constCbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj2.constCbEventFrame);

  EXPECT_EQ(message.frame_id, obj3.cbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj3.cbPtrFrame);
  EXPECT_EQ(message.frame_id, obj3.cbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj3.cbValueFrame);
  EXPECT_EQ(message.frame_id, obj3.cbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj3.cbEventFrame);

  EXPECT_EQ(message.frame_id, obj3.constCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, obj3.constCbPtrFrame);
  EXPECT_EQ(message.frame_id, obj3.constCbConstRefFrame);
  EXPECT_EQ(message.frame_id, obj3.constCbValueFrame);
  EXPECT_EQ(message.frame_id, obj3.constCbConstEventFrame);
  EXPECT_EQ(message.frame_id, obj3.constCbEventFrame);

  EXPECT_EQ(message.frame_id, noCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.frame_id, noCaptureLambdaPtrFrame);
  EXPECT_EQ(message.frame_id, noCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.frame_id, noCaptureLambdaValueFrame);
  EXPECT_EQ(message.frame_id, noCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.frame_id, noCaptureLambdaEventFrame);

  EXPECT_EQ(message.frame_id, refCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.frame_id, refCaptureLambdaPtrFrame);
  EXPECT_EQ(message.frame_id, refCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.frame_id, refCaptureLambdaValueFrame);
  EXPECT_EQ(message.frame_id, refCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.frame_id, refCaptureLambdaEventFrame);

  EXPECT_EQ(message.frame_id, valueCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.frame_id, valueCaptureLambdaPtrFrame);
  EXPECT_EQ(message.frame_id, valueCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.frame_id, valueCaptureLambdaValueFrame);
  EXPECT_EQ(message.frame_id, valueCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.frame_id, valueCaptureLambdaEventFrame);

  EXPECT_EQ(message.frame_id, lambdaCbConstPtrFrame);
  EXPECT_EQ(message.frame_id, lambdaCbPtrFrame);
  EXPECT_EQ(message.frame_id, lambdaCbConstRefFrame);
  EXPECT_EQ(message.frame_id, lambdaCbValueFrame);
  EXPECT_EQ(message.frame_id, lambdaCbConstEventFrame);
  EXPECT_EQ(message.frame_id, lambdaCbEventFrame);

  EXPECT_EQ(message.frame_id, lambdaHintsConstPtrFrame);
  EXPECT_EQ(message.frame_id, lambdaHintsPtrFrame);
  EXPECT_EQ(message.frame_id, lambdaHintsConstRefFrame);
  EXPECT_EQ(message.frame_id, lambdaHintsValueFrame);
  EXPECT_EQ(message.frame_id, lambdaHintsConstEventFrame);
  EXPECT_EQ(message.frame_id, lambdaHintsEventFrame);

  EXPECT_EQ(message.frame_id, boostLambdaConstPtrFrame);
  EXPECT_EQ(message.frame_id, boostLambdaPtrFrame);
  EXPECT_EQ(message.frame_id, boostLambdaConstRefFrame);
  EXPECT_EQ(message.frame_id, boostLambdaValueFrame);
  EXPECT_EQ(message.frame_id, boostLambdaConstEventFrame);
  EXPECT_EQ(message.frame_id, boostLambdaEventFrame);
}

TEST(DiagnosedSubscriber, AllSupportedCallbackSignaturesWithHeader)  // NOLINT
{
  XmlRpc::XmlRpcValue nodeParams;
  nodeParams.begin();  // make nodeParams a struct
  ros::NodeHandle nh;
  auto paramAdapter = std::make_shared<cras::XmlRpcValueGetParamAdapter>(nodeParams, "test_diag_utils");
  auto log = std::make_shared<cras::NodeLogHelper>();
  auto params = std::make_shared<cras::BoundParamHelper>(log, paramAdapter);

  CbTestHeader obj1, obj2, obj3;
  auto sharedObj = boost::make_shared<CbTestHeader>();

  std::string refCaptureLambdaConstPtrFrame {};
  std::string refCaptureLambdaPtrFrame {};
  std::string refCaptureLambdaConstRefFrame {};
  std::string refCaptureLambdaValueFrame {};
  std::string refCaptureLambdaConstEventFrame {};
  std::string refCaptureLambdaEventFrame {};

  cbConstPtrFrame = cbPtrFrame = cbConstRefFrame = cbValueFrame = cbConstEventFrame = cbEventFrame = "";
  cbConstPtrFrame = cbPtrFrame = cbConstRefFrame = cbValueFrame = cbConstEventFrame = cbEventFrame = "";
  lambdaHintsConstPtrFrame = lambdaHintsPtrFrame = lambdaHintsConstRefFrame = lambdaHintsValueFrame =
    lambdaHintsConstEventFrame = lambdaHintsEventFrame = "";
  noCaptureLambdaConstPtrFrame = noCaptureLambdaPtrFrame = noCaptureLambdaConstRefFrame = noCaptureLambdaValueFrame =
    noCaptureLambdaConstEventFrame = noCaptureLambdaEventFrame = "";
  valueCaptureLambdaConstPtrFrame = valueCaptureLambdaPtrFrame = valueCaptureLambdaConstRefFrame =
    valueCaptureLambdaValueFrame = valueCaptureLambdaConstEventFrame = valueCaptureLambdaEventFrame = "";

  DiagnosedSubscriber subConstPtr(nh, params, {9.0, 10}, "b", 0, &cbHeaderConstPtr);
  DiagnosedSubscriber subPtr(nh, params, {}, "b", 0, &cbHeaderPtr);
  DiagnosedSubscriber subConstRef(nh, params, {}, "b", 0, &cbHeaderConstRef);
  DiagnosedSubscriber subValue(nh, params, {}, "b", 0, &cbHeaderValue);
  DiagnosedSubscriber subConstEvent(nh, params, {}, "b", 0, &cbHeaderConstEvent);
  DiagnosedSubscriber subEvent(nh, params, {}, "b", 0, &cbHeaderEvent);
  EXPECT_TRUE((::std::is_same_v<decltype(subConstPtr), DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray>>));

  DiagnosedSubscriber subClassConstPtr(nh, params, {}, "b", 0, &CbTestHeader::cbConstPtr, &obj1);
  DiagnosedSubscriber subClassPtr(nh, params, {}, "b", 0, &CbTestHeader::cbPtr, &obj1);
  DiagnosedSubscriber subClassConstRef(nh, params, {}, "b", 0, &CbTestHeader::cbConstRef, &obj1);
  DiagnosedSubscriber subClassValue(nh, params, {}, "b", 0, &CbTestHeader::cbValue, &obj1);
  DiagnosedSubscriber subClassConstEvent(nh, params, {}, "b", 0, &CbTestHeader::cbConstEvent, &obj1);
  DiagnosedSubscriber subClassEvent(nh, params, {}, "b", 0, &CbTestHeader::cbEvent, &obj1);

  DiagnosedSubscriber subConstClassConstPtr(nh, params, {}, "b", 0, &CbTestHeader::constCbConstPtr, &obj1);
  DiagnosedSubscriber subConstClassPtr(nh, params, {}, "b", 0, &CbTestHeader::constCbPtr, &obj1);
  DiagnosedSubscriber subConstClassConstRef(nh, params, {}, "b", 0, &CbTestHeader::constCbConstRef, &obj1);
  DiagnosedSubscriber subConstClassValue(nh, params, {}, "b", 0, &CbTestHeader::constCbValue, &obj1);
  DiagnosedSubscriber subConstClassConstEvent(nh, params, {}, "b", 0, &CbTestHeader::constCbConstEvent, &obj1);
  DiagnosedSubscriber subConstClassEvent(nh, params, {}, "b", 0, &CbTestHeader::constCbEvent, &obj1);

  DiagnosedSubscriber subSharedClassConstPtr(nh, params, {}, "b", 0, &CbTestHeader::cbConstPtr, sharedObj);
  DiagnosedSubscriber subSharedClassPtr(nh, params, {}, "b", 0, &CbTestHeader::cbPtr, sharedObj);
  DiagnosedSubscriber subSharedClassConstRef(nh, params, {}, "b", 0, &CbTestHeader::cbConstRef, sharedObj);
  DiagnosedSubscriber subSharedClassValue(nh, params, {}, "b", 0, &CbTestHeader::cbValue, sharedObj);
  DiagnosedSubscriber subSharedClassConstEvent(nh, params, {}, "b", 0, &CbTestHeader::cbConstEvent, sharedObj);
  DiagnosedSubscriber subSharedClassEvent(nh, params, {}, "b", 0, &CbTestHeader::cbEvent, sharedObj);

  DiagnosedSubscriber subConstSharedClassConstPtr(nh, params, {}, "b", 0, &CbTestHeader::constCbConstPtr, sharedObj);
  DiagnosedSubscriber subConstSharedClassPtr(nh, params, {}, "b", 0, &CbTestHeader::constCbPtr, sharedObj);
  DiagnosedSubscriber subConstSharedClassConstRef(nh, params, {}, "b", 0, &CbTestHeader::constCbConstRef, sharedObj);
  DiagnosedSubscriber subConstSharedClassValue(nh, params, {}, "b", 0, &CbTestHeader::constCbValue, sharedObj);
  DiagnosedSubscriber subConstSharedClassConstEvent(nh, params, {}, "b", 0,
    &CbTestHeader::constCbConstEvent, sharedObj);
  DiagnosedSubscriber subConstSharedClassEvent(nh, params, {}, "b", 0, &CbTestHeader::constCbEvent, sharedObj);

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subNoCaptureLambdaConstPtr(nh, params, {}, "b", 0,
    [](const diagnostic_msgs::DiagnosticArrayConstPtr& m){noCaptureLambdaValueFrame = m->header.frame_id;});
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subNoCaptureLambdaPtr(nh, params, {}, "b", 0,
    [](const diagnostic_msgs::DiagnosticArrayPtr& m){noCaptureLambdaConstRefFrame = m->header.frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaConstRef(nh, params, {}, "b", 0,
    (void(*)(const diagnostic_msgs::DiagnosticArray&)) [](const diagnostic_msgs::DiagnosticArray& m){
      noCaptureLambdaPtrFrame = m.header.frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaValue(nh, params, {}, "b", 0,
    (void(*)(diagnostic_msgs::DiagnosticArray)) [](diagnostic_msgs::DiagnosticArray m){  // NOLINT
      noCaptureLambdaConstPtrFrame = m.header.frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaConstEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&))
      [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m){
        noCaptureLambdaEventFrame = m.getConstMessage()->header.frame_id;});
  DiagnosedSubscriber subNoCaptureLambdaEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&))
      [](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m){
        noCaptureLambdaConstEventFrame = m.getConstMessage()->header.frame_id;});

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subRefCaptureLambdaConstPtr(nh, params, {}, "b", 0,
    [&](const diagnostic_msgs::DiagnosticArrayConstPtr& m){refCaptureLambdaValueFrame = m->header.frame_id;});
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subRefCaptureLambdaPtr(nh, params, {}, "b", 0,
    [&](const diagnostic_msgs::DiagnosticArrayPtr& m){refCaptureLambdaConstRefFrame = m->header.frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      [&](const diagnostic_msgs::DiagnosticArray& m){refCaptureLambdaPtrFrame = m.header.frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      [&](diagnostic_msgs::DiagnosticArray m){refCaptureLambdaConstPtrFrame = m.header.frame_id;});  // NOLINT
  DiagnosedSubscriber subRefCaptureLambdaConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      [&](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m){
        refCaptureLambdaEventFrame = m.getConstMessage()->header.frame_id;});
  DiagnosedSubscriber subRefCaptureLambdaEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      [&](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m){
        refCaptureLambdaConstEventFrame = m.getConstMessage()->header.frame_id;});

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subValueCaptureLambdaConstPtr(nh, params, {}, "b", 0,
    [=](const diagnostic_msgs::DiagnosticArrayConstPtr& m){valueCaptureLambdaValueFrame = m->header.frame_id;});
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subValueCaptureLambdaPtr(nh, params, {}, "b", 0,
    [=](const diagnostic_msgs::DiagnosticArrayPtr& m){valueCaptureLambdaConstRefFrame = m->header.frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      [=](const diagnostic_msgs::DiagnosticArray& m){valueCaptureLambdaPtrFrame = m.header.frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      [=](diagnostic_msgs::DiagnosticArray m){valueCaptureLambdaConstPtrFrame = m.header.frame_id;});  // NOLINT
  DiagnosedSubscriber subValueCaptureLambdaConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      [=](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>& m){
        valueCaptureLambdaEventFrame = m.getConstMessage()->header.frame_id;});
  DiagnosedSubscriber subValueCaptureLambdaEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      [=](const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>& m){
        valueCaptureLambdaConstEventFrame = m.getConstMessage()->header.frame_id;});

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subLambdaConstPtr(nh, params, {}, "b", 0,
    lambdaHeaderCbConstPtr);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subLambdaPtr(nh, params, {}, "b", 0, lambdaHeaderCbPtr);
  DiagnosedSubscriber subLambdaConstRef(nh, params, {}, "b", 0,
    (void(*)(const diagnostic_msgs::DiagnosticArray&))lambdaHeaderCbConstRef);
  DiagnosedSubscriber subLambdaValue(nh, params, {}, "b", 0,
    (void(*)(diagnostic_msgs::DiagnosticArray))lambdaHeaderCbValue);
  DiagnosedSubscriber subLambdaConstEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&))lambdaHeaderCbConstEvent);
  DiagnosedSubscriber subLambdaEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&))lambdaHeaderCbEvent);

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subLambdaHintsConstPtr(nh, params, {}, "b", 0,
    lambdaHeaderHintsConstPtr, ros::TransportHints());
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subLambdaHintsPtr(nh, params, {}, "b", 0,
    lambdaHeaderHintsPtr, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsConstRef(nh, params, {}, "b", 0,
    (void(*)(const diagnostic_msgs::DiagnosticArray&))lambdaHeaderHintsConstRef, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsValue(nh, params, {}, "b", 0,
    (void(*)(diagnostic_msgs::DiagnosticArray))lambdaHeaderHintsValue, ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsConstEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&))lambdaHeaderHintsConstEvent,
    ros::TransportHints());
  DiagnosedSubscriber subLambdaHintsEvent(nh, params, {}, "b", 0,
    (void(*)(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&))lambdaHeaderHintsEvent, ros::TransportHints());

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subClassBindConstPtr(nh, params, {}, "b", 0,
    boost::bind(&CbTestHeader::cbConstPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subClassBindPtr(nh, params, {}, "b", 0,
    boost::bind(&CbTestHeader::cbPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      boost::bind(&CbTestHeader::cbConstRef, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      boost::bind(&CbTestHeader::cbValue, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      boost::bind(&CbTestHeader::cbConstEvent, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subClassBindEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      boost::bind(&CbTestHeader::cbEvent, &obj2, boost::placeholders::_1));

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subConstClassBindConstPtr(nh, params, {}, "b", 0,
    boost::bind(&CbTestHeader::constCbConstPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subConstClassBindPtr(nh, params, {}, "b", 0,
    boost::bind(&CbTestHeader::constCbPtr, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      boost::bind(&CbTestHeader::constCbConstRef, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      boost::bind(&CbTestHeader::constCbValue, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      boost::bind(&CbTestHeader::constCbConstEvent, &obj2, boost::placeholders::_1));
  DiagnosedSubscriber subConstClassBindEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      boost::bind(&CbTestHeader::constCbEvent, &obj2, boost::placeholders::_1));

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subSharedClassBindConstPtr(nh, params, {}, "b", 0,
    std::bind(&CbTestHeader::cbConstPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subSharedClassBindPtr(nh, params, {}, "b", 0,
    std::bind(&CbTestHeader::cbPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      std::bind(&CbTestHeader::cbConstRef, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      std::bind(&CbTestHeader::cbValue, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      std::bind(&CbTestHeader::cbConstEvent, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedClassBindEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      std::bind(&CbTestHeader::cbEvent, &obj3, std::placeholders::_1));

  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subSharedConstClassBindConstPtr(nh, params, {}, "b", 0,
    std::bind(&CbTestHeader::constCbConstPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subSharedConstClassBindPtr(nh, params, {}, "b", 0,
    std::bind(&CbTestHeader::constCbPtr, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindConstRef(nh, params, {}, "b", 0,
    (boost::function<void(const diagnostic_msgs::DiagnosticArray&)>)
      std::bind(&CbTestHeader::constCbConstRef, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindValue(nh, params, {}, "b", 0,
    (boost::function<void(diagnostic_msgs::DiagnosticArray)>)
      std::bind(&CbTestHeader::constCbValue, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindConstEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&)>)
      std::bind(&CbTestHeader::constCbConstEvent, &obj3, std::placeholders::_1));
  DiagnosedSubscriber subSharedConstClassBindEvent(nh, params, {}, "b", 0,
    (boost::function<void(const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&)>)
      std::bind(&CbTestHeader::constCbEvent, &obj3, std::placeholders::_1));

  DiagnosedSubscriber subBoostLambdaConstPtr(nh, params, {}, "b", 0, boostHeaderLambdaConstPtr);
  DiagnosedSubscriber subBoostLambdaPtr(nh, params, {}, "b", 0, boostHeaderLambdaPtr);
  DiagnosedSubscriber subBoostLambdaConstRef(nh, params, {}, "b", 0, boostHeaderLambdaConstRef);
  DiagnosedSubscriber subBoostLambdaValue(nh, params, {}, "b", 0, boostHeaderLambdaValue);
  DiagnosedSubscriber subBoostLambdaConstEvent(nh, params, {}, "b", 0, boostHeaderLambdaConstEvent);
  DiagnosedSubscriber subBoostLambdaEvent(nh, params, {}, "b", 0, boostHeaderLambdaEvent);

  ros::SubscribeOptions ops;
  ops.initByFullCallbackType<const diagnostic_msgs::DiagnosticArrayConstPtr &>("b", 10, &optionsHeaderCbConstPtr);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsConstPtr(nh, params, {}, ops);
  ops.initByFullCallbackType<const diagnostic_msgs::DiagnosticArrayPtr &>("b", 10, &optionsHeaderCbPtr);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsPtr(nh, params, {}, ops);
  ops.initByFullCallbackType<const diagnostic_msgs::DiagnosticArray&>("b", 10, &optionsHeaderCbConstRef);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsConstRef(nh, params, {}, ops);
  ops.initByFullCallbackType<diagnostic_msgs::DiagnosticArray>("b", 10, &optionsHeaderCbValue);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsValue(nh, params, {}, ops);
  ops.initByFullCallbackType<const ros::MessageEvent<diagnostic_msgs::DiagnosticArray const>&>(
    "b", 10, &optionsHeaderCbConstEvent);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsConstEvent(nh, params, {}, ops);
  ops.initByFullCallbackType<const ros::MessageEvent<diagnostic_msgs::DiagnosticArray>&>(
    "b", 10, &optionsHeaderCbEvent);
  DiagnosedSubscriber<diagnostic_msgs::DiagnosticArray> subOptionsEvent(nh, params, {}, ops);

  auto pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("b", 100);
  auto message = diagnostic_msgs::DiagnosticArray();
  message.header.frame_id = "testHeader";
  pub.publish(message);

  auto end = ros::WallTime::now() + ros::WallDuration(2);
  while (ros::WallTime::now() < end)
    ros::spinOnce();

  EXPECT_EQ(message.header.frame_id, cbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, cbPtrFrame);
  EXPECT_EQ(message.header.frame_id, cbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, cbValueFrame);
  EXPECT_EQ(message.header.frame_id, cbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, cbEventFrame);

  EXPECT_EQ(message.header.frame_id, optionsCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, optionsCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, optionsCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, optionsCbValueFrame);
  EXPECT_EQ(message.header.frame_id, optionsCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, optionsCbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj1.cbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj1.cbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj1.cbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj1.cbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj1.cbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj1.cbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj1.constCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj1.constCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj1.constCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj1.constCbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj1.constCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj1.constCbEventFrame);

  EXPECT_EQ(message.header.frame_id, sharedObj->cbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->cbPtrFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->cbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->cbValueFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->cbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->cbEventFrame);

  EXPECT_EQ(message.header.frame_id, sharedObj->constCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->constCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->constCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->constCbValueFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->constCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, sharedObj->constCbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj2.cbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj2.cbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj2.cbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj2.cbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj2.cbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj2.cbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj2.constCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj2.constCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj2.constCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj2.constCbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj2.constCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj2.constCbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj3.cbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj3.cbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj3.cbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj3.cbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj3.cbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj3.cbEventFrame);

  EXPECT_EQ(message.header.frame_id, obj3.constCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj3.constCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, obj3.constCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, obj3.constCbValueFrame);
  EXPECT_EQ(message.header.frame_id, obj3.constCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, obj3.constCbEventFrame);

  EXPECT_EQ(message.header.frame_id, noCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, noCaptureLambdaPtrFrame);
  EXPECT_EQ(message.header.frame_id, noCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.header.frame_id, noCaptureLambdaValueFrame);
  EXPECT_EQ(message.header.frame_id, noCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.header.frame_id, noCaptureLambdaEventFrame);

  EXPECT_EQ(message.header.frame_id, refCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, refCaptureLambdaPtrFrame);
  EXPECT_EQ(message.header.frame_id, refCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.header.frame_id, refCaptureLambdaValueFrame);
  EXPECT_EQ(message.header.frame_id, refCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.header.frame_id, refCaptureLambdaEventFrame);

  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaPtrFrame);
  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaConstRefFrame);
  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaValueFrame);
  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaConstEventFrame);
  EXPECT_EQ(message.header.frame_id, valueCaptureLambdaEventFrame);

  EXPECT_EQ(message.header.frame_id, lambdaCbConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, lambdaCbPtrFrame);
  EXPECT_EQ(message.header.frame_id, lambdaCbConstRefFrame);
  EXPECT_EQ(message.header.frame_id, lambdaCbValueFrame);
  EXPECT_EQ(message.header.frame_id, lambdaCbConstEventFrame);
  EXPECT_EQ(message.header.frame_id, lambdaCbEventFrame);

  EXPECT_EQ(message.header.frame_id, lambdaHintsConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, lambdaHintsPtrFrame);
  EXPECT_EQ(message.header.frame_id, lambdaHintsConstRefFrame);
  EXPECT_EQ(message.header.frame_id, lambdaHintsValueFrame);
  EXPECT_EQ(message.header.frame_id, lambdaHintsConstEventFrame);
  EXPECT_EQ(message.header.frame_id, lambdaHintsEventFrame);

  EXPECT_EQ(message.header.frame_id, boostLambdaConstPtrFrame);
  EXPECT_EQ(message.header.frame_id, boostLambdaPtrFrame);
  EXPECT_EQ(message.header.frame_id, boostLambdaConstRefFrame);
  EXPECT_EQ(message.header.frame_id, boostLambdaValueFrame);
  EXPECT_EQ(message.header.frame_id, boostLambdaConstEventFrame);
  EXPECT_EQ(message.header.frame_id, boostLambdaEventFrame);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_diag_utils");
  ros::Time::init();
  return RUN_ALL_TESTS();
}
