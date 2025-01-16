// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief C interface for sensor_msgs/distortion_models.h
 * \author Martin Pecka
 */

#include <sensor_msgs/distortion_models.h>

namespace dm = sensor_msgs::distortion_models;

extern "C" {

const char* PLUMB_BOB = dm::PLUMB_BOB.c_str();
const char* RATIONAL_POLYNOMIAL = dm::RATIONAL_POLYNOMIAL.c_str();
const char* EQUIDISTANT = dm::EQUIDISTANT.c_str();

}
