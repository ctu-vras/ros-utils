// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief C interface for sensor_msgs/image_encodings.h
 * \author Martin Pecka
 */

#include <sensor_msgs/image_encodings.h>

namespace ie = sensor_msgs::image_encodings;

extern "C" {

const char* RGB8 = ie::RGB8.c_str();
const char* RGBA8 = ie::RGBA8.c_str();
const char* RGB16 = ie::RGB16.c_str();
const char* RGBA16 = ie::RGBA16.c_str();
const char* BGR8 = ie::BGR8.c_str();
const char* BGRA8 = ie::BGRA8.c_str();
const char* BGR16 = ie::BGR16.c_str();
const char* BGRA16 = ie::BGRA16.c_str();
const char* MONO8 = ie::MONO8.c_str();
const char* MONO16 = ie::MONO16.c_str();
const char* TYPE_8UC1 = ie::TYPE_8UC1.c_str();
const char* TYPE_8UC2 = ie::TYPE_8UC2.c_str();
const char* TYPE_8UC3 = ie::TYPE_8UC3.c_str();
const char* TYPE_8UC4 = ie::TYPE_8UC4.c_str();
const char* TYPE_8SC1 = ie::TYPE_8SC1.c_str();
const char* TYPE_8SC2 = ie::TYPE_8SC2.c_str();
const char* TYPE_8SC3 = ie::TYPE_8SC3.c_str();
const char* TYPE_8SC4 = ie::TYPE_8SC4.c_str();
const char* TYPE_16UC1 = ie::TYPE_16UC1.c_str();
const char* TYPE_16UC2 = ie::TYPE_16UC2.c_str();
const char* TYPE_16UC3 = ie::TYPE_16UC3.c_str();
const char* TYPE_16UC4 = ie::TYPE_16UC4.c_str();
const char* TYPE_16SC1 = ie::TYPE_16SC1.c_str();
const char* TYPE_16SC2 = ie::TYPE_16SC2.c_str();
const char* TYPE_16SC3 = ie::TYPE_16SC3.c_str();
const char* TYPE_16SC4 = ie::TYPE_16SC4.c_str();
const char* TYPE_32SC1 = ie::TYPE_32SC1.c_str();
const char* TYPE_32SC2 = ie::TYPE_32SC2.c_str();
const char* TYPE_32SC3 = ie::TYPE_32SC3.c_str();
const char* TYPE_32SC4 = ie::TYPE_32SC4.c_str();
const char* TYPE_32FC1 = ie::TYPE_32FC1.c_str();
const char* TYPE_32FC2 = ie::TYPE_32FC2.c_str();
const char* TYPE_32FC3 = ie::TYPE_32FC3.c_str();
const char* TYPE_32FC4 = ie::TYPE_32FC4.c_str();
const char* TYPE_64FC1 = ie::TYPE_64FC1.c_str();
const char* TYPE_64FC2 = ie::TYPE_64FC2.c_str();
const char* TYPE_64FC3 = ie::TYPE_64FC3.c_str();
const char* TYPE_64FC4 = ie::TYPE_64FC4.c_str();
const char* BAYER_RGGB8 = ie::BAYER_RGGB8.c_str();
const char* BAYER_BGGR8 = ie::BAYER_BGGR8.c_str();
const char* BAYER_GBRG8 = ie::BAYER_GBRG8.c_str();
const char* BAYER_GRBG8 = ie::BAYER_GRBG8.c_str();
const char* BAYER_RGGB16 = ie::BAYER_RGGB16.c_str();
const char* BAYER_BGGR16 = ie::BAYER_BGGR16.c_str();
const char* BAYER_GBRG16 = ie::BAYER_GBRG16.c_str();
const char* BAYER_GRBG16 = ie::BAYER_GRBG16.c_str();
const char* YUV422 = ie::YUV422.c_str();

bool isColor(const char* encoding)
{
  return ie::isColor(encoding);
}

bool isMono(const char* encoding)
{
  return ie::isMono(encoding);
}

bool isBayer(const char* encoding)
{
  return ie::isBayer(encoding);
}

bool hasAlpha(const char* encoding)
{
  return ie::hasAlpha(encoding);
}

int numChannels(const char* encoding)
{
  return ie::numChannels(encoding);
}

int bitDepth(const char* encoding)
{
  return ie::bitDepth(encoding);
}

}
