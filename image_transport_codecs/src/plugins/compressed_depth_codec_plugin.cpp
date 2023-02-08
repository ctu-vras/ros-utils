// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for compressedDepth transport codec.
 * \author Martin Pecka
 */

#include <pluginlib/class_list_macros.h>

#include <image_transport_codecs/codecs/compressed_depth_codec.h>
#include <image_transport_codecs/image_transport_codec_plugin.h>

namespace image_transport_codecs
{

class CompressedDepthImageTransportCodecPlugin : public ImageTransportCodecPluginBase<CompressedDepthCodec> {};

}

PLUGINLIB_EXPORT_CLASS(image_transport_codecs::CompressedDepthImageTransportCodecPlugin,
                       image_transport_codecs::ImageTransportCodecPlugin)
