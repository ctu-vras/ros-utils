// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for compressed transport codec.
 * \author Martin Pecka
 */

#include <pluginlib/class_list_macros.h>

#include <image_transport_codecs/codecs/compressed_codec.h>
#include <image_transport_codecs/image_transport_codec_plugin.h>

namespace image_transport_codecs
{

class CompressedImageTransportCodecPlugin : public ImageTransportCodecPluginBase<CompressedCodec> {};

}

PLUGINLIB_EXPORT_CLASS(image_transport_codecs::CompressedImageTransportCodecPlugin,
                       image_transport_codecs::ImageTransportCodecPlugin)
