# image_transport_codecs

Image transport plugins available as direct APIs in C, C++ and Python.

This library extends the ideas of [image_transport](http://wiki.ros.org/image_transport) to also provide C, C++ and Python APIs, as not all use-cases involving `image_transport` automatically involve a running ROS system (e.g. bag file postprocessing).

With this library, you can decode and encode the compressed messages directly in your code with no need for a separate node just for this mundane work.

## Example C++ usage

```c++
#include <image_transport_codecs/image_transport_codecs.h>

image_transport_codecs::ImageTransportCodecs codecs;
sensor_msgs::Image raw = ...;  // fill the image
auto result = codecs.encode(raw, "compressed");  // also check encodeTyped<M>()
if (!result)
{
  ROS_ERROR_STREAM("Error encoding image: " << result.error();
  return false;
}
topic_tools::ShapeShifter compressed = result.value();  // ShapeShifter to allow any encoded message type

// beware, for decoding, we do not specify "raw", but the codec used for encoding
auto result2 = codecs.decode(compressed, "compressed");  // also check decodeTyped<M>()
if (!result2)
{
  ROS_ERROR_STREAM("Error encoding image: " << result2.error();
  return false;
}
sensor_msgs::Image raw2 = result2.value();
```

Or you can work with a particular codec directly if you know it at compile time. This should lead to highest performance.

```c++
#include <image_transport_codecs/codecs/compressed_codec.h>

image_transport_codecs::CompressedCodec codec;
sensor_msgs::Image raw = ...;  // fill the image
auto result = codec.encode(raw);
if (!result)
{
  ROS_ERROR_STREAM("Error encoding image: " << result.error();
  return false;
}
sensor_msgs::CompressedImage compressed = result.value();

// beware, for decoding, we do not specify "raw", but the codec used for encoding
auto result2 = codec.decode(compressed);
if (!result2)
{
  ROS_ERROR_STREAM("Error encoding image: " << result2.error();
  return false;
}
sensor_msgs::Image raw2 = result2.value();
```

If you wonder what is the type of `result`, it is [`cras::expected<CompressedImage, std::string>`](http://docs.ros.org/en/api/cras_cpp_common/html/expected_8hpp.html). `cras::expected` is a shim for [`std::expected`](https://en.cppreference.com/w/cpp/utility/expected) and expresses that the function either returns a value (in the expected case), or an error (in exceptional cases). So it has a lot of similarities to exceptions, however it doesn't come with the large performance penalties and unsure program flow.

## Example Python usage

```python
import rospy
from image_transport_codecs import decode, encode
from sensor_msgs.msg import CompressedImage, Image

raw = Image()
... # fill the image
compressed, err = encode(raw, "compressed")
if compressed is None:
  rospy.logerr("Error encoding image: " + err)
  return False
# work with the CompressedImage instance in variable compressed

# beware, for decoding, we do not specify "raw", but the codec used for encoding
raw2, err = decode(compressed, "compressed")
if raw2 is None:
  rospy.logerr("Error encoding image: " + err)
  return False
# work with the Image instance in variable raw2

# or you can work directly with a particular codec if you know which one you want in advance:
from image_transport_codecs import compressed_codec

compressed2, err = compressed_codec.encode(raw)
```

## The Plugins and Codecs

First, let's settle on some terminology:

- `codec` is a program that takes a raw image and converts it to a compressed byte stream, and vice versa
- `codec plugin` is a pluginlib-style ROS plugin that registers a `codec` so that it can be used via the generic interface explained in the first example in C++ section.
- `image_transport plugin` is a widely used standard for plugins that ROS image publishers and subscribers can use to ease conversion of the various compressed formats

This library handles `codecs` and `codec plugins`. There is unfortunately no way it could hook into classical `image_transport plugins` and offer their functionality as a codec.
Therefore, although the codec names are equal to the `image_transport` names, so if you use custom `image_transport plugins`, their functionality will not be available via this library out of the box.
If you however do not mind some architectural changes, it would be best to base your `image_transport plugin` on a `codec` and expose also the corresponding `codec` plugin.
This way, the compression mechanism will be available via a Python, C++ and ROS API.

## Performance

To provide a generic C++ API, the generic access via `ImageTransportCodecs` class uses `ShapeShifter`s to represent the messages.
Thus, each input compressed message is serialized before decoding, and each freshly compressed output image is serialized after being encodec.
The same limitation applies to the C and Python APIs.

The direct C++ APIs (but only the C++ ones) mitigate this performance bottle-neck and are the only APIs that provide full performance.