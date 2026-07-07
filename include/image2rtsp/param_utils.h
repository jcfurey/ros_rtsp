#ifndef IMAGE2RTSP_PARAM_UTILS_H
#define IMAGE2RTSP_PARAM_UTILS_H

// Pure, runtime-free helpers for parsing the stream config and building the
// GStreamer pipeline fragments. Kept free of ROS/GStreamer runtime so they can
// be unit-tested (see test/test_param_utils.cpp) - these are exactly the bits
// that have historically been fragile (mountpoint slashes, numeric types, ...).

#include <string>
#include <xmlrpcpp/XmlRpcValue.h>

namespace image2rtsp {
namespace params {

// GStreamer element name given to the encoder in every pipeline, so the running
// nodelet can look it up (gst_bin_get_by_name) and retune "bitrate" at runtime.
constexpr const char* ENCODER_NAME = "venc0";

// Convert a scalar XmlRpc value (string/int/double/bool) to a string; "" otherwise.
std::string to_string(XmlRpc::XmlRpcValue& v);

// Read a struct member as a string (tolerant of type), or def if absent.
std::string member_string(XmlRpc::XmlRpcValue& stream, const char* key, const std::string& def = "");

// Read a struct member as an int, accepting int / double / quoted-string;
// returns def if absent or unparseable.
int member_int(XmlRpc::XmlRpcValue& stream, const char* key, int def);

// RTSP mount path: trim whitespace, turn '\\' into '/', guarantee a single
// leading '/'. gst-rtsp-server rejects any path not starting with '/'.
std::string normalize_mountpoint(const std::string& raw);

// H.264/H.265 payloader tail placed after the encoder (pay element named pay0).
// codec: "h265"/"hevc" -> h265parse/rtph265pay, anything else -> h264.
std::string payloader_tail(const std::string& codec);

// Encoder element + output caps for the given encoder keyword, codec and bitrate.
//   encoder: x264 (default) | nvenc | x264enc | nvh264enc | ...
//   codec:   h264 (default) | h265/hevc
//   bitrate: kbit/sec
std::string encoder_fragment(const std::string& encoder, const std::string& codec,
                             const std::string& bitrate);

}  // namespace params
}  // namespace image2rtsp

#endif
