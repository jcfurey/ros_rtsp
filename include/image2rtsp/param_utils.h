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

// True for "h265"/"hevc" (case-insensitive); anything else is treated as H.264.
bool is_h265_codec(const std::string& codec);

// H.264/H.265 payloader tail placed after the encoder (pay element named pay0).
// codec: "h265"/"hevc" -> h265parse/rtph265pay, anything else -> h264.
std::string payloader_tail(const std::string& codec);

// True when the encoder keyword is one this package knows how to build a
// pipeline for. Single source of truth shared with encoder_fragment, so the
// "unknown encoder" warning can never disagree with the actual selection.
bool is_known_encoder(const std::string& encoder);

// Codec implied by a codec-specific encoder keyword: "h265" for
// nvh265enc/x265/x265enc, "h264" for nvh264enc/x264/x264enc, "" for
// codec-neutral keywords (nvenc, nv, sw, empty). Lets the caller honour
// 'encoder: nvh265enc' when 'codec' is omitted instead of silently
// producing H.264, and warn on an explicit encoder/codec mismatch.
std::string encoder_implied_codec(const std::string& encoder);

// Encoder element + output caps for the given encoder keyword, codec and bitrate.
//   encoder: x264 (default) | x265 | nvenc | x264enc | x265enc | nvh264enc | nvh265enc
//   codec:   h264 (default) | h265/hevc
//   bitrate: kbit/sec
// Every fragment is self-contained: it starts with videoconvert so it accepts
// any raw video upstream (cam sources included) and negotiates a format its
// encoder can actually encode.
std::string encoder_fragment(const std::string& encoder, const std::string& codec,
                             const std::string& bitrate);

}  // namespace params
}  // namespace image2rtsp

#endif
