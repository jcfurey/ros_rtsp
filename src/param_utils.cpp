#include <image2rtsp/param_utils.h>

#include <algorithm>
#include <cctype>
#include <sstream>

namespace image2rtsp {
namespace params {

static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}

std::string to_string(XmlRpc::XmlRpcValue& v) {
    switch (v.getType()) {
        case XmlRpc::XmlRpcValue::TypeString:  return static_cast<std::string>(v);
        case XmlRpc::XmlRpcValue::TypeInt:     return std::to_string(static_cast<int>(v));
        case XmlRpc::XmlRpcValue::TypeBoolean: return static_cast<bool>(v) ? "true" : "false";
        case XmlRpc::XmlRpcValue::TypeDouble: {
            std::ostringstream o; o << static_cast<double>(v); return o.str();
        }
        default: return "";
    }
}

std::string member_string(XmlRpc::XmlRpcValue& stream, const char* key, const std::string& def) {
    return stream.hasMember(key) ? to_string(stream[key]) : def;
}

int member_int(XmlRpc::XmlRpcValue& stream, const char* key, int def) {
    if (!stream.hasMember(key)) return def;
    XmlRpc::XmlRpcValue& v = stream[key];
    switch (v.getType()) {
        case XmlRpc::XmlRpcValue::TypeInt:    return static_cast<int>(v);
        case XmlRpc::XmlRpcValue::TypeDouble: return static_cast<int>(static_cast<double>(v));
        case XmlRpc::XmlRpcValue::TypeString:
            try { return std::stoi(static_cast<std::string>(v)); } catch (...) { return def; }
        default: return def;
    }
}

std::string normalize_mountpoint(const std::string& raw) {
    std::string mp = raw;
    std::replace(mp.begin(), mp.end(), '\\', '/');
    size_t a = mp.find_first_not_of(" \t\r\n");
    size_t b = mp.find_last_not_of(" \t\r\n");
    mp = (a == std::string::npos) ? "" : mp.substr(a, b - a + 1);
    if (mp.empty() || mp.front() != '/')
        mp = "/" + mp;
    return mp;
}

bool is_h265_codec(const std::string& codec) {
    std::string c = lower(codec);
    return c == "h265" || c == "hevc";
}

std::string payloader_tail(const std::string& codec) {
    if (is_h265_codec(codec))
        return " ! h265parse ! rtph265pay name=pay0 pt=96 config-interval=1 )";
    return " ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )";
}

/* Alias sets. These three helpers are the ONLY place encoder keywords are
 * interpreted; is_known_encoder and encoder_fragment share them so the
 * "unknown encoder" warning can never disagree with the pipeline actually built. */
static bool is_nv_encoder(const std::string& enc_lower) {
    return enc_lower == "nvenc" || enc_lower == "nv" ||
           enc_lower == "nvh264enc" || enc_lower == "nvh265enc";
}
static bool is_sw_encoder(const std::string& enc_lower) {
    return enc_lower.empty() || enc_lower == "sw" ||
           enc_lower == "x264" || enc_lower == "x264enc" ||
           enc_lower == "x265" || enc_lower == "x265enc";
}

bool is_known_encoder(const std::string& encoder) {
    std::string e = lower(encoder);
    return is_nv_encoder(e) || is_sw_encoder(e);
}

std::string encoder_implied_codec(const std::string& encoder) {
    std::string e = lower(encoder);
    if (e == "nvh265enc" || e == "x265" || e == "x265enc") return "h265";
    if (e == "nvh264enc" || e == "x264" || e == "x264enc") return "h264";
    return "";   // codec-neutral (nvenc, nv, sw, empty, unknown)
}

std::string encoder_fragment(const std::string& encoder, const std::string& codec,
                             const std::string& bitrate) {
    std::string enc = lower(encoder.empty() ? "x264" : encoder);
    bool h265 = is_h265_codec(codec);
    std::string out_caps = h265 ? "video/x-h265" : "video/x-h264, profile=baseline";

    // The encoder element is named ENCODER_NAME so the running nodelet can grab it
    // (gst_bin_get_by_name) and retune its "bitrate" property live via dynamic_reconfigure.
    // Every fragment starts with its own videoconvert: cam sources feed the fragment
    // directly (no caller-side conversion), and for topic streams the extra convert
    // negotiates passthrough, so it costs nothing.
    if (is_nv_encoder(enc)) {
        std::string el = h265 ? "nvh265enc" : "nvh264enc";
        return "videoconvert ! " + el + " name=" + ENCODER_NAME + " bitrate=" + bitrate +
               " gop-size=30 rc-mode=cbr preset=low-latency-hq ! " + out_caps;
    }

    // software x264 / x265 (both take tune=zerolatency, bitrate in kbit/sec, key-int-max)
    std::string el = h265 ? "x265enc" : "x264enc";
    // x265enc advertises both I420 and Y444 input; a free-running videoconvert may pick
    // Y444, which the default (main) profile can't encode ("Failed to find correct
    // level, tier or profile in VPS") and media-prepare fails. Pin I420 so the main
    // profile is always valid. x264's baseline output caps already force I420 upstream,
    // so it needs no extra filter.
    std::string in_caps = h265 ? "video/x-raw,format=I420 ! " : "";
    return "videoconvert ! " + in_caps + el + " name=" + ENCODER_NAME +
           " tune=zerolatency bitrate=" + bitrate + " key-int-max=30 ! " + out_caps;
}

}  // namespace params
}  // namespace image2rtsp
