#include <string>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <stdio.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <boost/bind/bind.hpp>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <image2rtsp.h>


using namespace std;
using namespace image2rtsp;


/* ---- tolerant parameter helpers -------------------------------------------
 * The config comes from YAML via XmlRpc, where "500" (string), 500 (int) and
 * 500.0 (double) are distinct types. The original code used static_cast<>, which
 * throws on a type mismatch and aborted the whole nodelet. These helpers accept
 * whatever the user wrote so one stray quote can't break everything. */
static std::string xmlrpc_to_string(XmlRpc::XmlRpcValue& v) {
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

static std::string member_string(XmlRpc::XmlRpcValue& stream, const char* key, const std::string& def="") {
    return stream.hasMember(key) ? xmlrpc_to_string(stream[key]) : def;
}

static int member_int(XmlRpc::XmlRpcValue& stream, const char* key, int def) {
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


void Image2RTSPNodelet::onInit() {
    // Common tail shared by every stream. h264parse + config-interval makes the
    // encoded SPS/PPS available to clients that connect mid-stream (shared factory).
    const string pipeline_tail = " ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )";

    NODELET_INFO("Initializing image2rtsp nodelet...");

    if (getenv((char*)"GST_DEBUG") == NULL) {
        // set GST_DEBUG to warning if unset
        putenv((char*)"GST_DEBUG=*:1");
    }

    ros::NodeHandle& nh = getPrivateNodeHandle();

    // port: accept an int or a string, default "8554".
    this->port = "8554";
    if (nh.hasParam("port")) {
        XmlRpc::XmlRpcValue p;
        nh.getParam("port", p);
        std::string s = xmlrpc_to_string(p);
        if (!s.empty()) this->port = s;
    }

    // streams: must be a mapping. If it isn't there, say so clearly and stop -
    // do NOT ROS_ASSERT (that would abort the whole nodelet manager).
    XmlRpc::XmlRpcValue streams;
    if (!nh.getParam("streams", streams) ||
        streams.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        NODELET_FATAL("No valid 'streams' parameter under namespace '%s'. "
                      "Make sure the launch file loads your stream_setup.yaml into this "
                      "node (rosparam load ... file=...). No RTSP streams will be served.",
                      nh.getNamespace().c_str());
        return;
    }

    video_mainloop_start();
    rtsp_server = rtsp_server_create(this->port);

    int registered = 0;
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin(); it != streams.end(); ++it)
    {
        const std::string& name = it->first;
        try {
            XmlRpc::XmlRpcValue stream = streams[name];
            if (stream.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                NODELET_ERROR("Stream '%s' is not a mapping - skipping.", name.c_str());
                continue;
            }

            std::string type       = member_string(stream, "type");
            std::string mountpoint  = stream_mountpoint(stream, name);
            std::string bitrate     = std::to_string(member_int(stream, "bitrate", 500));
            std::string encoder     = build_encoder(stream, bitrate);
            std::string pipeline;

            if (type == "cam") {
                std::string source = member_string(stream, "source");
                if (source.empty()) {
                    NODELET_ERROR("Stream '%s' (cam) has no 'source' - skipping.", name.c_str());
                    continue;
                }
                pipeline = "( " + source + " ! " + encoder + pipeline_tail;
                rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
            }
            else if (type == "topic") {
                std::string source = member_string(stream, "source");
                if (source.empty()) {
                    NODELET_ERROR("Stream '%s' (topic) has no 'source' topic - skipping.", name.c_str());
                    continue;
                }
                /* Track connected clients so we only subscribe while someone is watching. */
                num_of_clients[mountpoint] = 0;
                appsrc[mountpoint] = NULL;

                /* Optional 'caps': when set, rescale / cap the framerate before encoding.
                 * When omitted, serve the topic at its native resolution - the caps come
                 * from the incoming sensor_msgs/Image itself. */
                std::string scale = "";
                if (stream.hasMember("caps"))
                    scale = "videoscale ! " + xmlrpc_to_string(stream["caps"]) + " ! ";

                pipeline = "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 "
                           "max-bytes=1000 is-live=true ! videoconvert ! " + scale + encoder + pipeline_tail;
                rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), (GstElement **)&(appsrc[mountpoint]));
            }
            else {
                NODELET_ERROR("Stream '%s' has unknown type '%s' (expected 'topic' or 'cam') - skipping.",
                              name.c_str(), type.c_str());
                continue;
            }

            gchar *addr = gst_rtsp_server_get_address(rtsp_server);
            NODELET_INFO("Stream '%s' available at rtsp://%s:%s%s",
                         name.c_str(), addr, this->port.c_str(), mountpoint.c_str());
            g_free(addr);
            registered++;
        }
        catch (const XmlRpc::XmlRpcException& e) {
            NODELET_ERROR("Stream '%s' skipped - bad parameter: %s "
                          "(check field types, e.g. 'bitrate' must be a number).",
                          name.c_str(), e.getMessage().c_str());
        }
    }

    NODELET_INFO("image2rtsp: %d of %d stream(s) registered on port %s.",
                 registered, (int)streams.size(), this->port.c_str());
    if (registered == 0)
        NODELET_ERROR("No streams were registered. The RTSP server is up on port %s but has "
                      "no mount points, so clients will get 'no factory for path ...'.",
                      this->port.c_str());
}

/* RTSP mount point for a stream. Defaults to "/<stream name>" when the stream
 * omits the 'mountpoint' parameter. Used by onInit() and the client connect/
 * disconnect handlers so they all agree on the same path.
 *
 * gst-rtsp-server requires the path to start with '/'
 * (gst_rtsp_mount_points_add_factory: g_return_if_fail(path[0] == '/')). A path
 * that doesn't is silently rejected there and the mount never registers ("no
 * factory for path ..."). So normalise here: strip whitespace, turn backslashes
 * into forward slashes, and guarantee a single leading '/'. */
std::string Image2RTSPNodelet::stream_mountpoint(XmlRpc::XmlRpcValue& stream, const std::string& name) {
    std::string mp = stream.hasMember("mountpoint") ? xmlrpc_to_string(stream["mountpoint"]) : name;
    std::replace(mp.begin(), mp.end(), '\\', '/');
    size_t a = mp.find_first_not_of(" \t\r\n");
    size_t b = mp.find_last_not_of(" \t\r\n");
    mp = (a == std::string::npos) ? "" : mp.substr(a, b - a + 1);
    if (mp.empty() || mp.front() != '/')
        mp = "/" + mp;
    return mp;
}

/* Build the GStreamer encoder fragment for a stream (encoder element through the
 * output caps, i.e. everything between the raw-video source and rtph264pay).
 *
 * Selected with the optional per-stream parameters:
 *   encoder: x264   (default) - software H.264, works everywhere, no GPU required
 *   encoder: nvenc            - NVIDIA hardware H.264 (gst-plugins-bad "nvcodec", nvh264enc)
 *   encoder_override: "..."   - full custom encoder+caps fragment, used verbatim
 *                               (for VA-API, Jetson nvv4l2h264enc, etc.)
 *
 * bitrate is forwarded unchanged in kbit/sec, matching both x264enc and nvh264enc.
 */
std::string Image2RTSPNodelet::build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate) {
    // Full manual override wins: the user supplies the whole encoder + caps fragment.
    if (stream.hasMember("encoder_override")) {
        return xmlrpc_to_string(stream["encoder_override"]);
    }

    std::string encoder = member_string(stream, "encoder", "x264");
    std::transform(encoder.begin(), encoder.end(), encoder.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    if (encoder == "nvenc" || encoder == "nvh264enc" || encoder == "nv") {
        // NVIDIA desktop hardware encoder. videoconvert guarantees an input format
        // nvenc accepts; bitrate stays in kbit/sec.
        NODELET_INFO("Using NVIDIA hardware encoder (nvh264enc)");
        return "videoconvert ! nvh264enc bitrate=" + bitrate +
               " gop-size=30 rc-mode=cbr preset=low-latency-hq ! video/x-h264, profile=baseline";
    }

    if (encoder != "x264" && encoder != "x264enc" && encoder != "sw") {
        NODELET_WARN("Unknown encoder '%s', falling back to software x264. "
                     "Use 'encoder_override' for a fully custom pipeline.", encoder.c_str());
    }

    // Default: software x264 (identical to the original pipeline).
    return "x264enc tune=zerolatency bitrate=" + bitrate +
           " key-int-max=30 ! video/x-h264, profile=baseline";
}

/* Modified from https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp */
GstCaps* Image2RTSPNodelet::gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg)
{
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    static const ros::M_string known_formats = {{
        {sensor_msgs::image_encodings::RGB8, "RGB"},
        {sensor_msgs::image_encodings::RGB16, "RGB16"},
        {sensor_msgs::image_encodings::RGBA8, "RGBA"},
        {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGR16, "BGR16"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
    }};

    if (msg->is_bigendian) {
        ROS_ERROR_THROTTLE(5.0, "GST: big endian image format is not supported");
        return nullptr;
    }

    auto format = known_formats.find(msg->encoding);
    if (format == known_formats.end()) {
        ROS_ERROR_THROTTLE(5.0, "GST: image encoding '%s' is not supported (use rgb8/bgr8/mono8/...)",
                           msg->encoding.c_str());
        return nullptr;
    }

    return gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, format->second.c_str(),
            "width", G_TYPE_INT, msg->width,
            "height", G_TYPE_INT, msg->height,
            "framerate", GST_TYPE_FRACTION, 10, 1,
            nullptr);
}


void Image2RTSPNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic) {
    // Only push once a client is connected and the media pipeline's appsrc exists.
    if (appsrc[topic] == NULL)
        return;

    GstCaps *caps = gst_caps_new_from_image(msg);
    if (caps == NULL)   // unsupported encoding / big-endian (already logged, throttled)
        return;

    gst_app_src_set_caps(appsrc[topic], caps);
    gst_caps_unref(caps);   // set_caps takes its own ref; release ours (was leaked before)

    GstBuffer *buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

    gst_app_src_push_buffer(appsrc[topic], buf);   // takes ownership of buf
}


void Image2RTSPNodelet::url_connected(string url) {
    NODELET_INFO("Client connected: %s", url.c_str());
    ros::NodeHandle& nh = getPrivateNodeHandle();

    XmlRpc::XmlRpcValue streams;
    if (!nh.getParam("streams", streams) || streams.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        return;

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin(); it != streams.end(); ++it)
    {
        try {
            XmlRpc::XmlRpcValue stream = streams[it->first];
            if (stream.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

            std::string type       = member_string(stream, "type");
            std::string mountpoint = stream_mountpoint(stream, it->first);
            std::string source     = member_string(stream, "source");

            if (type == "topic" && url == mountpoint && !source.empty()) {
                if (num_of_clients[url] == 0) {
                    NODELET_INFO("Subscribing to '%s' for stream %s", source.c_str(), url.c_str());
                    subs[url] = nh.subscribe<sensor_msgs::Image>(
                        source, 1,
                        boost::bind(&Image2RTSPNodelet::imageCallback, this, boost::placeholders::_1, url));
                }
                num_of_clients[url]++;
            }
        }
        catch (const XmlRpc::XmlRpcException& e) {
            NODELET_WARN("url_connected: skipping malformed stream '%s': %s",
                         it->first.c_str(), e.getMessage().c_str());
        }
    }
}

void Image2RTSPNodelet::url_disconnected(string url) {
    NODELET_INFO("Client disconnected: %s", url.c_str());
    ros::NodeHandle& nh = getPrivateNodeHandle();

    XmlRpc::XmlRpcValue streams;
    if (!nh.getParam("streams", streams) || streams.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        return;

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = streams.begin(); it != streams.end(); ++it)
    {
        try {
            XmlRpc::XmlRpcValue stream = streams[it->first];
            if (stream.getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

            std::string mountpoint = stream_mountpoint(stream, it->first);
            if (url == mountpoint) {
                if (num_of_clients[url] > 0) num_of_clients[url]--;
                if (num_of_clients[url] == 0) {
                    // No-one else is connected. Stop the subscription.
                    subs[url].shutdown();
                    appsrc[url] = NULL;
                }
            }
        }
        catch (const XmlRpc::XmlRpcException& e) {
            NODELET_WARN("url_disconnected: skipping malformed stream '%s': %s",
                         it->first.c_str(), e.getMessage().c_str());
        }
    }
}

void Image2RTSPNodelet::print_info(char *s) {
    NODELET_INFO("%s",s);
}

void Image2RTSPNodelet::print_error(char *s) {
    NODELET_ERROR("%s",s);
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
