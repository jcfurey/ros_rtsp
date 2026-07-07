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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <image2rtsp.h>
#include <image2rtsp/param_utils.h>


using namespace std;
using namespace image2rtsp;

// Tolerant parameter + pipeline helpers live in param_utils (unit-tested).
namespace pu = image2rtsp::params;

/* Is a topic currently advertised on the ROS graph? Used only for a friendly
 * startup warning - a topic may still appear later, so this never blocks a stream. */
static bool topic_is_advertised(const std::string& topic) {
    ros::master::V_TopicInfo infos;
    if (!ros::master::getTopics(infos)) return true;   // can't reach master to check
    for (size_t i = 0; i < infos.size(); ++i)
        if (infos[i].name == topic) return true;
    return false;
}


void Image2RTSPNodelet::onInit() {
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
        std::string s = pu::to_string(p);
        if (!s.empty()) this->port = s;
    }

    // Optional auth / TLS (applies to the whole server). Both are off unless set.
    //   auth: { user: <name>, pass: <secret> }   -> require basic auth on every mount
    //   tls:  { cert: <path-to-PEM> }            -> serve rtsps:// (cert PEM holds cert+key)
    if (nh.hasParam("auth")) {
        XmlRpc::XmlRpcValue a;
        nh.getParam("auth", a);
        if (a.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            auth_user = pu::member_string(a, "user");
            auth_pass = pu::member_string(a, "pass");
        } else {
            NODELET_WARN("'auth' parameter is not a mapping (expected {user, pass}) - ignoring.");
        }
    }
    if (nh.hasParam("tls")) {
        XmlRpc::XmlRpcValue t;
        nh.getParam("tls", t);
        if (t.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            tls_cert_path = pu::member_string(t, "cert");
        else
            NODELET_WARN("'tls' parameter is not a mapping (expected {cert}) - ignoring.");
    }
    auth_enabled = !auth_user.empty();

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

            std::string type       = pu::member_string(stream, "type");
            std::string mountpoint  = stream_mountpoint(stream, name);
            std::string bitrate     = std::to_string(pu::member_int(stream, "bitrate", 500));
            std::string codec       = pu::member_string(stream, "codec", "h264");
            std::string encoder     = build_encoder(stream, bitrate);   // reads codec too
            std::string tail        = pu::payloader_tail(codec);
            std::string pipeline;

            if (type == "cam") {
                std::string source = pu::member_string(stream, "source");
                if (source.empty()) {
                    NODELET_ERROR("Stream '%s' (cam) has no 'source' - skipping.", name.c_str());
                    continue;
                }
                pipeline = "( " + source + " ! " + encoder + tail;
                rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
            }
            else if (type == "topic") {
                std::string source = pu::member_string(stream, "source");
                if (source.empty()) {
                    NODELET_ERROR("Stream '%s' (topic) has no 'source' topic - skipping.", name.c_str());
                    continue;
                }
                if (!topic_is_advertised(source))
                    NODELET_WARN("Stream '%s': source topic '%s' is not currently advertised. "
                                 "The stream is registered anyway; clients will receive video once "
                                 "something publishes to it (check the topic name/namespace if not).",
                                 name.c_str(), source.c_str());
                {
                    /* Track connected clients so we only subscribe while someone is watching. */
                    std::lock_guard<std::mutex> lk(mtx);
                    num_of_clients[mountpoint] = 0;
                    appsrc[mountpoint] = NULL;
                    topic_source[mountpoint] = source;
                    receiving[mountpoint] = false;
                    stream_codec[mountpoint] = codec;
                    // Advertised framerate for the appsrc caps (default 10 fps).
                    framerate[mountpoint] = pu::member_int(stream, "framerate", 10);
                }

                /* Optional 'caps': when set, rescale / cap the framerate before encoding.
                 * When omitted, serve the topic at its native resolution - the caps come
                 * from the incoming sensor_msgs/Image itself. */
                std::string scale = "";
                if (stream.hasMember("caps"))
                    scale = "videoscale ! " + pu::to_string(stream["caps"]) + " ! ";

                pipeline = "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 "
                           "max-bytes=1000 is-live=true ! videoconvert ! " + scale + encoder + tail;
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

    // Watchdog: while a client is connected, periodically warn if the source topic
    // has no publisher, so an unreachable/mistyped topic is obvious in the log.
    // Also publish per-stream status on /diagnostics so `rostopic echo /diagnostics`
    // or rqt_robot_monitor shows client counts and whether frames are flowing.
    if (!topic_source.empty()) {
        watchdog   = nh.createTimer(ros::Duration(5.0), &Image2RTSPNodelet::checkTopics, this);
        diag_pub   = getNodeHandle().advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
        diag_timer = nh.createTimer(ros::Duration(1.0), &Image2RTSPNodelet::publishDiagnostics, this);

        // Live bitrate tuning (rqt_reconfigure). The server fires the callback once
        // with the default (bitrate=0 = keep configured bitrate), so this does not
        // clobber the per-stream YAML values until a client actually changes it.
        dyn_srv.reset(new dynamic_reconfigure::Server<ros_rtsp::BitrateConfig>(nh));
        dyn_srv->setCallback(boost::bind(&Image2RTSPNodelet::reconfigure, this,
                                         boost::placeholders::_1, boost::placeholders::_2));
    }
}

/* Publish one diagnostic_msgs/DiagnosticStatus per topic stream on /diagnostics
 * (1 Hz). Level is OK when idle or actively streaming, WARN when a client is
 * connected but no frames are arriving (typically no publisher on the source). */
void Image2RTSPNodelet::publishDiagnostics(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();

    std::lock_guard<std::mutex> lk(mtx);
    for (std::map<std::string, std::string>::const_iterator it = topic_source.begin();
         it != topic_source.end(); ++it) {
        const std::string& mount  = it->first;
        const std::string& source = it->second;
        int clients = num_of_clients.count(mount) ? num_of_clients[mount] : 0;
        bool recv   = receiving.count(mount) ? receiving[mount] : false;
        int pubs = 0;
        std::map<std::string, ros::Subscriber>::const_iterator s = subs.find(mount);
        if (s != subs.end()) pubs = s->second.getNumPublishers();

        diagnostic_msgs::DiagnosticStatus st;
        st.name = "ros_rtsp: " + mount;
        st.hardware_id = source;
        if (clients <= 0) {
            st.level = diagnostic_msgs::DiagnosticStatus::OK;
            st.message = "idle (no clients connected)";
        } else if (recv) {
            st.level = diagnostic_msgs::DiagnosticStatus::OK;
            st.message = "streaming to " + std::to_string(clients) + " client(s)";
        } else if (pubs == 0) {
            st.level = diagnostic_msgs::DiagnosticStatus::WARN;
            st.message = std::to_string(clients) + " client(s) connected, no publisher on '" + source + "'";
        } else {
            st.level = diagnostic_msgs::DiagnosticStatus::WARN;
            st.message = std::to_string(clients) + " client(s) connected, waiting for frames";
        }

        auto kv = [&st](const std::string& k, const std::string& v) {
            diagnostic_msgs::KeyValue p; p.key = k; p.value = v; st.values.push_back(p);
        };
        kv("source_topic", source);
        kv("clients", std::to_string(clients));
        kv("publishers", std::to_string(pubs));
        kv("receiving", recv ? "true" : "false");
        kv("codec", stream_codec.count(mount) ? stream_codec[mount] : "h264");
        kv("framerate", std::to_string(framerate.count(mount) ? framerate[mount] : 10));
        kv("mountpoint", mount);
        arr.status.push_back(st);
    }
    diag_pub.publish(arr);
}

/* Periodic check: for every topic stream that currently has a client, warn (throttled)
 * if nothing is publishing to its source topic. Keeps unreachable topics visible
 * instead of the client just hanging until the RTSP media-prepare timeout. */
void Image2RTSPNodelet::checkTopics(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mtx);
    for (std::map<std::string, int>::const_iterator it = num_of_clients.begin();
         it != num_of_clients.end(); ++it) {
        if (it->second <= 0) continue;                 // nobody watching -> not subscribed
        std::map<std::string, ros::Subscriber>::const_iterator s = subs.find(it->first);
        if (s == subs.end()) continue;
        if (s->second.getNumPublishers() == 0)
            NODELET_WARN_THROTTLE(10.0,
                "Stream %s: no publisher on source topic '%s' - connected clients will get no "
                "video until it starts publishing.", it->first.c_str(), topic_source[it->first].c_str());
    }
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
    std::string raw = stream.hasMember("mountpoint") ? pu::to_string(stream["mountpoint"]) : name;
    return pu::normalize_mountpoint(raw);
}

/* Build the encoder+caps fragment for a stream. Selected with the optional
 * per-stream parameters:
 *   encoder: x264 (default) | nvenc      - software / NVIDIA hardware
 *   codec:   h264 (default) | h265        - H.264 or H.265/HEVC
 *   encoder_override: "..."               - full custom encoder+caps, used verbatim
 * The actual pipeline strings live in the unit-tested params::encoder_fragment. */
std::string Image2RTSPNodelet::build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate) {
    if (stream.hasMember("encoder_override"))
        return pu::to_string(stream["encoder_override"]);

    std::string encoder = pu::member_string(stream, "encoder", "x264");
    std::string codec   = pu::member_string(stream, "codec", "h264");

    std::string e = encoder;
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c){ return std::tolower(c); });
    bool known = (e=="x264" || e=="x264enc" || e=="sw" ||
                  e=="nvenc" || e=="nv" || e=="nvh264enc" || e=="nvh265enc");
    if (!known)
        NODELET_WARN("Unknown encoder '%s' - falling back to software. Use 'encoder_override' "
                     "for a fully custom pipeline.", encoder.c_str());

    return pu::encoder_fragment(encoder, codec, bitrate);
}

/* Modified from https://github.com/ProjectArtemis/gst_video_server/blob/master/src/server_nodelet.cpp */
GstCaps* Image2RTSPNodelet::gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg, int fps)
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
            "framerate", GST_TYPE_FRACTION, fps, 1,
            nullptr);
}


void Image2RTSPNodelet::imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic) {
    std::lock_guard<std::mutex> lk(mtx);   // serialise with url_(dis)connected clearing appsrc

    // Only push once a client is connected and the media pipeline's appsrc exists.
    if (appsrc[topic] == NULL)
        return;

    int fps = framerate.count(topic) ? framerate[topic] : 10;
    GstCaps *caps = gst_caps_new_from_image(msg, fps);
    if (caps == NULL)   // unsupported encoding / big-endian (already logged, throttled)
        return;

    if (!receiving[topic]) {   // first frame since (re)subscribe - confirm it's flowing
        receiving[topic] = true;
        NODELET_INFO("Stream %s: receiving frames from '%s' (%dx%d %s)",
                     topic.c_str(), topic_source[topic].c_str(),
                     msg->width, msg->height, msg->encoding.c_str());
    }

    gst_app_src_set_caps(appsrc[topic], caps);
    gst_caps_unref(caps);   // set_caps takes its own ref; release ours (was leaked before)

    /* Zero-copy: wrap the ROS image data instead of memcpy'ing it, and keep the
     * message alive (a heap-held ConstPtr) until GStreamer is done with the buffer. */
    GstBuffer *buf = gst_buffer_new_wrapped_full(
        (GstMemoryFlags)0,
        (gpointer)msg->data.data(), msg->data.size(),   // data + maxsize
        0, msg->data.size(),                            // offset + size
        new sensor_msgs::Image::ConstPtr(msg),          // user_data: keeps msg alive
        [](gpointer p){ delete static_cast<sensor_msgs::Image::ConstPtr*>(p); });
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

            std::string type       = pu::member_string(stream, "type");
            std::string mountpoint = stream_mountpoint(stream, it->first);
            std::string source     = pu::member_string(stream, "source");

            if (type == "topic" && url == mountpoint && !source.empty()) {
                std::lock_guard<std::mutex> lk(mtx);
                if (num_of_clients[url] == 0) {
                    NODELET_INFO("Subscribing to '%s' for stream %s", source.c_str(), url.c_str());
                    receiving[url] = false;
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
                std::lock_guard<std::mutex> lk(mtx);
                if (num_of_clients[url] > 0) num_of_clients[url]--;
                if (num_of_clients[url] == 0) {
                    // No-one else is connected. Stop the subscription. The appsrc
                    // itself is released by on_media_unprepared() when the shared
                    // media is torn down, so a reconnecting client still finds it.
                    subs[url].shutdown();
                    receiving[url] = false;
                }
            }
        }
        catch (const XmlRpc::XmlRpcException& e) {
            NODELET_WARN("url_disconnected: skipping malformed stream '%s': %s",
                         it->first.c_str(), e.getMessage().c_str());
        }
    }
}

/* Called (on the GStreamer thread) when a mount's shared media is torn down.
 * This is the catch-all that also handles clients which vanish without a clean
 * RTSP TEARDOWN: release the appsrc ref taken in media_configure, drop the
 * subscription, and reset the client count so the topic isn't consumed forever. */
void Image2RTSPNodelet::on_media_unprepared(const std::string& mount) {
    std::lock_guard<std::mutex> lk(mtx);
    std::map<std::string, GstAppSrc*>::iterator a = appsrc.find(mount);
    if (a != appsrc.end() && a->second != NULL) {
        gst_object_unref(a->second);   // release the ref from gst_bin_get_by_name()
        a->second = NULL;
    }
    std::map<std::string, GstElement*>::iterator e = encoders.find(mount);
    if (e != encoders.end() && e->second != NULL) {
        gst_object_unref(e->second);   // release the encoder ref taken in register_encoder
        e->second = NULL;
    }
    num_of_clients[mount] = 0;
    receiving[mount] = false;
    std::map<std::string, ros::Subscriber>::iterator s = subs.find(mount);
    if (s != subs.end()) s->second.shutdown();
    NODELET_INFO("Stream %s: RTSP media released; source subscription stopped.", mount.c_str());
}

/* Called (on the GStreamer thread) from media_configure with the freshly built
 * encoder element for this mount. We keep the ref so dynamic_reconfigure can
 * change its bitrate live, and apply any override that's already in effect so a
 * newly (re)connecting client immediately gets the current bitrate. */
void Image2RTSPNodelet::register_encoder(const std::string& mount, GstElement* enc) {
    std::lock_guard<std::mutex> lk(mtx);
    std::map<std::string, GstElement*>::iterator e = encoders.find(mount);
    if (e != encoders.end() && e->second != NULL)
        gst_object_unref(e->second);   // drop a stale handle if media rebuilt without unprepare
    encoders[mount] = enc;             // takes ownership of the gst_bin_get_by_name ref
    if (enc != NULL && live_bitrate > 0) {
        g_object_set(G_OBJECT(enc), "bitrate", (guint)live_bitrate, NULL);
        NODELET_INFO("Stream %s: applied live bitrate %d kbit/s to new media.",
                     mount.c_str(), live_bitrate);
    }
}

/* dynamic_reconfigure callback (also fired once at startup with the defaults).
 * bitrate == 0 means "leave each stream at its configured bitrate"; any positive
 * value is pushed to every encoder that currently has prepared media. */
void Image2RTSPNodelet::reconfigure(ros_rtsp::BitrateConfig& config, uint32_t level) {
    std::lock_guard<std::mutex> lk(mtx);
    live_bitrate = config.bitrate;
    if (live_bitrate <= 0) {
        NODELET_INFO("Live bitrate override cleared - streams keep their configured bitrate.");
        return;
    }
    int applied = 0;
    for (std::map<std::string, GstElement*>::iterator it = encoders.begin();
         it != encoders.end(); ++it) {
        if (it->second != NULL) {
            g_object_set(G_OBJECT(it->second), "bitrate", (guint)live_bitrate, NULL);
            applied++;
        }
    }
    NODELET_INFO("Live bitrate set to %d kbit/s (applied to %d active stream(s)).",
                 live_bitrate, applied);
}

void Image2RTSPNodelet::print_info(char *s) {
    NODELET_INFO("%s",s);
}

void Image2RTSPNodelet::print_error(char *s) {
    NODELET_ERROR("%s",s);
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
