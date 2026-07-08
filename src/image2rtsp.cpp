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
    // A present-but-unusable security block FAILS CLOSED: better no server at all
    // than an operator who believes auth/TLS is on while the server is wide open.
    if (nh.hasParam("auth")) {
        XmlRpc::XmlRpcValue a;
        nh.getParam("auth", a);
        if (a.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            auth_user = pu::member_string(a, "user");
            auth_pass = pu::member_string(a, "pass");
        }
        if (auth_user.empty()) {
            NODELET_FATAL("'auth' is configured but has no usable 'user' key (expected "
                          "auth: {user: <name>, pass: <secret>}). Refusing to start an "
                          "unauthenticated server that was configured to require auth.");
            return;
        }
        if (auth_pass.empty())
            NODELET_WARN("'auth' has an empty 'pass' - clients authenticate with an "
                         "empty password.");
    }
    if (nh.hasParam("tls")) {
        XmlRpc::XmlRpcValue t;
        nh.getParam("tls", t);
        if (t.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            tls_cert_path = pu::member_string(t, "cert");
        if (tls_cert_path.empty()) {
            NODELET_FATAL("'tls' is configured but has no 'cert' key (expected "
                          "tls: {cert: /path/to/cert.pem}). Refusing to start an "
                          "unencrypted server that was configured for TLS.");
            return;
        }
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
    if (rtsp_server == NULL)
        return;   // rtsp_server_create already logged the fatal reason (port / TLS cert)

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
            std::string mountpoint = stream_mountpoint(stream, name);
            int bitrate_kbps       = pu::member_int(stream, "bitrate", 500);
            std::string bitrate    = std::to_string(bitrate_kbps);
            std::string codec      = resolve_codec(stream, name);
            std::string encoder    = build_encoder(stream, bitrate, codec);
            std::string tail       = pu::payloader_tail(codec);
            std::string source     = pu::member_string(stream, "source");
            std::string pipeline;

            if (source.empty()) {
                NODELET_ERROR("Stream '%s' has no 'source' - skipping.", name.c_str());
                continue;
            }

            bool is_topic;
            if (type == "cam") {
                is_topic = false;
                pipeline = "( " + source + " ! " + encoder + tail;
            }
            else if (type == "topic") {
                is_topic = true;
                if (!topic_is_advertised(source))
                    NODELET_WARN("Stream '%s': source topic '%s' is not currently advertised. "
                                 "The stream is registered anyway; clients will receive video once "
                                 "something publishes to it (check the topic name/namespace if not).",
                                 name.c_str(), source.c_str());

                /* Optional 'caps': rescale and/or adjust the framerate before encoding.
                 * videorate + videoscale sit in front of the caps filter so a caps
                 * framerate different from the stream's 'framerate' (the rate the
                 * appsrc advertises) converts cleanly instead of failing negotiation.
                 * When omitted, the topic is served at its native resolution. */
                std::string scale = "";
                if (stream.hasMember("caps"))
                    scale = "videorate ! videoscale ! " + pu::to_string(stream["caps"]) + " ! ";

                pipeline = "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 "
                           "max-bytes=1000 is-live=true ! videoconvert ! " + scale + encoder + tail;
            }
            else {
                NODELET_ERROR("Stream '%s' has unknown type '%s' (expected 'topic' or 'cam') - skipping.",
                              name.c_str(), type.c_str());
                continue;
            }

            {
                std::lock_guard<std::mutex> lk(mtx);
                StreamState& st  = stream_state[mountpoint];
                st.topic_stream   = is_topic;
                st.source         = source;
                st.codec          = codec;
                st.config_bitrate = bitrate_kbps;
                st.fps            = pu::member_int(stream, "framerate", 10);
            }
            rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), is_topic);

            gchar *addr = gst_rtsp_server_get_address(rtsp_server);
            NODELET_INFO("Stream '%s' available at rtsp%s://%s:%s%s",
                         name.c_str(), tls_cert_path.empty() ? "" : "s",
                         addr, this->port.c_str(), mountpoint.c_str());
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
    if (registered == 0) {
        NODELET_ERROR("No streams were registered. The RTSP server is up on port %s but has "
                      "no mount points, so clients will get 'no factory for path ...'.",
                      this->port.c_str());
        return;
    }

    // Watchdog: while media is live, periodically warn if a source topic has no
    // publisher. Diagnostics: per-stream status on /diagnostics (1 Hz). Live
    // bitrate: rqt_reconfigure knob covering every registered stream (topic + cam).
    watchdog   = nh.createTimer(ros::Duration(5.0), &Image2RTSPNodelet::checkTopics, this);
    diag_pub   = getNodeHandle().advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    diag_timer = nh.createTimer(ros::Duration(1.0), &Image2RTSPNodelet::publishDiagnostics, this);

    // The server fires the callback once with the default (bitrate=0 = per-stream
    // config), so constructing it never clobbers the YAML values.
    dyn_srv.reset(new dynamic_reconfigure::Server<ros_rtsp::BitrateConfig>(nh));
    dyn_srv->setCallback(boost::bind(&Image2RTSPNodelet::reconfigure, this,
                                     boost::placeholders::_1, boost::placeholders::_2));
}

/* Periodic check: for every topic stream whose media is currently prepared, warn
 * (throttled) if nothing is publishing to its source topic. Keeps unreachable
 * topics visible instead of the client just hanging until the RTSP timeout. */
void Image2RTSPNodelet::checkTopics(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lk(mtx);
    for (std::map<std::string, StreamState>::const_iterator it = stream_state.begin();
         it != stream_state.end(); ++it) {
        const StreamState& st = it->second;
        if (!st.topic_stream || st.current_media == NULL) continue;  // no media -> not subscribed
        if (st.sub && st.sub.getNumPublishers() == 0)
            NODELET_WARN_THROTTLE(10.0,
                "Stream %s: no publisher on source topic '%s' - connected clients will get no "
                "video until it starts publishing.", it->first.c_str(), st.source.c_str());
    }
}

/* Publish one diagnostic_msgs/DiagnosticStatus per stream on /diagnostics (1 Hz).
 * Level is OK when idle or actively streaming, WARN when media is prepared for a
 * topic stream but no frames are arriving (typically no publisher on the source). */
void Image2RTSPNodelet::publishDiagnostics(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();

    std::lock_guard<std::mutex> lk(mtx);
    for (std::map<std::string, StreamState>::const_iterator it = stream_state.begin();
         it != stream_state.end(); ++it) {
        const std::string& mount = it->first;
        const StreamState& st    = it->second;
        bool active = (st.current_media != NULL);
        int pubs = (st.topic_stream && st.sub) ? (int)st.sub.getNumPublishers() : 0;

        diagnostic_msgs::DiagnosticStatus s;
        s.name = "ros_rtsp: " + mount;
        s.hardware_id = st.topic_stream ? st.source : "gstreamer";
        if (!active) {
            s.level = diagnostic_msgs::DiagnosticStatus::OK;
            s.message = "idle (no clients connected)";
        } else if (!st.topic_stream) {
            s.level = diagnostic_msgs::DiagnosticStatus::OK;
            s.message = "streaming to " + std::to_string(st.clients) + " client(s)";
        } else if (st.receiving) {
            s.level = diagnostic_msgs::DiagnosticStatus::OK;
            s.message = "streaming to " + std::to_string(st.clients) + " client(s)";
        } else if (pubs == 0) {
            s.level = diagnostic_msgs::DiagnosticStatus::WARN;
            s.message = "media prepared, no publisher on '" + st.source + "'";
        } else {
            s.level = diagnostic_msgs::DiagnosticStatus::WARN;
            s.message = "media prepared, waiting for frames";
        }

        auto kv = [&s](const std::string& k, const std::string& v) {
            diagnostic_msgs::KeyValue p; p.key = k; p.value = v; s.values.push_back(p);
        };
        kv("type", st.topic_stream ? "topic" : "cam");
        kv("source", st.source);
        kv("clients", std::to_string(st.clients));
        if (st.topic_stream) {
            kv("publishers", std::to_string(pubs));
            kv("receiving", st.receiving ? "true" : "false");
            kv("framerate", std::to_string(st.fps));
        }
        kv("codec", st.codec);
        kv("mountpoint", mount);
        arr.status.push_back(s);
    }
    diag_pub.publish(arr);
}

/* RTSP mount point for a stream. Defaults to "/<stream name>" when the stream
 * omits the 'mountpoint' parameter. gst-rtsp-server requires the path to start
 * with '/' (a path that doesn't is silently rejected and the mount never
 * registers), so normalise: strip whitespace, backslashes -> slashes, and
 * guarantee a single leading '/'. */
std::string Image2RTSPNodelet::stream_mountpoint(XmlRpc::XmlRpcValue& stream, const std::string& name) {
    std::string raw = stream.hasMember("mountpoint") ? pu::to_string(stream["mountpoint"]) : name;
    return pu::normalize_mountpoint(raw);
}

/* Codec for a stream. Explicit 'codec' wins; otherwise a codec-specific encoder
 * keyword (nvh265enc, x265enc, ...) implies its codec, so asking for an H.265
 * element without also writing 'codec: h265' does what the user meant instead
 * of silently producing H.264. An explicit mismatch gets a warning. */
std::string Image2RTSPNodelet::resolve_codec(XmlRpc::XmlRpcValue& stream, const std::string& name) {
    std::string encoder = pu::member_string(stream, "encoder", "");
    std::string implied = pu::encoder_implied_codec(encoder);

    if (stream.hasMember("codec")) {
        std::string codec = pu::to_string(stream["codec"]);
        if (!implied.empty() && pu::is_h265_codec(codec) != pu::is_h265_codec(implied))
            NODELET_WARN("Stream '%s': encoder '%s' implies codec '%s' but 'codec: %s' is set - "
                         "using '%s'. Drop one of the two to silence this.",
                         name.c_str(), encoder.c_str(), implied.c_str(), codec.c_str(), codec.c_str());
        return codec;
    }
    return implied.empty() ? "h264" : implied;
}

/* Build the encoder+caps fragment for a stream. Selected with the optional
 * per-stream parameters:
 *   encoder: x264 (default) | x265 | nvenc     - software / NVIDIA hardware
 *   codec:   h264 (default) | h265             - resolved by resolve_codec()
 *   encoder_override: "..."                    - full custom fragment, used verbatim
 * The actual pipeline strings live in the unit-tested params::encoder_fragment. */
std::string Image2RTSPNodelet::build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate,
                                             const std::string& codec) {
    if (stream.hasMember("encoder_override"))
        return pu::to_string(stream["encoder_override"]);

    std::string encoder = pu::member_string(stream, "encoder", "x264");
    if (!pu::is_known_encoder(encoder))
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
    std::lock_guard<std::mutex> lk(mtx);   // serialise with on_media_ready/unprepared

    std::map<std::string, StreamState>::iterator it = stream_state.find(topic);
    if (it == stream_state.end() || it->second.appsrc == NULL)
        return;   // no media prepared for this mount right now
    StreamState& st = it->second;

    GstCaps *caps = gst_caps_new_from_image(msg, st.fps);
    if (caps == NULL)   // unsupported encoding / big-endian (already logged, throttled)
        return;

    if (!st.receiving) {   // first frame since media prepared - confirm it's flowing
        st.receiving = true;
        NODELET_INFO("Stream %s: receiving frames from '%s' (%dx%d %s)",
                     topic.c_str(), st.source.c_str(),
                     msg->width, msg->height, msg->encoding.c_str());
    }

    gst_app_src_set_caps(st.appsrc, caps);   // short-circuits internally on identical caps
    gst_caps_unref(caps);

    /* Zero-copy: wrap the ROS image data instead of memcpy'ing it, and keep the
     * message alive (a heap-held ConstPtr) until GStreamer is done with the buffer.
     * READONLY is essential: the message is shared with every other in-process
     * subscriber, so a downstream in-place transform must copy-on-write, never
     * scribble into msg->data. */
    GstBuffer *buf = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        (gpointer)msg->data.data(), msg->data.size(),   // data + maxsize
        0, msg->data.size(),                            // offset + size
        new sensor_msgs::Image::ConstPtr(msg),          // user_data: keeps msg alive
        [](gpointer p){ delete static_cast<sensor_msgs::Image::ConstPtr*>(p); });
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

    gst_app_src_push_buffer(st.appsrc, buf);   // takes ownership of buf
}


/* Client sent PLAY for a mount: bump the display count. The topic subscription
 * is NOT tied to this - it follows the media lifecycle (on_media_ready /
 * on_media_unprepared), which is auth-gated and covers vanished clients. */
void Image2RTSPNodelet::url_connected(string url) {
    std::lock_guard<std::mutex> lk(mtx);
    std::map<std::string, StreamState>::iterator it = stream_state.find(url);
    if (it == stream_state.end()) return;
    it->second.clients++;
    NODELET_INFO("Client playing %s (%d client(s))", url.c_str(), it->second.clients);
}

/* Client sent TEARDOWN for a mount: drop the display count. Clients that vanish
 * without TEARDOWN are reconciled when their media unprepares. */
void Image2RTSPNodelet::url_disconnected(string url) {
    std::lock_guard<std::mutex> lk(mtx);
    std::map<std::string, StreamState>::iterator it = stream_state.find(url);
    if (it == stream_state.end()) return;
    if (it->second.clients > 0) it->second.clients--;
    NODELET_INFO("Client disconnected: %s (%d client(s) left)", url.c_str(), it->second.clients);
}

/* A new media for this mount was constructed (client passed the auth check at
 * DESCRIBE). Takes ownership of the element refs. Runs on a GStreamer thread. */
void Image2RTSPNodelet::on_media_ready(const std::string& mount, GstRTSPMedia *media,
                                       GstElement *appsrc_elem, GstElement *encoder_elem) {
    std::lock_guard<std::mutex> lk(mtx);
    std::map<std::string, StreamState>::iterator it = stream_state.find(mount);
    if (it == stream_state.end()) {   // unknown mount: just drop the refs
        if (appsrc_elem)  gst_object_unref(appsrc_elem);
        if (encoder_elem) gst_object_unref(encoder_elem);
        return;
    }
    StreamState& st = it->second;

    // A rebuilt media can arrive before the old one's "unprepared" fires; release
    // the stale handles so the refs from this media are the only ones we hold.
    if (st.appsrc)  { gst_object_unref(st.appsrc);  st.appsrc  = NULL; }
    if (st.encoder) { gst_object_unref(st.encoder); st.encoder = NULL; }

    st.appsrc        = (GstAppSrc *)appsrc_elem;   // NULL for cam streams
    st.encoder       = encoder_elem;               // NULL with encoder_override
    st.current_media = media;
    st.receiving     = false;

    // Safe under mtx: media-configure fires before the media prepares, so the
    // encoder isn't running yet and this set can't block on encoder drain
    // (unlike reconfigure(), which handles live encoders - see there).
    if (st.encoder && live_bitrate > 0) {
        g_object_set(G_OBJECT(st.encoder), "bitrate", (guint)live_bitrate, NULL);
        NODELET_INFO("Stream %s: applied live bitrate %d kbit/s to new media.",
                     mount.c_str(), live_bitrate);
    }

    if (st.topic_stream && !st.sub) {
        NODELET_INFO("Subscribing to '%s' for stream %s", st.source.c_str(), mount.c_str());
        ros::NodeHandle& nh = getPrivateNodeHandle();
        st.sub = nh.subscribe<sensor_msgs::Image>(
            st.source, 1,
            boost::bind(&Image2RTSPNodelet::imageCallback, this, boost::placeholders::_1, mount));
    }
}

/* The media for a mount was torn down (clean TEARDOWN or vanished client).
 * Identity-checked: a stale signal from an older media of the same mount must
 * not clobber the state a newer media has already registered. The subscriber is
 * shut down OUTSIDE the mutex - roscpp's shutdown() blocks until an executing
 * imageCallback returns, and imageCallback takes the mutex first, so shutting
 * down under the lock would deadlock the GStreamer thread against the spinner. */
void Image2RTSPNodelet::on_media_unprepared(const std::string& mount, GstRTSPMedia *media) {
    ros::Subscriber doomed;
    {
        std::lock_guard<std::mutex> lk(mtx);
        std::map<std::string, StreamState>::iterator it = stream_state.find(mount);
        if (it == stream_state.end()) return;
        StreamState& st = it->second;
        if (st.current_media != media)
            return;   // stale signal from an already-replaced media - newer state wins

        if (st.appsrc)  { gst_object_unref(st.appsrc);  st.appsrc  = NULL; }
        if (st.encoder) { gst_object_unref(st.encoder); st.encoder = NULL; }
        st.current_media = NULL;
        st.clients   = 0;
        st.receiving = false;
        doomed  = st.sub;
        st.sub  = ros::Subscriber();
    }
    doomed.shutdown();   // outside mtx (see comment above)
    NODELET_INFO("Stream %s: RTSP media released; source subscription stopped.", mount.c_str());
}

/* Executed by GStreamer's helper thread (gst_element_call_async below). */
static void apply_bitrate_async(GstElement *encoder, gpointer kbps) {
    g_object_set(G_OBJECT(encoder), "bitrate", (guint)GPOINTER_TO_UINT(kbps), NULL);
}

/* dynamic_reconfigure callback (also fired once at startup with the defaults).
 * bitrate > 0 overrides every live encoder; bitrate == 0 restores each stream's
 * configured (YAML) bitrate on its live encoder.
 *
 * The property sets are dispatched with gst_element_call_async, NEVER done
 * synchronously here: setting "bitrate" on a live x265enc blocks until the
 * encoder drains, and a pipeline whose TCP client just vanished doesn't drain
 * until the media is reaped. This callback runs on the nodelet's single-threaded
 * callback queue - the same queue as imageCallback and the diagnostics/watchdog
 * timers - so blocking here froze the entire ROS side of the node (observed
 * live, gdb-confirmed). call_async runs the set on a GStreamer helper thread
 * (which holds its own ref on the element) and returns immediately. */
void Image2RTSPNodelet::reconfigure(ros_rtsp::BitrateConfig& config, uint32_t level) {
    int scheduled = 0;
    {
        std::lock_guard<std::mutex> lk(mtx);
        bool clearing = (config.bitrate <= 0) && (live_bitrate > 0);
        live_bitrate = config.bitrate > 0 ? config.bitrate : 0;

        if (config.bitrate <= 0 && !clearing)
            return;   // startup default / repeated 0 - nothing to change

        for (std::map<std::string, StreamState>::iterator it = stream_state.begin();
             it != stream_state.end(); ++it) {
            StreamState& st = it->second;
            if (st.encoder == NULL) continue;
            guint target = (guint)(live_bitrate > 0 ? live_bitrate : st.config_bitrate);
            gst_element_call_async(st.encoder, apply_bitrate_async,
                                   GUINT_TO_POINTER(target), NULL);
            scheduled++;
        }
    }

    if (live_bitrate > 0)
        NODELET_INFO("Live bitrate set to %d kbit/s (applied to %d active stream(s)).",
                     live_bitrate, scheduled);
    else
        NODELET_INFO("Live bitrate override cleared - restored configured bitrate on %d "
                     "active stream(s).", scheduled);
}

void Image2RTSPNodelet::print_info(char *s) {
    NODELET_INFO("%s",s);
}

void Image2RTSPNodelet::print_error(char *s) {
    NODELET_ERROR("%s",s);
}

PLUGINLIB_EXPORT_CLASS(image2rtsp::Image2RTSPNodelet, nodelet::Nodelet)
