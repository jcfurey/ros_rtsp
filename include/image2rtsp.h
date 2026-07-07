#ifndef IMAGE_TO_RTSP_H
#define IMAGE_TO_RTSP_H

#include <map>
#include <string>
#include <mutex>
#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <ros_rtsp/BitrateConfig.h>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server.h>

namespace image2rtsp {

    /* Everything the nodelet tracks about one RTSP mount, kept in a single
     * struct so the lifecycle paths (media ready / media unprepared / client
     * count) can't update part of the state and miss the rest. All fields are
     * guarded by Image2RTSPNodelet::mtx. */
    struct StreamState {
        bool topic_stream = false;    // fed from a ROS Image topic via appsrc
        std::string source;           // ROS topic ('topic') / informational ('cam')
        std::string codec = "h264";   // resolved codec label (diagnostics)
        int fps = 10;                 // framerate advertised on the appsrc caps
        int config_bitrate = 500;     // kbit/s from the YAML - restored on reconfigure(0)
        ros::Subscriber sub;          // active only while the media is prepared
        GstAppSrc *appsrc = NULL;     // owned ref, set while the media is prepared
        GstElement *encoder = NULL;   // owned ref, set while the media is prepared
        GstRTSPMedia *current_media = NULL;  // identity only - never dereferenced
        int clients = 0;              // PLAY minus TEARDOWN (display/diagnostics)
        bool receiving = false;       // first frame logged since media prepared
    };

    class Image2RTSPNodelet : public nodelet::Nodelet {
        public:
            GstRTSPServer *rtsp_server;
            void onInit();
            void url_connected(std::string url);      // client sent PLAY for a mount
            void url_disconnected(std::string url);   // client sent TEARDOWN for a mount
            /* Called from media-configure (GStreamer thread) with the new media's
             * elements. Takes ownership of the appsrc/encoder refs, publishes them
             * under the mutex, and starts the topic subscription. */
            void on_media_ready(const std::string& mount, GstRTSPMedia *media,
                                GstElement *appsrc_elem, GstElement *encoder_elem);
            /* Called from the media "unprepared" signal. Ignores stale signals from
             * an older media of the same mount (identity-checked against the media
             * registered by on_media_ready). */
            void on_media_unprepared(const std::string& mount, GstRTSPMedia *media);
            void print_info(char *s);
            void print_error(char *s);

        private:
            std::string port;
            // Optional RTSP auth / TLS, read from params in onInit (empty => disabled).
            std::string auth_user;       // basic-auth username
            std::string auth_pass;       // basic-auth password
            std::string tls_cert_path;   // PEM (cert+key) path; set => serve rtsps://
            bool auth_enabled = false;   // true when auth_user is non-empty
            bool require_factory_role = false;  // auth/TLS active: gate mounts by role
            std::map<std::string, StreamState> stream_state;   // key: mountpoint
            int live_bitrate = 0;        // dynamic_reconfigure override (0 = per-stream config)
            std::mutex mtx;              // guards stream_state + live_bitrate (GLib vs ROS threads)
            ros::Timer watchdog;         // warns about unreachable source topics
            ros::Timer diag_timer;       // publishes /diagnostics at 1 Hz
            ros::Publisher diag_pub;     // diagnostic_msgs/DiagnosticArray
            std::unique_ptr<dynamic_reconfigure::Server<ros_rtsp::BitrateConfig>> dyn_srv;
            void checkTopics(const ros::TimerEvent&);
            void publishDiagnostics(const ros::TimerEvent&);
            void reconfigure(ros_rtsp::BitrateConfig& config, uint32_t level);
            std::string stream_mountpoint(XmlRpc::XmlRpcValue& stream, const std::string& name);
            std::string resolve_codec(XmlRpc::XmlRpcValue& stream, const std::string& name);
            std::string build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate,
                                      const std::string& codec);
            GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg, int fps);
            void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic);
            void video_mainloop_start();
            void rtsp_server_add_url(const char *url, const char *sPipeline, bool topic_stream);
            GstRTSPServer *rtsp_server_create(const std::string& port);
    };
}

#endif
