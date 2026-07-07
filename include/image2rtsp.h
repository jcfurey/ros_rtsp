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
    class Image2RTSPNodelet : public nodelet::Nodelet {
        public:
            GstRTSPServer *rtsp_server;
            void onInit();
            void url_connected(std::string url);
            void url_disconnected(std::string url);
            void on_media_unprepared(const std::string& mount);
            void register_encoder(const std::string& mount, GstElement* enc);
            void print_info(char *s);
            void print_error(char *s);

        private:
            std::string port;
            // Optional RTSP auth / TLS, read from params in onInit (empty => disabled).
            std::string auth_user;       // basic-auth username
            std::string auth_pass;       // basic-auth password
            std::string tls_cert_path;   // PEM (cert+key) path; set => serve rtsps://
            bool auth_enabled = false;   // true when auth_user is non-empty
            bool require_factory_role = false;  // set once auth/TLS is active: gate mounts by role
            std::map<std::string, ros::Subscriber> subs;
            std::map<std::string, GstAppSrc*> appsrc;
            std::map<std::string, int> num_of_clients;
            std::map<std::string, std::string> topic_source;   // mount -> source topic (topic streams)
            std::map<std::string, bool> receiving;             // mount -> have we logged first frame
            std::map<std::string, int> framerate;              // mount -> advertised fps for appsrc caps
            std::map<std::string, std::string> stream_codec;   // mount -> codec label (for diagnostics)
            std::map<std::string, GstElement*> encoders;       // mount -> live encoder element (bitrate-tunable)
            int live_bitrate = 0;                              // dynamic_reconfigure override (0 = keep config)
            std::mutex mtx;                                    // guards the maps above (GLib vs ROS threads)
            ros::Timer watchdog;                               // warns about unreachable source topics
            ros::Timer diag_timer;                             // publishes /diagnostics at 1 Hz
            ros::Publisher diag_pub;                           // diagnostic_msgs/DiagnosticArray
            std::unique_ptr<dynamic_reconfigure::Server<ros_rtsp::BitrateConfig>> dyn_srv;
            void checkTopics(const ros::TimerEvent&);
            void publishDiagnostics(const ros::TimerEvent&);
            void reconfigure(ros_rtsp::BitrateConfig& config, uint32_t level);
            std::string stream_mountpoint(XmlRpc::XmlRpcValue& stream, const std::string& name);
            std::string build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate);
            GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg, int fps);
            void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic);
            void video_mainloop_start();
            void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
            GstRTSPServer *rtsp_server_create(const std::string& port);
    };
}

#endif
