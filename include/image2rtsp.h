#ifndef IMAGE_TO_RTSP_H
#define IMAGE_TO_RTSP_H

#include <map>
#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

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
            void print_info(char *s);
            void print_error(char *s);

        private:
            std::string port;
            std::map<std::string, ros::Subscriber> subs;
            std::map<std::string, GstAppSrc*> appsrc;
            std::map<std::string, int> num_of_clients;
            std::string build_encoder(XmlRpc::XmlRpcValue& stream, const std::string& bitrate);
            GstCaps* gst_caps_new_from_image(const sensor_msgs::Image::ConstPtr &msg);
            void imageCallback(const sensor_msgs::Image::ConstPtr& msg, const std::string& topic);
            void video_mainloop_start();
            void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
            GstRTSPServer *rtsp_server_create(const std::string& port);
    };
}

#endif
