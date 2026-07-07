#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include <image2rtsp.h>

using namespace std;
using namespace image2rtsp;


/* Per-mount context handed to the media-configure / unprepared callbacks so they
 * can find the owning nodelet, the mount path, and where to stash the appsrc. */
struct MediaCtx {
    Image2RTSPNodelet *nodelet;
    std::string mount;
    GstElement **appsrc;   // -> nodelet's appsrc[mount] map entry
};

static void media_ctx_free(gpointer p) { delete static_cast<MediaCtx*>(p); }


static void *mainloop(void *arg) {
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    g_main_loop_run(loop);

    g_main_loop_unref(loop);
    return NULL;
}


void Image2RTSPNodelet::video_mainloop_start() {
    pthread_t tloop;

    gst_init(NULL, NULL);
    pthread_create(&tloop, NULL, &mainloop, NULL);
}


static void client_options(GstRTSPClient *client, GstRTSPContext *state, Image2RTSPNodelet *nodelet) {
    if (state->uri) {
        nodelet->url_connected(state->uri->abspath);
    }
}


static void client_teardown(GstRTSPClient *client, GstRTSPContext *state, Image2RTSPNodelet *nodelet) {
    if (state->uri) {
        nodelet->url_disconnected(state->uri->abspath);
    }
}


static void new_client(GstRTSPServer *server, GstRTSPClient *client, Image2RTSPNodelet *nodelet) {
    nodelet->print_info((char *)"New RTSP client");
    g_signal_connect(client, "options-request", G_CALLBACK(client_options), nodelet);
    g_signal_connect(client, "teardown-request", G_CALLBACK(client_teardown), nodelet);
}

/* this function is periodically run to clean up the expired sessions from the pool. */
static gboolean session_cleanup(Image2RTSPNodelet *nodelet, gboolean ignored)
{
    GstRTSPServer *server = nodelet->rtsp_server;
    GstRTSPSessionPool *pool;
    int num;

    pool = gst_rtsp_server_get_session_pool(server);
    num = gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    if (num > 0) {
        char s[32];
        snprintf(s, 32, (char *)"Sessions cleaned: %d", num);
        nodelet->print_info(s);
    }

    return TRUE;
}

GstRTSPServer *Image2RTSPNodelet::rtsp_server_create(const std::string& port) {
    GstRTSPServer *server;

    /* create a server instance */
    server = gst_rtsp_server_new();

  // char *port = (char *) port;
  g_object_set(server, "service", port.c_str(), NULL);

    /* attach the server to the default maincontext */
    if (gst_rtsp_server_attach(server, NULL) == 0) {
        NODELET_FATAL("Failed to start the RTSP server on port %s - is the port already in "
                      "use (another ros_rtsp / RTSP server still running)? No streams will "
                      "be reachable.", port.c_str());
    }

    g_signal_connect(server, "client-connected", G_CALLBACK(new_client), this);

    /* add a timeout for the session cleanup */
    g_timeout_add_seconds(2, (GSourceFunc)session_cleanup, this);

    return server;
}


/* fired when a media (RTSP pipeline instance) is torn down. Safety net that also
 * covers clients which vanish without a clean TEARDOWN: release the appsrc and
 * stop the ROS subscription so we don't keep consuming the topic or hold a stale
 * appsrc pointer. */
static void media_unprepared(GstRTSPMedia *media, MediaCtx *ctx)
{
    if (ctx && ctx->nodelet)
        ctx->nodelet->on_media_unprepared(ctx->mount);
}

/* called when a new media pipeline is constructed. We can query the
 * pipeline and configure our appsrc */
static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, MediaCtx *ctx)
{    if (ctx) {
        GstElement *pipeline = gst_rtsp_media_get_element(media);

        *ctx->appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");

        /* this instructs appsrc that we will be dealing with timed buffer */
        gst_util_set_object_arg(G_OBJECT(*ctx->appsrc), "format", "time");

        gst_object_unref(pipeline);

        /* clean up when this media goes away (covers clients that just vanish) */
        g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared), ctx);
    }
    else
    {
        guint i, n_streams;
        n_streams = gst_rtsp_media_n_streams (media);

        for (i = 0; i < n_streams; i++) {
            GstRTSPAddressPool *pool;
            GstRTSPStream *stream;
            gchar *min, *max;

            stream = gst_rtsp_media_get_stream (media, i);

            /* make a new address pool */
            pool = gst_rtsp_address_pool_new ();

            min = g_strdup_printf ("224.3.0.%d", (2 * i) + 1);
            max = g_strdup_printf ("224.3.0.%d", (2 * i) + 2);
            gst_rtsp_address_pool_add_range (pool, min, max,
                5000 + (10 * i), 5010 + (10 * i), 1);
            g_free (min);
            g_free (max);

            gst_rtsp_stream_set_address_pool (stream, pool);
            g_object_unref (pool);
          }
    }
}


void Image2RTSPNodelet::rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc) {
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    /* get the mount points for this server, every server has a default object
    * that be used to map uri mount points to media factories */
    mounts = gst_rtsp_server_get_mount_points(rtsp_server);

    /* make a media factory for a test stream. The default media factory can use
     * gst-launch syntax to create pipelines.
     * any launch line works as long as it contains elements named pay%d. Each
     * element with pay%d names will be a stream */
    factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, sPipeline);

    /* For topic streams (appsrc != NULL) build a context so media-configure can
     * wire the appsrc and the unprepared cleanup. cam streams pass NULL and take
     * the multicast-address-pool branch. The context is owned by the factory. */
    MediaCtx *ctx = NULL;
    if (appsrc != NULL) {
        ctx = new MediaCtx();
        ctx->nodelet = this;
        ctx->mount   = url;
        ctx->appsrc  = appsrc;
        g_object_set_data_full(G_OBJECT(factory), "ros_rtsp_ctx", ctx, media_ctx_free);
    }

    /* notify when our media is ready, This is called whenever someone asks for
     * the media and a new pipeline is created */
    g_signal_connect(factory, "media-configure", (GCallback)media_configure, ctx);

    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* attach the factory to the url */
    gst_rtsp_mount_points_add_factory(mounts, url, factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);
}
