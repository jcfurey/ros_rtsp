#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <string>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtsp-server/rtsp-auth.h>
#include <gst/rtsp-server/rtsp-token.h>
#include <gst/rtsp-server/rtsp-permissions.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "sensor_msgs/Image.h"
#include <image2rtsp.h>
#include <image2rtsp/param_utils.h>

using namespace std;
using namespace image2rtsp;

/* Role tying a basic-auth login to the per-factory access/construct permissions.
 * Shared by rtsp_server_create (mints the token) and rtsp_server_add_url (grants
 * the role on each mount) so authenticated clients can reach the streams. */
static const char *AUTH_ROLE = "ros-rtsp-user";


/* Per-mount context handed to the media-configure / unprepared callbacks. Owned
 * by the factory (g_object_set_data_full), shared by every media the factory
 * constructs - so it carries only immutable identity, never per-media state. */
struct MediaCtx {
    Image2RTSPNodelet *nodelet;
    std::string mount;
    bool topic_stream;
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


/* Client-count bookkeeping. PLAY (not OPTIONS) marks a watching client: OPTIONS
 * is not gated by authentication and is sent by probes/keep-alives, so counting
 * it would let an unauthenticated probe distort the count. The ROS subscription
 * does not depend on these counts - it follows the media lifecycle below. */
static void client_play(GstRTSPClient *client, GstRTSPContext *state, Image2RTSPNodelet *nodelet) {
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
    g_signal_connect(client, "play-request", G_CALLBACK(client_play), nodelet);
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
    g_object_set(server, "service", port.c_str(), NULL);

    /* Optional basic auth and/or TLS. Once a GstRTSPAuth object is installed the
     * server denies any client whose token lacks a role permitted on the factory,
     * so both paths grant AUTH_ROLE on the mounts (see rtsp_server_add_url):
     *   - basic auth on  -> the login mints a token carrying AUTH_ROLE;
     *   - TLS only       -> anonymous clients get AUTH_ROLE via a default token,
     *                       so the stream stays open but the transport is encrypted.
     * Security config errors FAIL CLOSED: a server the operator configured as
     * encrypted/authenticated must not silently come up open. */
    GTlsCertificate *cert = NULL;
    if (!this->tls_cert_path.empty()) {
        GError *err = NULL;
        cert = g_tls_certificate_new_from_file(this->tls_cert_path.c_str(), &err);
        if (cert == NULL) {
            NODELET_FATAL("Failed to load TLS certificate '%s': %s. Refusing to start "
                          "without the encryption this server is configured for.",
                          this->tls_cert_path.c_str(), err ? err->message : "unknown error");
            if (err) g_error_free(err);
            g_object_unref(server);
            return NULL;
        }
    }

    if (this->auth_enabled || cert != NULL) {
        GstRTSPAuth *auth = gst_rtsp_auth_new();

        if (this->auth_enabled) {
            GstRTSPToken *token = gst_rtsp_token_new(
                GST_RTSP_TOKEN_MEDIA_FACTORY_ROLE, G_TYPE_STRING, AUTH_ROLE, NULL);
            gchar *basic = gst_rtsp_auth_make_basic(this->auth_user.c_str(),
                                                    this->auth_pass.c_str());
            gst_rtsp_auth_add_basic(auth, basic, token);
            g_free(basic);
            gst_rtsp_token_unref(token);
            NODELET_INFO("RTSP basic auth enabled (user '%s'): clients must supply credentials.",
                         this->auth_user.c_str());
        } else {
            /* TLS only, no login required: hand anonymous clients the role so they
             * can still reach the (now role-gated) factories over the encrypted link. */
            GstRTSPToken *def = gst_rtsp_token_new(
                GST_RTSP_TOKEN_MEDIA_FACTORY_ROLE, G_TYPE_STRING, AUTH_ROLE, NULL);
            gst_rtsp_auth_set_default_token(auth, def);
            gst_rtsp_token_unref(def);
        }

        if (cert != NULL) {
            gst_rtsp_auth_set_tls_certificate(auth, cert);
            NODELET_INFO("RTSP TLS enabled (connect via rtsps://) using certificate '%s'.",
                         this->tls_cert_path.c_str());
        }

        gst_rtsp_server_set_auth(server, auth);
        g_object_unref(auth);
        this->require_factory_role = true;   // mounts must grant AUTH_ROLE (add_url)
    }
    if (cert) g_object_unref(cert);

    /* attach the server to the default maincontext */
    if (gst_rtsp_server_attach(server, NULL) == 0) {
        NODELET_FATAL("Failed to start the RTSP server on port %s - is the port already in "
                      "use (another ros_rtsp / RTSP server still running)? No streams will "
                      "be served.", port.c_str());
        g_object_unref(server);
        return NULL;
    }

    g_signal_connect(server, "client-connected", G_CALLBACK(new_client), this);

    /* add a timeout for the session cleanup */
    g_timeout_add_seconds(2, (GSourceFunc)session_cleanup, this);

    return server;
}


/* Fired when a media (one RTSP pipeline instance) is torn down - including for
 * clients that vanish without a clean TEARDOWN. The nodelet identity-checks the
 * media pointer so a stale signal can't clobber a newer media's state. */
static void media_unprepared(GstRTSPMedia *media, MediaCtx *ctx)
{
    if (ctx && ctx->nodelet)
        ctx->nodelet->on_media_unprepared(ctx->mount, media);
}

/* Called when a new media pipeline is constructed (the requesting client has
 * already passed the auth check at DESCRIBE). Hand the pipeline's elements to
 * the nodelet - it stores them under its mutex and starts the ROS subscription
 * for topic streams - then set up the per-media teardown hook. */
static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, MediaCtx *ctx)
{
    if (!ctx || !ctx->nodelet) return;

    GstElement *pipeline = gst_rtsp_media_get_element(media);

    GstElement *appsrc = NULL;
    if (ctx->topic_stream) {
        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");
        /* timed buffers; set BEFORE the appsrc is published to imageCallback so
         * no frame can be pushed while the element is still in bytes format */
        if (appsrc)
            gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
    }
    /* The encoder is present in every built-in pipeline (topic and cam) and is
     * NULL with encoder_override unless the user names their element venc0. */
    GstElement *encoder = gst_bin_get_by_name(GST_BIN(pipeline), params::ENCODER_NAME);

    gst_object_unref(pipeline);

    /* transfers the appsrc/encoder refs to the nodelet (locked in there) */
    ctx->nodelet->on_media_ready(ctx->mount, media, appsrc, encoder);

    /* clean up when this media goes away (covers clients that just vanish) */
    g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared), ctx);

    if (!ctx->topic_stream) {
        /* cam streams: keep the original multicast address pool configuration */
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


void Image2RTSPNodelet::rtsp_server_add_url(const char *url, const char *sPipeline, bool topic_stream) {
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

    /* Context for media-configure/unprepared: which mount, which kind of stream.
     * Owned by the factory, shared by all of its medias. */
    MediaCtx *ctx = new MediaCtx();
    ctx->nodelet      = this;
    ctx->mount        = url;
    ctx->topic_stream = topic_stream;
    g_object_set_data_full(G_OBJECT(factory), "ros_rtsp_ctx", ctx, media_ctx_free);

    /* notify when our media is ready, This is called whenever someone asks for
     * the media and a new pipeline is created */
    g_signal_connect(factory, "media-configure", (GCallback)media_configure, ctx);

    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* When auth or TLS is active, gate this mount on AUTH_ROLE. Both the basic-auth
     * login token and the TLS-only default token carry that role, so permitted
     * clients pass and everyone else is refused ("not authorized to see factory"). */
    if (this->require_factory_role) {
        GstRTSPPermissions *permissions = gst_rtsp_permissions_new();
        gst_rtsp_permissions_add_role(permissions, AUTH_ROLE,
            GST_RTSP_PERM_MEDIA_FACTORY_ACCESS,    G_TYPE_BOOLEAN, TRUE,
            GST_RTSP_PERM_MEDIA_FACTORY_CONSTRUCT, G_TYPE_BOOLEAN, TRUE, NULL);
        gst_rtsp_media_factory_set_permissions(factory, permissions);
        gst_rtsp_permissions_unref(permissions);
    }

    /* attach the factory to the url */
    gst_rtsp_mount_points_add_factory(mounts, url, factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);
}
