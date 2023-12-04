#include <getopt.h>
#include <libwebsockets.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(WIN32) || defined(_WIN32)
#else
#include <unistd.h>
#endif

int close_testing;
int max_poll_elements;
int debug_level = 7;

volatile int force_exit = 0, dynamic_vhost_enable = 0;
struct lws_vhost *dynamic_vhost;
struct lws_context *context;
struct lws_plat_file_ops fops_plat;

char *resource_path = HTTP_LOCAL_RESOURCE_PATH;

#if defined(LWS_WITH_TLS) && defined(LWS_HAVE_SSL_CTX_set1_param)
char crl_path[1024] = "";
#endif

#define LWS_PLUGIN_STATIC
#include "plugins/protocol_tof.cpp"

static int lws_callback_http(struct lws *wsi, enum lws_callback_reasons reason,
                             void *user, void *in, size_t len) {
    const unsigned char *c;
    char buf[1024];
    int n = 0, hlen;

    switch (reason) {
    case LWS_CALLBACK_HTTP:

        do {
            c = lws_token_to_string((lws_token_indexes)n);
            if (!c) {
                n++;
                continue;
            }

            hlen = lws_hdr_total_length(wsi, (lws_token_indexes)n);
            if (!hlen || hlen > (int)sizeof(buf) - 1) {
                n++;
                continue;
            }

            if (lws_hdr_copy(wsi, buf, sizeof buf, (lws_token_indexes)n) < 0)
                fprintf(stderr, "    %s (too big)\n", (char *)c);
            else {
                buf[sizeof(buf) - 1] = '\0';

                fprintf(stderr, "    %s = %s\n", (char *)c, buf);
            }
            n++;
        } while (c);

        n = 0;
        while (lws_hdr_copy_fragment(wsi, buf, sizeof(buf),
                                     WSI_TOKEN_HTTP_URI_ARGS, n) > 0) {
            lwsl_notice("URI Arg %d: %s\n", ++n, buf);
        }

        if (lws_return_http_status(wsi, HTTP_STATUS_NOT_FOUND, NULL))
            return -1;

        if (lws_http_transaction_completed(wsi))
            return -1;

        return 0;
    default:
        break;
    }

    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

static struct lws_protocols protocols[] = {
    {
        "http-only",
        lws_callback_http,
        0,
        MAX_MESSAGE_LEN,
    },
    {"connection-check-protocol", callback_tof, sizeof(struct pss__tof),
     MAX_MESSAGE_LEN, 0, NULL, 0},
    {NULL, NULL, 0, 0}};

static lws_fop_fd_t test_server_fops_open(const struct lws_plat_file_ops *fops,
                                          const char *vfs_path,
                                          const char *vpath,
                                          lws_fop_flags_t *flags) {
    lws_fop_fd_t fop_fd;

    fop_fd = fops_plat.open(fops, vfs_path, vpath, flags);

    if (fop_fd)
        lwsl_info("%s: opening %s, ret %p, len %lu\n", __func__, vfs_path,
                  fop_fd, (long)lws_vfs_get_length(fop_fd));
    else
        lwsl_info("%s: open %s failed\n", __func__, vfs_path);

    return fop_fd;
}

void sighandler(int sig) {
#if !defined(WIN32) && !defined(_WIN32)
    if (sig == SIGUSR1) {

        dynamic_vhost_enable ^= 1;
        lws_cancel_service(context);
        lwsl_notice("SIGUSR1: dynamic_vhost_enable: %d\n",
                    dynamic_vhost_enable);
        return;
    }
#endif
    force_exit = 1;
    lws_cancel_service(context);
}

static const struct lws_extension exts[] = {{"permessage-deflate",
                                             lws_extension_callback_pm_deflate,
                                             "permessage-deflate"},
                                            {NULL, NULL, NULL}};

static const struct lws_http_mount mount_ziptest = {
    NULL, "/ziptest", HTTP_LOCAL_RESOURCE_PATH "/candide.zip",
    NULL, NULL,       NULL,
    NULL, NULL,       0,
    0,    0,          0,
    0,    0,          LWSMPRO_FILE,
    8,    NULL,       {NULL, NULL}};

static const struct lws_http_mount mount_post = {
    (struct lws_http_mount *)&mount_ziptest,
    "/formtest",
    "protocol-post-demo",
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    0,
    0,
    0,
    0,
    0,
    0,
    LWSMPRO_CALLBACK,
    9,
    NULL,
    {NULL, NULL}};

static const struct lws_http_mount mount = {
    (struct lws_http_mount *)&mount_post,
    "/",
    HTTP_LOCAL_RESOURCE_PATH,
    "test.html",
    NULL,
    NULL,
    NULL,
    NULL,
    0,
    0,
    0,
    0,
    0,
    0,
    LWSMPRO_FILE,
    1,
    NULL,
    {NULL, NULL}};

int main(int argc, char **argv) {
    struct lws_context_creation_info info;
    struct lws_vhost *vhost;
    const char *iface = NULL;
    int uid = -1, gid = -1;
    int use_ssl = 0;
    int pp_secs = 0;
    int opts = 0;
    int n = 0;

    memset(&info, 0, sizeof info);
    info.port = 7681;
    signal(SIGINT, sighandler);
#if !defined(WIN32) && !defined(_WIN32)
    signal(SIGUSR1, sighandler);
#endif
    lws_set_log_level(debug_level, NULL);

    printf("Using resource path \"%s\"\n", resource_path);
    info.iface = iface;
    info.protocols = protocols;
    info.ssl_cert_filepath = NULL;
    info.ssl_private_key_filepath = NULL;
    info.ws_ping_pong_interval = pp_secs;
    info.gid = gid;
    info.uid = uid;
    info.options = opts | LWS_SERVER_OPTION_VALIDATE_UTF8 |
                   LWS_SERVER_OPTION_EXPLICIT_VHOSTS;
    info.extensions = exts;
    info.timeout_secs = 5;
    info.mounts = &mount;
    info.ip_limit_ah = 24;
    info.ip_limit_wsi = 400;

    if (use_ssl)
        info.options |= LWS_SERVER_OPTION_REDIRECT_HTTP_TO_HTTPS;

    context = lws_create_context(&info);
    if (context == NULL) {
        lwsl_err("libwebsocket init failed\n");
        return -1;
    }

    vhost = lws_create_vhost(context, &info);
    if (!vhost) {
        lwsl_err("vhost creation failed\n");
        return -1;
    }

    info.port++;

#if !defined(LWS_NO_CLIENT) && defined(LWS_WITH_TLS)
    lws_init_vhost_client_ssl(&info, vhost);
#endif
    fops_plat = *(lws_get_fops(context));
    lws_get_fops(context)->open = test_server_fops_open;

    n = 0;
    while (n >= 0 && !force_exit) {
        struct timeval tv;

        gettimeofday(&tv, NULL);
        n = lws_service(context, 50);

        if (dynamic_vhost_enable && !dynamic_vhost) {
            lwsl_notice("creating dynamic vhost...\n");
            dynamic_vhost = lws_create_vhost(context, &info);
        } else if (!dynamic_vhost_enable && dynamic_vhost) {
            lwsl_notice("destroying dynamic vhost...\n");
            lws_vhost_destroy(dynamic_vhost);
            dynamic_vhost = NULL;
        }
    }
    lws_context_destroy(context);

    lwsl_notice("libwebsockets-http-server exited cleanly\n");

    return 0;
}
