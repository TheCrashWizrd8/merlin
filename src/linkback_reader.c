/*
 * linkback_reader — stream a LinkBack "lem01camera" (USB 3456:4321) to stdout.
 *
 *   linkback_reader [usb-port-path]        e.g. 3-1.2   (default: first camera)
 *
 * Why this is C: the camera sends ~25,000 short USB transfers per second. The
 * Python USB library spends ~25 us on each, so one camera cost most of a CPU
 * core and a busy model starved the reader completely (no frames). Here the
 * whole USB/packet path is compiled code in its own process.
 *
 * Output on stdout: messages of { u32 magic "LBK1", u32 length } followed by
 * `length` bytes. length 0 = "streaming started"; length FRAME_BYTES = one
 * complete YUY2 frame (450x450). Errors go to stderr; exit status: 0 stopped
 * by signal, 2 could not open/start, 3 camera lost.
 *
 * Stream format (see src/linkback_camera.py): every 512-byte bulk packet starts
 * with a UVC-style header (byte0 = header length, byte1 flags: bit0 frame id,
 * bit1 end of frame, bit6 error, bit7 end of header), then YUY2 payload.
 *
 * Build: gcc -O2 -Wall -pthread -o linkback_reader linkback_reader.c $(pkg-config --cflags --libs libusb-1.0)
 */
#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <unistd.h>

#define VID 0x3456
#define PID 0x4321
#define FRAME_BYTES (450 * 450 * 2)
#define PACKET 512
#define EP_IN 0x82
#define NQ_DEFAULT 768   /* queued transfers: covers ~15-20 ms of host scheduling delay */
#define XFER_DEFAULT 4096 /* bytes per transfer; a short packet ends a transfer early anyway */
#define MAGIC 0x314B424CU /* "LBK1" little-endian */

static volatile sig_atomic_t g_stop = 0;
static int g_dead = 0;
static int g_inflight = 0;

/* frame assembly (USB event thread only) */
static uint8_t asm_buf[FRAME_BYTES + PACKET];
static size_t asm_fill = 0;
static int asm_fid = -1;
static int asm_bad = 0;

/* latest finished frame, handed to the writer thread */
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static uint8_t out_buf[FRAME_BYTES];
static int out_ready = 0;
static int out_quit = 0;

static int write_all(const void *buf, size_t n)
{
    const uint8_t *p = buf;
    while (n > 0) {
        ssize_t w = write(STDOUT_FILENO, p, n);
        if (w < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        p += w;
        n -= (size_t)w;
    }
    return 0;
}

static int send_msg(const void *data, uint32_t len)
{
    uint32_t hdr[2] = {MAGIC, len};
    if (write_all(hdr, sizeof hdr) < 0)
        return -1;
    return len ? write_all(data, len) : 0;
}

static void *writer_main(void *arg)
{
    static uint8_t mine[FRAME_BYTES];
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&mu);
        while (!out_ready && !out_quit)
            pthread_cond_wait(&cv, &mu);
        if (out_quit && !out_ready) {
            pthread_mutex_unlock(&mu);
            return NULL;
        }
        memcpy(mine, out_buf, FRAME_BYTES);
        out_ready = 0;
        pthread_mutex_unlock(&mu);
        if (send_msg(mine, FRAME_BYTES) < 0) {
            g_stop = 1; /* consumer gone */
            return NULL;
        }
    }
}

static void finish_frame(void)
{
    if (asm_fill == FRAME_BYTES && !asm_bad) {
        pthread_mutex_lock(&mu);
        memcpy(out_buf, asm_buf, FRAME_BYTES); /* an unsent older frame is replaced */
        out_ready = 1;
        pthread_cond_signal(&cv);
        pthread_mutex_unlock(&mu);
    }
    asm_fill = 0;
    asm_bad = 0;
}

static void handle_data(const uint8_t *data, size_t len)
{
    for (size_t off = 0; off < len; off += PACKET) {
        size_t n = len - off < PACKET ? len - off : PACKET;
        const uint8_t *p = data + off;
        if (n < 2)
            continue;
        unsigned hlen = p[0], flags = p[1];
        if (hlen < 2 || hlen > n || !(flags & 0x80)) {
            asm_bad = 1;
            continue;
        }
        int fid = flags & 1;
        if (asm_fid >= 0 && fid != asm_fid)
            finish_frame();
        asm_fid = fid;
        if (flags & 0x40)
            asm_bad = 1;
        size_t payload = n - hlen;
        if (asm_fill + payload <= FRAME_BYTES) {
            memcpy(asm_buf + asm_fill, p + hlen, payload);
            asm_fill += payload;
        } else {
            asm_bad = 1;
            asm_fill = FRAME_BYTES + 1; /* too long: never a valid frame */
        }
        if (flags & 0x02)
            finish_frame();
    }
}

static void LIBUSB_CALL on_transfer(struct libusb_transfer *t)
{
    g_inflight--;
    if (t->status == LIBUSB_TRANSFER_COMPLETED) {
        if (t->actual_length > 0)
            handle_data(t->buffer, (size_t)t->actual_length);
    } else if (t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_TIMED_OUT) {
        g_dead = 1;
    }
    if (!g_stop && !g_dead) {
        if (libusb_submit_transfer(t) == 0)
            g_inflight++;
        else
            g_dead = 1;
    }
}

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int path_of(libusb_device *dev, char *out, size_t cap)
{
    uint8_t ports[8];
    int n = libusb_get_port_numbers(dev, ports, sizeof ports);
    if (n <= 0)
        return -1;
    int w = snprintf(out, cap, "%d-", libusb_get_bus_number(dev));
    for (int i = 0; i < n; i++)
        w += snprintf(out + w, cap - (size_t)w, i ? ".%d" : "%d", ports[i]);
    return 0;
}

int main(int argc, char **argv)
{
    const char *want = argc > 1 ? argv[1] : "";
    prctl(PR_SET_PDEATHSIG, SIGTERM); /* die with the parent */
    signal(SIGPIPE, SIG_IGN);
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    libusb_context *ctx = NULL;
    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "libusb_init failed\n");
        return 2;
    }

    libusb_device **list = NULL;
    ssize_t count = libusb_get_device_list(ctx, &list);
    libusb_device_handle *h = NULL;
    char found[64] = "";
    int open_err = 0;
    for (ssize_t i = 0; i < count && !h; i++) {
        struct libusb_device_descriptor d;
        char path[64];
        if (libusb_get_device_descriptor(list[i], &d) != 0 || d.idVendor != VID || d.idProduct != PID)
            continue;
        if (path_of(list[i], path, sizeof path) != 0)
            continue;
        if (found[0] == 0)
            snprintf(found, sizeof found, "%s", path);
        if (want[0] && strcmp(want, path) != 0)
            continue;
        open_err = libusb_open(list[i], &h);
        if (open_err != 0)
            h = NULL;
    }
    libusb_free_device_list(list, 1);
    if (!h) {
        if (open_err == LIBUSB_ERROR_ACCESS)
            fprintf(stderr, "permission denied opening LinkBack camera (install config/udev/99-linkback-camera.rules)\n");
        else if (open_err != 0)
            fprintf(stderr, "cannot open LinkBack camera: %s\n", libusb_error_name(open_err));
        else if (found[0] == 0)
            fprintf(stderr, "no LinkBack camera found (USB 3456:4321)\n");
        else
            fprintf(stderr, "no LinkBack camera on port %s (found %s)\n", want, found);
        libusb_exit(ctx);
        return 2;
    }

    int claimed0 = 0, claimed1 = 0, rc;
    if ((rc = libusb_claim_interface(h, 0)) != 0 || (claimed0 = 1, (rc = libusb_claim_interface(h, 1)) != 0) ||
        (claimed1 = 1, (rc = libusb_set_interface_alt_setting(h, 1, 1)) != 0)) {
        fprintf(stderr, "cannot start LinkBack camera: %s\n", libusb_error_name(rc));
        if (claimed1)
            libusb_release_interface(h, 1);
        if (claimed0)
            libusb_release_interface(h, 0);
        libusb_close(h);
        libusb_exit(ctx);
        return 2;
    }

    int nq = getenv("LINKBACK_NQ") ? atoi(getenv("LINKBACK_NQ")) : NQ_DEFAULT;
    int xfer = getenv("LINKBACK_XFER") ? atoi(getenv("LINKBACK_XFER")) : XFER_DEFAULT;
    if (nq < 2 || nq > 4096)
        nq = NQ_DEFAULT;
    if (xfer < PACKET || xfer % PACKET)
        xfer = XFER_DEFAULT;
    struct libusb_transfer **xfers = calloc((size_t)nq, sizeof *xfers);
    for (int i = 0; i < nq; i++) {
        xfers[i] = libusb_alloc_transfer(0);
        uint8_t *buf = malloc((size_t)xfer);
        libusb_fill_bulk_transfer(xfers[i], h, EP_IN, buf, xfer, on_transfer, NULL, 0);
        if (libusb_submit_transfer(xfers[i]) == 0)
            g_inflight++;
    }

    pthread_t writer;
    if (send_msg(NULL, 0) < 0 || pthread_create(&writer, NULL, writer_main, NULL) != 0)
        g_stop = 1;

    while (!g_stop && !g_dead) {
        struct timeval tv = {0, 100000};
        libusb_handle_events_timeout_completed(ctx, &tv, NULL);
    }

    for (int i = 0; i < nq; i++)
        libusb_cancel_transfer(xfers[i]);
    for (int spins = 0; g_inflight > 0 && spins < 40; spins++) {
        struct timeval tv = {0, 50000};
        libusb_handle_events_timeout_completed(ctx, &tv, NULL);
    }

    pthread_mutex_lock(&mu);
    out_quit = 1;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mu);
    pthread_join(writer, NULL);

    libusb_set_interface_alt_setting(h, 1, 0);
    libusb_release_interface(h, 1);
    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(ctx);
    return g_dead ? 3 : 0;
}
