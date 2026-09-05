// stpbt_vhci_bridge.c
// Bridge MTK consys BT chardev (/dev/stpbt, H4 byte stream from the
// factory bt_drv_6895.ko) to the kernel virtual HCI controller
// (/dev/vhci, CONFIG_BT_HCIVHCI). Gives BlueZ a real hci0 with no
// kernel driver work. Run as root; Android Bluetooth must be OFF.
//
// Build: aarch64-linux-gnu-gcc -static -O2 -o stpbt_bridge stpbt_vhci_bridge.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#define STPBT_DEFAULT "/dev/stpbt"
#define VHCI_DEFAULT  "/dev/vhci"
#define LOGFILE       "/data/local/tmp/stpbt_bridge.log"
#define MAX_FRAME     4096

static int stp_fd = -1, vh_fd = -1;
static FILE *logf;
static volatile sig_atomic_t stop_flag;

static void logmsg(const char *fmt, ...)
{
	va_list ap;
	time_t t = time(NULL);
	struct tm tm;
	char ts[32];

	localtime_r(&t, &tm);
	strftime(ts, sizeof(ts), "%m-%d %H:%M:%S", &tm);

	va_start(ap, fmt);
	fprintf(stderr, "%s ", ts);
	vfprintf(stderr, fmt, ap);
	fprintf(stderr, "\n");
	if (logf) {
		fprintf(logf, "%s ", ts);
		va_start(ap, fmt);
		vfprintf(logf, fmt, ap);
		va_end(ap);
		fprintf(logf, "\n");
		fflush(logf);
	}
	va_end(ap);
}

static void on_signal(int sig)
{
	(void)sig;
	stop_flag = 1;
}

/* Total H4 frame length with type byte at b[0], or 0 if more bytes needed. */
static size_t h4_frame_len(const unsigned char *b, size_t n)
{
	size_t t, h, plen;

	if (n < 1)
		return 0;
	t = b[0];
	switch (t) {
	case 0x01: h = 4; if (n < h) return 0; plen = b[3]; break;      /* cmd  */
	case 0x02: h = 5; if (n < h) return 0; plen = b[3] | (b[4] << 8); break; /* acl */
	case 0x03: h = 4; if (n < h) return 0; plen = b[3]; break;      /* sco  */
	case 0x04: h = 3; if (n < h) return 0; plen = b[2]; break;      /* evt  */
	case 0x05: h = 5; if (n < h) return 0; plen = b[3] | (b[4] << 8); break; /* iso */
	default:
		return (size_t)-1;                                      /* bad type */
	}
	return h + plen;
}

static unsigned char acc[MAX_FRAME * 2];
static size_t acc_len;

/* Pump accumulated complete frames to vhci. Returns -1 on bad type. */
static int flush_frames(void)
{
	for (;;) {
		size_t len = h4_frame_len(acc, acc_len);
		ssize_t w;

		if (len == 0)
			return 0;
		if (len == (size_t)-1) {
			logmsg("H4 resync, dropping %zu bytes: %02x %02x %02x %02x",
			       acc_len, acc[0], acc[1],
			       acc_len > 2 ? acc[2] : 0,
			       acc_len > 3 ? acc[3] : 0);
			acc_len = 0;
			return 0;
		}
		if (acc_len < len)
			return 0;
		w = write(vh_fd, acc, len);
		if (w < 0) {
			logmsg("vhci write: %s", strerror(errno));
			return -1;
		}
		acc_len -= len;
		memmove(acc, acc + len, acc_len);
	}
}

int main(int argc, char **argv)
{
	const char *stp_path = argc > 1 ? argv[1] : STPBT_DEFAULT;
	const char *vh_path = argc > 2 ? argv[2] : VHCI_DEFAULT;
	struct pollfd pf[2];
	unsigned char buf[MAX_FRAME];
	static const unsigned char cfg[] = { 0xff, 0x00 };
	int n;

	logf = fopen(LOGFILE, "a");
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	logmsg("bridge starting: %s <-> %s", stp_path, vh_path);

	vh_fd = open(vh_path, O_RDWR);
	if (vh_fd < 0) {
		logmsg("open %s: %s", vh_path, strerror(errno));
		return 1;
	}
	if (write(vh_fd, cfg, sizeof(cfg)) != (ssize_t)sizeof(cfg)) {
		logmsg("vhci config write failed: %s", strerror(errno));
		return 1;
	}
	logmsg("vhci primary controller requested (hci0 created)");

	stp_fd = open(stp_path, O_RDWR | O_NOCTTY);
	if (stp_fd < 0) {
		logmsg("open %s: %s (is Android Bluetooth off?)",
		       stp_path, strerror(errno));
		return 1;
	}
	logmsg("stpbt opened - controller powered");

	pf[0].fd = stp_fd;
	pf[0].events = POLLIN;
	pf[1].fd = vh_fd;
	pf[1].events = POLLIN;

	while (!stop_flag) {
		n = poll(pf, 2, -1);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			logmsg("poll: %s", strerror(errno));
			break;
		}
		if (pf[0].revents & (POLLIN | POLLHUP)) {
			n = read(stp_fd, buf, sizeof(buf));
			if (n <= 0) {
				if (n < 0 && errno != EINTR && errno != EAGAIN)
					logmsg("stpbt read: %s", strerror(errno));
				continue;
			}
			if (acc_len + (size_t)n > sizeof(acc)) {
				logmsg("acc overflow, resync");
				acc_len = 0;
			}
			memcpy(acc + acc_len, buf, n);
			acc_len += n;
			if (flush_frames() < 0)
				break;
		}
		if (pf[1].revents & (POLLIN | POLLHUP)) {
			n = read(vh_fd, buf, sizeof(buf));
			if (n <= 0)
				continue;
			if (write(stp_fd, buf, n) < 0)
				logmsg("stpbt write: %s", strerror(errno));
		}
	}

	logmsg("bridge exiting");
	if (stp_fd >= 0)
		close(stp_fd);
	if (vh_fd >= 0)
		close(vh_fd);
	if (logf)
		fclose(logf);
	return 0;
}
