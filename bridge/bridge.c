// stpbt_vhci_bridge.c  v2
// Bridge MTK consys BT chardev (/dev/stpbt, H4 byte stream from the
// factory bt_drv_6895.ko) to the kernel virtual HCI controller
// (/dev/vhci, CONFIG_BT_HCIVHCI=y). Gives BlueZ a real hci0 with no
// kernel driver work. Run as root; Android Bluetooth must be OFF.
//
// v2: the MTK firmware asserts if the host stays silent for ~3s after
// power-on, and signals failures via whole-chip reset + HW Error event
// on the read path. Handle both: keep-alive HCI_Reset right after
// open, transparent close/reopen of /dev/stpbt on reset, buffering
// stack traffic while the session is down.
//
// Build: aarch64-linux-gnu-gcc -static -O2 -o stpbt_bridge stpbt_vhci_bridge.c
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
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
#define ACC_SIZE      16384
#define PENDING_SIZE  65536

static const unsigned char HCI_RESET[] = { 0x01, 0x03, 0x0c, 0x00 };
static const unsigned char HW_ERROR[]  = { 0x04, 0x10, 0x01 };
static const unsigned char CC_RESET[]  = { 0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00 };

static int stp_fd = -1, vh_fd = -1;
static FILE *logf;
static volatile sig_atomic_t stop_flag;
static unsigned char acc[ACC_SIZE];         /* stpbt -> vhci H4 accumulator */
static size_t acc_len;
static unsigned char pending[PENDING_SIZE]; /* vhci -> stpbt while down */
static size_t pend_len;
static int session_up;

static void logmsg(const char *fmt, ...)
{
	va_list ap;
	time_t t = time(NULL);
	struct tm tm;
	char ts[32];

	localtime_r(&t, &tm);
	strftime(ts, sizeof(ts), "%m-%d %H:%M:%S", &tm);

	fprintf(stderr, "%s ", ts);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");
	if (logf) {
		fprintf(logf, "%s ", ts);
		va_start(ap, fmt);
		vfprintf(logf, fmt, ap);
		va_end(ap);
		fprintf(logf, "\n");
		fflush(logf);
	}
}

static void on_signal(int sig)
{
	(void)sig;
	stop_flag = 1;
}

/* Total H4 frame length with type byte at b[0], or 0 if more needed. */
static size_t h4_frame_len(const unsigned char *b, size_t n)
{
	size_t t, h, plen;

	if (n < 1)
		return 0;
	t = b[0];
	switch (t) {
	case 0x01: h = 4; if (n < h) return 0; plen = b[3]; break;
	case 0x02: h = 5; if (n < h) return 0; plen = b[3] | (b[4] << 8); break;
	case 0x03: h = 4; if (n < h) return 0; plen = b[3]; break;
	case 0x04: h = 3; if (n < h) return 0; plen = b[2]; break;
	case 0x05: h = 5; if (n < h) return 0; plen = b[3] | (b[4] << 8); break;
	default:
		return (size_t)-1;
	}
	return h + plen;
}

static void stp_close(void)
{
	if (stp_fd >= 0) {
		close(stp_fd);
		stp_fd = -1;
	}
	session_up = 0;
}

/* Send HCI_Reset and consume everything until its Command Complete. */
static void keepalive_reset(void)
{
	unsigned char buf[512];
	struct pollfd pf;
	size_t got = 0;
	int matched = 0, n, i;

	if (write(stp_fd, HCI_RESET, sizeof(HCI_RESET)) !=
	    (ssize_t)sizeof(HCI_RESET)) {
		logmsg("keepalive write failed: %s", strerror(errno));
		return;
	}
	pf.fd = stp_fd;
	pf.events = POLLIN;
	for (i = 0; i < 20 && !matched; i++) {
		n = poll(&pf, 1, 500);
		if (n <= 0)
			continue;
		n = read(stp_fd, buf + got, sizeof(buf) - got);
		if (n <= 0)
			continue;
		got += n;
		if (got >= sizeof(CC_RESET) &&
		    !memcmp(buf + got - sizeof(CC_RESET), CC_RESET,
			    sizeof(CC_RESET)))
			matched = 1;
	}
	logmsg(matched ? "keepalive Reset answered (controller alive)"
		       : "keepalive Reset: no Command Complete");
}

static int stp_open_with_keepalive(void)
{
	stp_fd = open(STPBT_DEFAULT, O_RDWR | O_NOCTTY);
	if (stp_fd < 0) {
		logmsg("open %s: %s", STPBT_DEFAULT, strerror(errno));
		return -1;
	}
	keepalive_reset();
	session_up = 1;
	return 0;
}

/* Consume complete frames from acc; forward to vhci; catch HW error.
 * Returns 1 when a reopen is requested, -1 on fatal error. */
static int process_acc(void)
{
	for (;;) {
		size_t len = h4_frame_len(acc, acc_len);
		ssize_t w;

		if (len == 0)
			return 0;
		if (len == (size_t)-1) {
			logmsg("H4 resync, dropping %zu bytes: %02x %02x",
			       acc_len,
			       acc_len > 0 ? acc[0] : 0,
			       acc_len > 1 ? acc[1] : 0);
			acc_len = 0;
			return 0;
		}
		if (acc_len < len)
			return 0;

		/* Mask the controller's bogus Synchronization Train feature
		 * (page 2 byte 0 bit 2): it claims support but returns
		 * "Unknown HCI Command" for Read Sync Train Params, which
		 * aborts the kernel's init sequence. */
		if (len >= 16 && acc[1] == 0x0e && acc[4] == 0x04 &&
		    acc[5] == 0x10 && acc[6] == 0x00 && acc[7] == 0x02 &&
		    (acc[8] & 0x04)) {
			logmsg("masking bogus Synchronization Train feature");
			acc[8] &= (unsigned char)~0x04;
		}

		if (len >= sizeof(HW_ERROR) &&
		    !memcmp(acc, HW_ERROR, sizeof(HW_ERROR))) {
			logmsg("HW Error event from controller - reset cycle");
			w = write(vh_fd, acc, len);
			(void)w;
			acc_len -= len;
			memmove(acc, acc + len, acc_len);
			stp_close();               /* force fresh session */
			return 1;
		}

		w = write(vh_fd, acc, len);
		if (w < 0) {
			logmsg("vhci write: %s", strerror(errno));
			return -1;
		}
		acc_len -= len;
		memmove(acc, acc + len, acc_len);
	}
}

static void pending_put(const unsigned char *data, size_t n)
{
	if (pend_len + n > sizeof(pending)) {
		logmsg("pending overflow, dropping %zu bytes", n);
		return;
	}
	memcpy(pending + pend_len, data, n);
	pend_len += n;
}

static int pending_flush(void)
{
	ssize_t w = write(stp_fd, pending, pend_len);

	if (w < 0) {
		logmsg("pending flush: %s", strerror(errno));
		return -1;
	}
	logmsg("pending flushed: %zd/%zu bytes", w, pend_len);
	if ((size_t)w < pend_len) {
		memmove(pending, pending + w, pend_len - w);
		pend_len -= w;
		return -1;
	}
	pend_len = 0;
	return 0;
}

int main(int argc, char **argv)
{
	const char *stp_path = argc > 1 ? argv[1] : STPBT_DEFAULT;
	const char *vh_path = argc > 2 ? argv[2] : VHCI_DEFAULT;
	struct pollfd pf[2];
	unsigned char buf[MAX_FRAME];
	static const unsigned char cfg[] = { 0xff, 0x00 };
	int nf, n, reopen_tries = 0;

	logf = fopen(LOGFILE, "a");
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	logmsg("bridge v3 starting: %s <-> %s", stp_path, vh_path);

	vh_fd = open(vh_path, O_RDWR);
	if (vh_fd < 0) {
		logmsg("open %s: %s", vh_path, strerror(errno));
		return 1;
	}

	if (stp_open_with_keepalive() < 0) {
		logmsg("cannot bring up stpbt session");
		return 1;
	}

	if (write(vh_fd, cfg, sizeof(cfg)) != (ssize_t)sizeof(cfg)) {
		logmsg("vhci config write failed: %s", strerror(errno));
		return 1;
	}
	logmsg("vhci primary controller requested (hci0 created)");

	while (!stop_flag) {
		nf = 1;
		pf[0].fd = vh_fd;
		pf[0].events = POLLIN;
		if (stp_fd >= 0) {
			pf[1].fd = stp_fd;
			pf[1].events = POLLIN;
			nf = 2;
		}

		n = poll(pf, nf, 500);
		if (n < 0 && errno != EINTR) {
			logmsg("poll: %s", strerror(errno));
			break;
		}

		/* vhci -> stpbt (stack commands) */
		if (pf[0].revents & (POLLIN | POLLHUP)) {
			n = read(vh_fd, buf, sizeof(buf));
			if (n > 0) {
				if (stp_fd >= 0 && session_up) {
					if (write(stp_fd, buf, n) < 0) {
						logmsg("stpbt write: %s - reopening session",
						       strerror(errno));
						pending_put((unsigned char *)buf, n);
						stp_close();
					}
				} else {
					pending_put((unsigned char *)buf, n);
				}
			}
		}

		/* stpbt -> vhci (controller events) */
		if (stp_fd >= 0 && nf == 2 &&
		    (pf[1].revents & (POLLIN | POLLHUP))) {
			n = read(stp_fd, buf, sizeof(buf));
			if (n > 0) {
				int rc;
				if (acc_len + (size_t)n > sizeof(acc)) {
					logmsg("acc overflow, resync");
					acc_len = 0;
				}
				memcpy(acc + acc_len, buf, n);
				acc_len += n;
				rc = process_acc();
				if (rc < 0)
					break;
				if (rc == 1)
					stp_close();
			}
		}

		/* session down? reopen with backoff, flush pending traffic */
		if (stp_fd < 0 && !stop_flag) {
			if (reopen_tries < 10) {
				reopen_tries++;
				logmsg("reopening stpbt session (try %d), pending %zu bytes",
				       reopen_tries, pend_len);
				if (stp_open_with_keepalive() == 0) {
					if (pend_len)
						pending_flush();
					reopen_tries = 0;
					logmsg("session restored");
				} else {
					sleep(2);
				}
			} else {
				sleep(2);
			}
		}
	}

	logmsg("bridge exiting");
	stp_close();
	close(vh_fd);
	if (logf)
		fclose(logf);
	return 0;
}
