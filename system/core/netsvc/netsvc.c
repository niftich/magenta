// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "netsvc.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include <inet6/inet6.h>
#include <inet6/netifc.h>

#include <launchpad/launchpad.h>
#include <magenta/process.h>
#include <magenta/processargs.h>
#include <magenta/syscalls.h>
#include <magenta/syscalls/log.h>
#include <mxio/io.h>

#include <magenta/boot/netboot.h>

#include "device_id.h"

#define FILTER_IPV6 1

#define MAX_LOG_LINE (MX_LOG_RECORD_MAX + 32)

static mx_handle_t loghandle;

int get_log_line(char* out) {
    char buf[MX_LOG_RECORD_MAX + 1];
    mx_log_record_t* rec = (mx_log_record_t*)buf;
    if (mx_log_read(loghandle, MX_LOG_RECORD_MAX, rec, 0) > 0) {
        if (rec->datalen && (rec->data[rec->datalen - 1] == '\n')) {
            rec->datalen--;
        }
        rec->data[rec->datalen] = 0;
        snprintf(out, MAX_LOG_LINE, "[%05d.%03d] %05" PRIu64 ".%05" PRIu64 "> %s\n",
                 (int)(rec->timestamp / 1000000000ULL),
                 (int)((rec->timestamp / 1000000ULL) % 1000ULL),
                 rec->pid, rec->tid, rec->data);
        return strlen(out);
    } else {
        return 0;
    }
}

static volatile uint32_t seqno = 1;
static volatile uint32_t pending = 0;

static void run_program(const char *progname, int argc, const char** argv, mx_handle_t h) {

    launchpad_t* lp;
    launchpad_create(0, progname, &lp);
    launchpad_clone(lp, LP_CLONE_ALL & (~LP_CLONE_MXIO_STDIO));
    launchpad_load_from_file(lp, argv[0]);
    launchpad_set_args(lp, argc, argv);
    mx_handle_t handle = MX_HANDLE_INVALID;
    mx_log_create(0, &handle);
    launchpad_add_handle(lp, handle, PA_HND(PA_MXIO_LOGGER, 0 | MXIO_FLAG_USE_FOR_STDIO));
    if (h != MX_HANDLE_INVALID) {
        launchpad_add_handle(lp, h, PA_HND(PA_USER0, 0));
    }
    mx_status_t status;
    const char* errmsg;
    if ((status = launchpad_go(lp, NULL, &errmsg)) < 0) {
        printf("netsvc: cannot launch %s: %d: %s\n", argv[0], status, errmsg);
    }
}

static void run_command(const char* cmd) {
    const char* args[] = {
        "/boot/bin/sh", "-c", cmd
    };
    printf("net cmd: %s\n", cmd);
    run_program("net:sh", 3, args, 0);
}

static void run_server(const char* progname, const char* bin, mx_handle_t h) {
    run_program(progname, 1, &bin, h);
}

static const char* nodename = "magenta";

void udp6_recv(void* data, size_t len,
               const ip6_addr_t* daddr, uint16_t dport,
               const ip6_addr_t* saddr, uint16_t sport) {

    bool mcast = (memcmp(daddr, &ip6_ll_all_nodes, sizeof(ip6_addr_t)) == 0);

    if (dport == NB_SERVER_PORT) {
        nbmsg* msg = data;
        if ((len < (sizeof(nbmsg) + 1)) ||
            (msg->magic != NB_MAGIC)) {
            return;
        }
        // null terminate the payload
        len -= sizeof(nbmsg);
        msg->data[len - 1] = 0;

        switch (msg->cmd) {
        case NB_QUERY:
            if (strcmp((char*)msg->data, "*") &&
                strcmp((char*)msg->data, nodename)) {
                break;
            }
            size_t dlen = strlen(nodename) + 1;
            char buf[1024 + sizeof(nbmsg)];
            if ((dlen + sizeof(nbmsg)) > sizeof(buf)) {
                return;
            }
            msg->cmd = NB_ACK;
            memcpy(buf, msg, sizeof(nbmsg));
            memcpy(buf + sizeof(nbmsg), nodename, dlen);
            udp6_send(buf, sizeof(nbmsg) + dlen, saddr, sport, dport);
            break;
        case NB_SHELL_CMD:
            if (!mcast) {
                run_command((char*) msg->data);
                return;
            }
            break;
        case NB_OPEN:
            netfile_open((char*)msg->data, msg->cookie, msg->arg, saddr, sport, dport);
            break;
        case NB_READ:
            netfile_read(msg->cookie, msg->arg, saddr, sport, dport);
            break;
        case NB_WRITE:
            len--; // NB NUL-terminator is not part of the data
            netfile_write((char*)msg->data, len, msg->cookie, msg->arg, saddr, sport, dport);
            break;
        case NB_CLOSE:
            netfile_close(msg->cookie, saddr, sport, dport);
            break;
        }
        return;
    }

    if (dport == DEBUGLOG_ACK_PORT) {
        if ((len != 8) || mcast) {
            return;
        }
        logpacket_t* pkt = data;
        if ((pkt->magic != 0xaeae1123) || (pkt->seqno != seqno)) {
            return;
        }
        if (pending) {
            seqno++;
            pending = 0;
            // ensure we stop polling
            netifc_set_timer(0);
        }
    }
}

void netifc_recv(void* data, size_t len) {
    eth_recv(data, len);
}

int main(int argc, char** argv) {
    logpacket_t pkt;
    int len = 0;
    unsigned char mac[6];
    uint16_t mtu;
    char device_id[DEVICE_ID_MAX];

    if (mx_log_create(MX_LOG_FLAG_READABLE, &loghandle) < 0) {
        return -1;
    }

    for (;;) {
        if (netifc_open() != 0) {
            printf("netsvc: fatal error initializing network\n");
            return -1;
        }

        // Use mac address to generate unique nodename unless one was provided.
        if ((argc > 1) && (argv[1][0])) {
            nodename = argv[1];
        } else {
            netifc_get_info(mac, &mtu);
            device_id_get(mac, device_id);
            nodename = device_id;
        }

        printf("netsvc: nodename='%s'\n", nodename);
        printf("netsvc: start\n");
        for (;;) {
            if (pending == 0) {
                pkt.magic = 0xaeae1123;
                pkt.seqno = seqno;
                strncpy(pkt.nodename, nodename, sizeof(pkt.nodename) - 1);
                len = 0;
                while (len < (MAX_LOG_DATA - MAX_LOG_LINE)) {
                    int r = get_log_line(pkt.data + len);
                    if (r > 0) {
                        len += r;
                    } else {
                        break;
                    }
                }
                if (len) {
                    // include header and nodename in length
                    len += MAX_NODENAME_LENGTH + sizeof(uint32_t) * 2;
                    pending = 1;
                    goto transmit;
                }
            }
            if (netifc_timer_expired()) {
            transmit:
                if (pending) {
                    udp6_send(&pkt, len, &ip6_ll_all_nodes, DEBUGLOG_PORT, DEBUGLOG_ACK_PORT);
                }
            }
            //TODO: wakeup early for log traffic too
            netifc_set_timer(100);
            if (netifc_poll())
                break;
        }
        netifc_close();
    }

    return 0;
}
