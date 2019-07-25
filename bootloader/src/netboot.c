// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdio.h>
#include <string.h>

#include <device_id.h>
#include <inet6.h>
#include <netifc.h>

#include <magenta/boot/netboot.h>

static uint32_t last_cookie = 0;
static uint32_t last_cmd = 0;
static uint32_t last_arg = 0;
static uint32_t last_ack_cmd = 0;
static uint32_t last_ack_arg = 0;

static int nb_boot_now = 0;
static int nb_active = 0;

// item being downloaded
static nbfile* item;

static char advertise_nodename[64] = "";
static char advertise_data[256] = "nodename=magenta";

static void send_query_ack(const ip6_addr* addr, uint16_t port,
                           uint32_t cookie) {
    uint8_t buffer[256];
    nbmsg* msg = (void*)buffer;
    msg->magic = NB_MAGIC;
    msg->cookie = cookie;
    msg->cmd = NB_ACK;
    msg->arg = NB_VERSION_CURRENT;
    memcpy(msg->data, advertise_nodename, sizeof(advertise_nodename));
    udp6_send(buffer, sizeof(nbmsg) + strlen(advertise_nodename) + 1,
              addr, port, NB_SERVER_PORT);
}

static void advertise(void) {
    uint8_t buffer[sizeof(nbmsg) + sizeof(advertise_data)];
    nbmsg* msg = (void*)buffer;
    msg->magic = NB_MAGIC;
    msg->cookie = 0;
    msg->cmd = NB_ADVERTISE;
    msg->arg = NB_VERSION_CURRENT;
    size_t data_len = strlen(advertise_data) + 1;
    memcpy(msg->data, advertise_data, data_len);
    udp6_send(buffer, sizeof(nbmsg) + data_len, &ip6_ll_all_nodes,
              NB_ADVERT_PORT, NB_SERVER_PORT);
}

void udp6_recv(void* data, size_t len,
               const ip6_addr* daddr, uint16_t dport,
               const ip6_addr* saddr, uint16_t sport) {
    nbmsg* msg = data;
    nbmsg ack;
    int do_transmit = 1;

    if (dport != NB_SERVER_PORT)
        return;

    if (len < sizeof(nbmsg))
        return;
    len -= sizeof(nbmsg);

    // printf("netboot: MSG %08x %08x %08x %08x datalen %zu\n",
    //        msg->magic, msg->cookie, msg->cmd, msg->arg, len);

    if ((last_cookie == msg->cookie) &&
        (last_cmd == msg->cmd) && (last_arg == msg->arg)) {
        // host must have missed the ack. resend
        ack.magic = NB_MAGIC;
        ack.cookie = last_cookie;
        ack.cmd = last_ack_cmd;
        ack.arg = last_ack_arg;
        goto transmit;
    }

    ack.cmd = NB_ACK;
    ack.arg = 0;

    switch (msg->cmd) {
    case NB_COMMAND:
        if (len == 0)
            return;
        msg->data[len - 1] = 0;
        break;
    case NB_SEND_FILE:
        if (len == 0)
            return;
        msg->data[len - 1] = 0;
        for (int i = 0; i < (len - 1); i++) {
            if ((msg->data[i] < ' ') || (msg->data[i] > 127)) {
                msg->data[i] = '.';
            }
        }
        item = netboot_get_buffer((const char*)msg->data, msg->arg);
        if (item) {
            item->offset = 0;
            ack.arg = msg->arg;
            printf("netboot: Receive File '%s'...\n", (char*) msg->data);
        } else {
            printf("netboot: Rejected File '%s'...\n", (char*) msg->data);
            ack.cmd = NB_ERROR_BAD_FILE;
        }
        break;

    case NB_DATA:
    case NB_LAST_DATA:
        if (item == 0) {
            printf("netboot: > received chunk before NB_FILE\n");
            return;
        }
        if (msg->arg != item->offset) {
            // printf("netboot: < received chunk at offset %d but current offset is %zu\n", msg->arg, item->offset);
            ack.arg = item->offset;
            ack.cmd = NB_ACK;
        } else if ((item->offset + len) > item->size) {
            ack.cmd = NB_ERROR_TOO_LARGE;
            ack.arg = msg->arg;
        } else {
            memcpy(item->data + item->offset, msg->data, len);
            item->offset += len;
            ack.cmd = msg->cmd == NB_LAST_DATA ? NB_FILE_RECEIVED : NB_ACK;
            if (msg->cmd != NB_LAST_DATA) {
                do_transmit = 0;
            }
        }
        break;
    case NB_BOOT:
        nb_boot_now = 1;
        printf("netboot: Boot Kernel...\n");
        break;
    case NB_QUERY:
        // Send reply and return w/o getting the netboot state out of sync.
        send_query_ack(saddr, sport, msg->cookie);
        return;
    default:
        ack.cmd = NB_ERROR_BAD_CMD;
        ack.arg = 0;
    }

    last_cookie = msg->cookie;
    last_cmd = msg->cmd;
    last_arg = msg->arg;
    last_ack_cmd = ack.cmd;
    last_ack_arg = ack.arg;

    ack.cookie = msg->cookie;
    ack.magic = NB_MAGIC;
transmit:
    nb_active = 1;
    if (do_transmit) {
        // printf("netboot: MSG %08x %08x %08x %08x\n",
        //   ack.magic, ack.cookie, ack.cmd, ack.arg);

        udp6_send(&ack, sizeof(ack), saddr, sport, NB_SERVER_PORT);
    }
}

#define FAST_TICK 100
#define SLOW_TICK 1000

int netboot_init(const char* nodename) {
    if (netifc_open()) {
        printf("netboot: Failed to open network interface\n");
        return -1;
    }
    char buf[DEVICE_ID_MAX];
    if (!nodename || (nodename[0] == 0)) {
        device_id(eth_addr(), buf);
        nodename = buf;
    }
    if (nodename) {
        strncpy(advertise_nodename, nodename, sizeof(advertise_nodename) - 1);
        snprintf(advertise_data, sizeof(advertise_data),
                 "version=%s;nodename=%s", BOOTLOADER_VERSION, nodename);
    }
    return 0;
}

const char* netboot_nodename() {
    return advertise_nodename;
}

static int nb_fastcount = 0;
static int nb_online = 0;

int netboot_poll(void) {
    if (netifc_active()) {
        if (nb_online == 0) {
            printf("netboot: interface online\n");
            nb_online = 1;
            nb_fastcount = 20;
            netifc_set_timer(FAST_TICK);
            advertise();
        }
    } else {
        if (nb_online == 1) {
            printf("netboot: interface offline\n");
            nb_online = 0;
        }
        return 0;
    }
    if (netifc_timer_expired()) {
        if (nb_fastcount) {
            nb_fastcount--;
            netifc_set_timer(FAST_TICK);
        } else {
            netifc_set_timer(SLOW_TICK);
        }
        if (nb_active) {
            // don't advertise if we're in a transfer
            nb_active = 0;
        } else {
            advertise();
        }
    }

    netifc_poll();

    if (nb_boot_now) {
        nb_boot_now = 0;
        return 1;
    } else {
        return 0;
    }
}

void netboot_close(void) {
    netifc_close();
}
