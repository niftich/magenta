// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <inet6.h>

#if 1
#define BAD(n, ...)                 \
    do {                            \
        printf("error: ");          \
        printf(n, ##__VA_ARGS__); \
        printf("\n");               \
        return;                     \
    } while (0)
#else
#define BAD(n)  \
    do {        \
        return; \
    } while (0)
#endif

// useful addresses
const ip6_addr ip6_ll_all_nodes = {
    .x = {0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
};
const ip6_addr ip6_ll_all_routers = {
    .x = {0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2},
};

// Convert MAC Address to IPv6 Link Local Address
// aa:bb:cc:dd:ee:ff => FF80::aabb:ccFF:FEdd:eeff
// bit 2 (U/L) of the mac is inverted
void ll6addr_from_mac(ip6_addr* _ip, const mac_addr* _mac) {
    uint8_t* ip = _ip->x;
    const uint8_t* mac = _mac->x;
    memset(ip, 0, IP6_ADDR_LEN);
    ip[0] = 0xFE;
    ip[1] = 0x80;
    memset(ip + 2, 0, 6);
    ip[8] = mac[0] ^ 2;
    ip[9] = mac[1];
    ip[10] = mac[2];
    ip[11] = 0xFF;
    ip[12] = 0xFE;
    ip[13] = mac[3];
    ip[14] = mac[4];
    ip[15] = mac[5];
}

// Convert MAC Address to IPv6 Solicit Neighbor Multicast Address
// aa:bb:cc:dd:ee:ff -> FF02::1:FFdd:eeff
void snmaddr_from_mac(ip6_addr* _ip, const mac_addr* _mac) {
    uint8_t* ip = _ip->x;
    const uint8_t* mac = _mac->x;
    ip[0] = 0xFF;
    ip[1] = 0x02;
    memset(ip + 2, 0, 9);
    ip[11] = 0x01;
    ip[12] = 0xFF;
    ip[13] = mac[3];
    ip[14] = mac[4];
    ip[15] = mac[5];
}

// Convert IPv6 Multicast Address to Ethernet Multicast Address
void multicast_from_ip6(mac_addr* _mac, const ip6_addr* _ip6) {
    const uint8_t* ip = _ip6->x;
    uint8_t* mac = _mac->x;
    mac[0] = 0x33;
    mac[1] = 0x33;
    mac[2] = ip[12];
    mac[3] = ip[13];
    mac[4] = ip[14];
    mac[5] = ip[15];
}

// ip6 stack configuration
mac_addr ll_mac_addr;
ip6_addr ll_ip6_addr;
mac_addr snm_mac_addr;
ip6_addr snm_ip6_addr;

// cache for the last source addresses we've seen
static mac_addr rx_mac_addr;
static ip6_addr rx_ip6_addr;

void ip6_init(void* macaddr) {
    char tmp[IP6TOAMAX];
    mac_addr all;

    // save our ethernet MAC and synthesize link layer addresses
    memcpy(&ll_mac_addr, macaddr, 6);
    ll6addr_from_mac(&ll_ip6_addr, &ll_mac_addr);
    snmaddr_from_mac(&snm_ip6_addr, &ll_mac_addr);
    multicast_from_ip6(&snm_mac_addr, &snm_ip6_addr);

    eth_add_mcast_filter(&snm_mac_addr);

    multicast_from_ip6(&all, &ip6_ll_all_nodes);
    eth_add_mcast_filter(&all);

    printf("macaddr: %02x:%02x:%02x:%02x:%02x:%02x\n",
           ll_mac_addr.x[0], ll_mac_addr.x[1], ll_mac_addr.x[2],
           ll_mac_addr.x[3], ll_mac_addr.x[4], ll_mac_addr.x[5]);
    printf("ip6addr: %s\n", ip6toa(tmp, &ll_ip6_addr));
    printf("snmaddr: %s\n", ip6toa(tmp, &snm_ip6_addr));
}

mac_addr eth_addr() {
    return ll_mac_addr;
}

static int resolve_ip6(mac_addr* _mac, const ip6_addr* _ip) {
    const uint8_t* ip = _ip->x;

    // Multicast addresses are a simple transform
    if (ip[0] == 0xFF) {
        multicast_from_ip6(_mac, _ip);
        return 0;
    }

    // Trying to send to the IP that we last received a packet from?
    // Assume their mac address has not changed
    if (memcmp(_ip, &rx_ip6_addr, sizeof(rx_ip6_addr)) == 0) {
        memcpy(_mac, &rx_mac_addr, sizeof(rx_mac_addr));
        return 0;
    }

    // We don't know how to find peers or routers yet, so give up...
    return -1;
}

static uint16_t checksum(const void* _data, size_t len, uint16_t _sum) {
    uint32_t sum = _sum;
    const uint16_t* data = _data;
    while (len > 1) {
        sum += *data++;
        len -= 2;
    }
    if (len) {
        sum += (*data & 0xFF);
    }
    while (sum > 0xFFFF) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return sum;
}

typedef struct {
    uint8_t eth[16];
    ip6_hdr ip6;
    uint8_t data[0];
} ip6_pkt;

typedef struct {
    uint8_t eth[16];
    ip6_hdr ip6;
    udp_hdr udp;
    uint8_t data[0];
} udp_pkt;

static unsigned ip6_checksum(ip6_hdr* ip, unsigned type, size_t length) {
    uint16_t sum;

    // length and protocol field for pseudo-header
    sum = checksum(&ip->length, 2, htons(type));
    // src/dst for pseudo-header + payload
    sum = checksum(ip->src, 32 + length, sum);

    // 0 is illegal, so 0xffff remains 0xffff
    if (sum != 0xffff) {
        return ~sum;
    } else {
        return sum;
    }
}

static int ip6_setup(ip6_pkt* p, const ip6_addr* daddr, size_t length, uint8_t type) {
    mac_addr dmac;

    if (resolve_ip6(&dmac, daddr))
        return -1;

    // ethernet header
    memcpy(p->eth + 2, &dmac, ETH_ADDR_LEN);
    memcpy(p->eth + 8, &ll_mac_addr, ETH_ADDR_LEN);
    p->eth[14] = (ETH_IP6 >> 8) & 0xFF;
    p->eth[15] = ETH_IP6 & 0xFF;

    // ip6 header
    p->ip6.ver_tc_flow = 0x60; // v=6, tc=0, flow=0
    p->ip6.length = htons(length);
    p->ip6.next_header = type;
    p->ip6.hop_limit = 255;
    memcpy(p->ip6.src, &ll_ip6_addr, sizeof(ip6_addr));
    memcpy(p->ip6.dst, daddr, sizeof(ip6_addr));

    return 0;
}

#define UDP6_MAX_PAYLOAD (ETH_MTU - ETH_HDR_LEN - IP6_HDR_LEN - UDP_HDR_LEN)

int udp6_send(const void* data, size_t dlen, const ip6_addr* daddr, uint16_t dport, uint16_t sport) {
    size_t length = dlen + UDP_HDR_LEN;
    udp_pkt* p = eth_get_buffer(ETH_MTU + 2);

    if (p == 0)
        return -1;
    if (dlen > UDP6_MAX_PAYLOAD)
        goto fail;
    if (ip6_setup((void*)p, daddr, length, HDR_UDP))
        goto fail;

    // udp header
    p->udp.src_port = htons(sport);
    p->udp.dst_port = htons(dport);
    p->udp.length = htons(length);
    p->udp.checksum = 0;

    memcpy(p->data, data, dlen);
    p->udp.checksum = ip6_checksum(&p->ip6, HDR_UDP, length);
    return eth_send(p->eth + 2, ETH_HDR_LEN + IP6_HDR_LEN + length);

fail:
    eth_put_buffer(p);
    return -1;
}

#define ICMP6_MAX_PAYLOAD (ETH_MTU - ETH_HDR_LEN - IP6_HDR_LEN)

static int icmp6_send(const void* data, size_t length, const ip6_addr* daddr) {
    ip6_pkt* p;
    icmp6_hdr* icmp;

    p = eth_get_buffer(ETH_MTU + 2);
    if (p == 0)
        return -1;
    if (length > ICMP6_MAX_PAYLOAD)
        goto fail;
    if (ip6_setup(p, daddr, length, HDR_ICMP6))
        goto fail;

    icmp = (void*)p->data;
    memcpy(icmp, data, length);
    icmp->checksum = ip6_checksum(&p->ip6, HDR_ICMP6, length);
    return eth_send(p->eth + 2, ETH_HDR_LEN + IP6_HDR_LEN + length);

fail:
    eth_put_buffer(p);
    return -1;
}

void _udp6_recv(ip6_hdr* ip, void* _data, size_t len) {
    udp_hdr* udp = _data;
    uint16_t sum, n;

    if (len < UDP_HDR_LEN)
        BAD("Bogus Header Len");
    if (udp->checksum == 0)
        BAD("Checksum Invalid");
    if (udp->checksum == 0xFFFF)
        udp->checksum = 0;

    sum = checksum(&ip->length, 2, htons(HDR_UDP));
    sum = checksum(ip->src, 32 + len, sum);
    if (sum != 0xFFFF)
        BAD("Checksum Incorrect");

    n = ntohs(udp->length);
    if (n < UDP_HDR_LEN)
        BAD("Bogus Header Len");
    if (n > len)
        BAD("Packet Too Short");
    len = n - UDP_HDR_LEN;

    udp6_recv((uint8_t*)_data + UDP_HDR_LEN, len,
              (void*)ip->dst, ntohs(udp->dst_port),
              (void*)ip->src, ntohs(udp->src_port));
}

void icmp6_recv(ip6_hdr* ip, void* _data, size_t len) {
    icmp6_hdr* icmp = _data;
    uint16_t sum;

    if (icmp->checksum == 0)
        BAD("Checksum Invalid");
    if (icmp->checksum == 0xFFFF)
        icmp->checksum = 0;

    sum = checksum(&ip->length, 2, htons(HDR_ICMP6));
    sum = checksum(ip->src, 32 + len, sum);
    if (sum != 0xFFFF)
        BAD("Checksum Incorrect");

    if (icmp->type == ICMP6_NDP_N_SOLICIT) {
        ndp_n_hdr* ndp = _data;
        struct {
            ndp_n_hdr hdr;
            uint8_t opt[8];
        } msg;

        if (len < sizeof(ndp_n_hdr))
            BAD("Bogus NDP Message");
        if (ndp->code != 0)
            BAD("Bogus NDP Code");
        if (memcmp(ndp->target, &ll_ip6_addr, IP6_ADDR_LEN))
            BAD("NDP Not For Me");

        msg.hdr.type = ICMP6_NDP_N_ADVERTISE;
        msg.hdr.code = 0;
        msg.hdr.checksum = 0;
        msg.hdr.flags = 0x60; // (S)olicited and (O)verride flags
        memcpy(msg.hdr.target, &ll_ip6_addr, IP6_ADDR_LEN);
        msg.opt[0] = NDP_N_TGT_LL_ADDR;
        msg.opt[1] = 1;
        memcpy(msg.opt + 2, &ll_mac_addr, ETH_ADDR_LEN);

        icmp6_send(&msg, sizeof(msg), (void*)ip->src);
        return;
    }

    if (icmp->type == ICMP6_ECHO_REQUEST) {
        icmp->checksum = 0;
        icmp->type = ICMP6_ECHO_REPLY;
        icmp6_send(_data, len, (void*)ip->src);
        return;
    }

    BAD("ICMP6 Unhandled %d", icmp->type);
}

void eth_recv(void* _data, size_t len) {
    uint8_t* data = _data;
    ip6_hdr* ip;
    uint32_t n;

    if (len < (ETH_HDR_LEN + IP6_HDR_LEN))
        BAD("Bogus Header Len");
    if (data[12] != (ETH_IP6 >> 8))
        return;
    if (data[13] != (ETH_IP6 & 0xFF))
        return;

    ip = (void*)(data + ETH_HDR_LEN);
    data += (ETH_HDR_LEN + IP6_HDR_LEN);
    len -= (ETH_HDR_LEN + IP6_HDR_LEN);

    // require v6
    if ((ip->ver_tc_flow & 0xF0) != 0x60)
        BAD("Unknown IP6 Version");

    // ensure length is sane
    n = ntohs(ip->length);
    if (n > len)
        BAD("IP6 Length Mismatch %d %zu", n, len);

    // ignore any trailing data in the ethernet frame
    len = n;

    // require that we are the destination
    if (memcmp(&ll_ip6_addr, ip->dst, IP6_ADDR_LEN) &&
        memcmp(&snm_ip6_addr, ip->dst, IP6_ADDR_LEN) &&
        memcmp(&ip6_ll_all_nodes, ip->dst, IP6_ADDR_LEN)) {
        return;
    }

    // stash the sender's info to simplify replies
    memcpy(&rx_mac_addr, (uint8_t*)_data + 6, ETH_ADDR_LEN);
    memcpy(&rx_ip6_addr, ip->src, IP6_ADDR_LEN);

    if (ip->next_header == HDR_ICMP6) {
        icmp6_recv(ip, data, len);
        return;
    }

    if (ip->next_header == HDR_UDP) {
        _udp6_recv(ip, data, len);
        return;
    }

    BAD("Unhandled IP6 %d", ip->next_header);
}

char* ip6toa(char* _out, void* ip6addr) {
    const uint8_t* x = ip6addr;
    const uint8_t* end = x + 16;
    char* out = _out;
    uint16_t n;

    n = (x[0] << 8) | x[1];
    while ((n == 0) && (x < end)) {
        x += 2;
        n = (x[0] << 8) | x[1];
    }

    if ((end - x) < 16) {
        if (end == x) {
            // all 0s - special case
            sprintf(out, "::");
            return _out;
        }
        // we consumed some number of leading 0s
        out += sprintf(out, ":");
        while (x < end) {
            out += sprintf(out, ":%x", n);
            x += 2;
            n = (x[0] << 8) | x[1];
        }
        return _out;
    }

    while (x < (end - 2)) {
        out += sprintf(out, "%x:", n);
        x += 2;
        n = (x[0] << 8) | x[1];
        if (n == 0)
            goto middle_zeros;
    }
    out += sprintf(out, "%x", n);
    return _out;

middle_zeros:
    while ((n == 0) && (x < end)) {
        x += 2;
        n = (x[0] << 8) | x[1];
    }
    if (x == end) {
        out += sprintf(out, ":");
        return _out;
    }
    while (x < end) {
        out += sprintf(out, ":%x", n);
        x += 2;
        n = (x[0] << 8) | x[1];
    }
    return _out;
}
