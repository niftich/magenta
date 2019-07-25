#define _GNU_SOURCE
#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

char* if_indextoname(unsigned index, char* name) {
    struct ifreq ifr;
    int fd, r;

    if ((fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0)) < 0)
        return 0;
    ifr.ifr_ifindex = index;
    r = ioctl(fd, SIOCGIFNAME, &ifr);
    close(fd);
    return r < 0 ? 0 : strncpy(name, ifr.ifr_name, IF_NAMESIZE);
}
