#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

void* __mmap(void*, size_t, int, int, int, off_t);

const char unsigned* __map_file(const char* pathname, size_t* size) {
    struct stat st;
    const unsigned char* map = MAP_FAILED;
    int fd = open(pathname, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    if (fd < 0)
        return 0;
    if (!fstat(fd, &st)) {
        map = __mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
        *size = st.st_size;
    }
    close(fd);
    return map == MAP_FAILED ? 0 : map;
}
