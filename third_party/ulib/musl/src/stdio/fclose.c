#include "libc.h"
#include "stdio_impl.h"

int fclose(FILE* f) {
    int r;
    int perm;

    FLOCK(f);

    if (!(perm = f->flags & F_PERM)) {
        FILE** head = __ofl_lock();
        if (f->prev)
            f->prev->next = f->next;
        if (f->next)
            f->next->prev = f->prev;
        if (*head == f)
            *head = f->next;
        __ofl_unlock();
    }

    r = fflush(f);
    r |= f->close(f);

    if (f->getln_buf)
        free(f->getln_buf);
    if (!perm)
        free(f);
    else
        FUNLOCK(f);

    return r;
}
