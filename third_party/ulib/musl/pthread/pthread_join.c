#include "pthread_impl.h"
#include <magenta/process.h>

int pthread_join(pthread_t t, void** res) {
    switch (mxr_thread_join(&t->mxr_thread)) {
    case NO_ERROR:
        if (res)
            *res = t->result;
        _mx_vmar_unmap(_mx_vmar_root_self(),
                       (uintptr_t)t->tcb_region.iov_base,
                       t->tcb_region.iov_len);
        return 0;
    default:
        return EINVAL;
    }
}
