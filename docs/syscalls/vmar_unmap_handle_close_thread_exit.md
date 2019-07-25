# mx_vmar_unmap_handle_close_thread_exit

## NAME

vmar_unmap_handle_close_thread_exit - unmap memory, close handle, exit

## SYNOPSIS

```
#include <magenta/syscalls.h>

mx_status_t mx_vmar_unmap_handle_close_thread_exit(mx_handle_t vmar_handle,
                                                   uintptr_t addr, size_t len,
                                                   mx_handle_t close_handle);
```

## DESCRIPTION

**vmar_unmap_handle_close_thread_exit**() does a sequence of three operations:
1. `mx_vmar_unmap(vmar_handle, addr, len);`
2. `mx_handle_close(close_handle);`
3. `mx_thread_exit();`

The expectation is that the first operation unmaps a region including the
calling thread's own stack.  (It's not required, but it's permitted.)  This
is valid for this call, though it would be invalid for *mx_vmar_unmap*() or
any other call.

If the *vmar_unmap* operation is successful, then this call never returns.
If `close_handle` is an invalid handle so that the *handle_close* operation
fails, then the thread takes a trap (as if by `__builtin_trap();`).

## RETURN VALUE

**vmar_unmap_handle_close_thread_exit**() does not return on success.

## ERRORS

Same as [*mx_vmar_unmap*()](vmar_unmap.md).

## NOTES

The intended use for this is for a dying thread to unmap its own stack,
close its own thread handle, and exit.  The thread handle cannot be closed
beforehand because closing the last handle to a thread kills that thread.
The stack cannot be unmapped beforehand because the thread must have some
stack space on which to make its final system calls.

This call is used for detached threads, while
[*futex_wake_handle_close_thread_exit*()](futex_wake_handle_close_thread_exit.md)
is used for joinable threads.

## SEE ALSO

[vmar_unmap](vmar_unmap.md),
[handle_close](handle_close.md),
[thread_exit](thread_exit.md),
[futex_wake_handle_close_thread_exit](futex_wake_handle_close_thread_exit.md).
