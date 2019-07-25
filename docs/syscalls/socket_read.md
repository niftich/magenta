# mx_socket_read

## NAME

socket_read - read data from a socket

## SYNOPSIS

```
#include <magenta/syscalls.h>

mx_status_t mx_socket_read(mx_handle_t handle, uint32_t options,
                           void* buffer, size_t size,
                           size_t* actual) {
```

## DESCRIPTION

**socket_read**() attempts to read *size* bytes into *buffer*. If
successful, the number of bytes actually read are return via
*actual*.

If a NULL *buffer* and 0 *size* are passed in, then this syscall
instead requests that the number of outstanding bytes to be returned
via *actual*.

If a NULL *actual* is passed in, it will be ignored.

## RETURN VALUE

**socket_read**() returns **NO_ERROR** on success, and writes into
*actual* (if non-NULL) the exact number of bytes read.

## ERRORS

**ERR_BAD_HANDLE**  *handle* is not a valid handle.

**ERR_WRONG_TYPE**  *handle* is not a socket handle.

**ERR_INVALID_ARGS** If any of *buffer* or *actual* are non-NULL
but invalid pointers, or if *buffer* is NULL but *size* is positive,
or if *options* is nonzero.

**ERR_ACCESS_DENIED**  *handle* does not have **MX_RIGHT_READ**.

**ERR_SHOULD_WAIT**  The socket contained no data to read.

**ERR_PEER_CLOSED**  The other side of the socket is closed, or this
side of the socket has been previously closed via a write with the
**MX_SOCKET_HALF_CLOSE** flag.

**ERR_NO_MEMORY**  (Temporary) Failure due to lack of memory.

## SEE ALSO

[socket_create](socket_create.md),
[socket_write](socket_write.md).
