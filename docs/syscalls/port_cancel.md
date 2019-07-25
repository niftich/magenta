# mx_port_cancel

## NAME

port_cancel - cancels async port notifications on an object

## SYNOPSIS

```
#include <magenta/syscalls.h>

mx_status_t mx_port_cancel(mx_handle_t port,
                           mx_handle_t source,
                           uint64_t key);
```

## DESCRIPTION

**port_cancel**() is a non-blocking syscall which cancels
pending **object_wait_async**() calls done with *handle* and *key*.

When this call succeeds no new packets from the object pointed by
*handle* with *key* will be delivered to *port*.

## RETURN VALUE

**mx_port_cancel**() returns **NO_ERROR** if cancellation succeeded.

## ERRORS

**ERR_BAD_HANDLE**  *handle* or *port* is not a valid handle.

**ERR_WRONG_TYPE**  *port* is not a port handle.

**ERR_ACCESS_DENIED**  *handle* or *port* does not have **MX_RIGHT_WRITE**.

**ERR_NOT_SUPPORTED**  *handle* is a handle that cannot be waited on.

## SEE ALSO

[port_wait](port_wait.md).
