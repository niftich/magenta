# mx_eventpair_create

## NAME

eventpair_create - create an event pair

## SYNOPSIS

```
#include <magenta/syscalls.h>

mx_status_t mx_eventpair_create(uint32_t options, mx_handle_t* out0, mx_handle_t* out1);
```


## DESCRIPTION

**eventpair_create**() creates an event pair, which is a pair of objects that
are mutually signalable.

The signals *MX_EPAIR_SIGNALED* and *MX_USER_SIGNAL_n* (where *n* is 0 through 7)
may be set or cleared using **object_signal**() (modifying the signals on the
object itself), or **object_signal_peer**() (modifying the signals on its
counterpart).

When all the handles to one of the objects have been closed, the *MX_EPAIR_PEER_CLOSED*
signal will be asserted on the opposing object.

The newly-created handles will have the *MX_RIGHT_TRANSER*,
*MX_RIGHT_DUPLICATE*, *MX_RIGHT_READ*, and *MX_RIGHT_WRITE* rights.

Currently, no options are supported, so *options* must be set to 0.


## RETURN VALUE

**eventpair_create**() returns **NO_ERROR** on success. On failure, a (negative)
error code is returned.


## ERRORS

**ERR_INVALID_ARGS**  *out0* or *out1* is an invalid pointer or NULL.

**ERR_NOT_SUPPORTED**  *options* has an unsupported flag set (i.e., is not 0).

**ERR_NO_MEMORY**  (Temporary) Failure due to lack of memory.


## NOTES

The right *MX_RIGHT_WRITE* gates both the ability to modify the object's signals
and the signals of the object's peer.  These should probably become distinct rights.


## SEE ALSO

[event_create](event_create.md),
[handle_close](handle_close.md),
[handle_duplicate](handle_duplicate.md),
[object_wait_one](object_wait_one.md),
[object_wait_many](object_wait_many.md),
[handle_replace](handle_replace.md),
[object_signal](object_signal.md),
[object_signal_peer](object_signal.md).
