# Time units

## Userspace exposed time units

*mx\_time\_t* is in nanoseconds.

Use [mx_time_get()](syscalls/time_get.md) to get the current time.

## Kernel-internal time units

*lk\_time\_t* is in nanoseconds.

To get the current time since boot, use:

```
#include <platform.h>

lk_time_t current_time(void);
```
