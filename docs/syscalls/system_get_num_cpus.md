# mx_system_num_cpus

## NAME

system_num_cpus - get number of logical processors on the system

## SYNOPSIS

```
#include <magenta/syscalls.h>

uint32_t mx_system_get_num_cpus(void);
```

## DESCRIPTION

**system_get_num_cpus**() returns the number of CPUs (logical processors)
that exist on the system currently running.  This number cannot change
during a run of the system, only at boot time.

## RETURN VALUE

**system_get_num_cpus**() returns the number of CPUs.

## ERRORS

**system_get_num_cpus**() cannot fail.

## NOTES

## SEE ALSO
[system_get_physmem](system_get_physmem.md).
