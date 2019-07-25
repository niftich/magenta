# mx_ticks_get

## NAME

ticks_get - Read the number of high-precision timer ticks since boot.

## SYNOPSIS

```
#include <magenta/syscalls.h>

uint64_t mx_ticks_get(void)
```

## DESCRIPTION

**mx_ticks_get**() returns the number of high-precision timer ticks since boot.

These ticks may be processor cycles, high speed timer, profiling timer, etc.
They are not guaranteed to continue advancing when the system is asleep.

## RETURN VALUE

**mx_ticks_get**() returns the number of high-precision timer ticks since boot.

## ERRORS

**mx_tick_get**() does not report any error conditions.

## NOTES

The returned value may be highly variable. Factors that can affect it include:
- Changes in processor frequency
- Migration between processors
- Reset of the processor cycle counter
- Reordering of instructions (if required, use a memory barrier)

## SEE ALSO

[ticks_per_second](ticks_per_second.md)
