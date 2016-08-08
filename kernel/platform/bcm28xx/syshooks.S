
#include <asm.h>
#include <arch/asm_macros.h>


FUNCTION(arm64_get_cache_levels)
    mrs     x1, clidr_el1
    str     x1, [x0]
    ret

FUNCTION(arm64_get_cache_linesize)
    mrs     x1, CCSIDR_EL1
    str     x1, [x0]
    ret