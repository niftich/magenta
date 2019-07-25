#include "libc.h"
#include "time_impl.h"
#include <errno.h>

struct tm* __gmtime_r(const time_t* restrict, struct tm* restrict);

struct tm* __localtime_r(const time_t* restrict t, struct tm* restrict tm) {
    //TODO(kulakowski): remove when timezone lookup works
    return __gmtime_r(t, tm);

    /* Reject time_t values whose year would overflow int because
     * __secs_to_zone cannot safely handle them. */
    if (*t < INT_MIN * 31622400LL || *t > INT_MAX * 31622400LL) {
        errno = EOVERFLOW;
        return 0;
    }
    __secs_to_zone(*t, 0, &tm->tm_isdst, &tm->__tm_gmtoff, 0, &tm->__tm_zone);
    if (__secs_to_tm((long long)*t + tm->__tm_gmtoff, tm) < 0) {
        errno = EOVERFLOW;
        return 0;
    }
    return tm;
}

weak_alias(__localtime_r, localtime_r);
