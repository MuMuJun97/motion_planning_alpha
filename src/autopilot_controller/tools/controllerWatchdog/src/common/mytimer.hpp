#ifndef _MYTIMER_HPP_
#define _MYTIMER_HPP_

#include <time.h>

namespace mytimer {

    typedef char YYYYMMDDCStr[11];
    typedef char HHMMSSUSCStr[16];

    constexpr int CCT = (+8);

    struct timeval getCurTimeval();

// return time difference in us
    unsigned int timevalDiffInUs(const struct timeval *t0, const struct timeval *t1);

    void extractYYYYMMDD(const struct timeval *tv, YYYYMMDDCStr dateStr);

    void getYYYYMMDD(YYYYMMDDCStr dateStr);

    void getNULLYYYYMMDD(YYYYMMDDCStr dateStr);

    void extractHHMMSSUS(const struct timeval *tv, HHMMSSUSCStr timeStr);

    void getHHMMSSUS(HHMMSSUSCStr timeStr);

    void getNULLHHMMSSUS(YYYYMMDDCStr timeStr);

}

#endif /* _MYTIMER_HPP_ */
