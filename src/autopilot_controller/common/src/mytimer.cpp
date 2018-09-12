#include "mytimer.hpp"
#include <sys/time.h>
#include <unistd.h>
#include <cassert>
#include <cstring>
#include <cstdio>

struct timeval mytimer::getBeijingTimeval() {
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    tv.tv_sec -= (tz.tz_minuteswest - CCT * 60) * 60;
    return tv;
}

struct timeval mytimer::getTimeVal() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv;
}

unsigned int mytimer::timevalDiffInUs(const struct timeval *t0, const struct timeval *t1) {
    struct timeval result;

    if (t0->tv_sec > t1->tv_sec)
        return -timevalDiffInUs(t1, t0);

    if ((t0->tv_sec == t1->tv_sec) && (t0->tv_usec > t1->tv_usec))
        return -timevalDiffInUs(t1, t0);

    result.tv_sec = (t1->tv_sec - t0->tv_sec);
    result.tv_usec = (t1->tv_usec - t0->tv_usec);

    if (result.tv_usec < 0) {
        result.tv_sec--;
        result.tv_usec += 1000000;
    }

    return result.tv_sec * 1000000 + result.tv_usec;
}

void mytimer::extractYYYYMMDD(const struct timeval *tv, YYYYMMDDCStr dateStr) {
    assert(dateStr);
    const struct tm *const ptm = gmtime(&(tv->tv_sec));
    sprintf(dateStr, "%4d-%02d-%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday);
}

void mytimer::getYYYYMMDD(YYYYMMDDCStr dateStr) {
    const struct timeval tv = getBeijingTimeval();
    extractYYYYMMDD(&tv, dateStr);
}

void mytimer::getNULLYYYYMMDD(YYYYMMDDCStr dateStr) {
    strcpy(dateStr, "0000-00-00");
}

void mytimer::extractHHMMSSUS(const struct timeval *tv, HHMMSSUSCStr timeStr) {
    assert(timeStr);
    const struct tm *const ptm = gmtime(&(tv->tv_sec));
    sprintf(timeStr, "%02d:%02d:%02d:%06ld", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, tv->tv_usec);
}

void mytimer::getHHMMSSUS(HHMMSSUSCStr timeStr) {
    const struct timeval tv = getBeijingTimeval();
    extractHHMMSSUS(&tv, timeStr);
}

void mytimer::getNULLHHMMSSUS(YYYYMMDDCStr timeStr) {
    strcpy(timeStr, "00:00:00:000000");
}

void mytimer::stallNusFromTimeVal(const struct timeval *t0, unsigned int n) {
    const struct timeval curTimeVal = getTimeVal();
    const unsigned int elapsedus = timevalDiffInUs(t0, &curTimeVal);

    if (elapsedus < n)
        usleep(n - elapsedus);
}

mytimer::LoopRate::LoopRate(const double loop_rate) :
        loop_rate(loop_rate),
        loop_interval(static_cast<unsigned int>(1000000.0 / loop_rate)) {}

void mytimer::LoopRate::sleep() {
    stallNusFromTimeVal(&last_time_val, loop_interval);

    last_time_val = getTimeVal();
}
