#ifndef CPUTIME_H
#define CPUTIME_H
// code copied from MiniSAT (version 1.14), 11-APR-2006
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

static inline double cpuTime(void) {
    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);
    return (double)ru.ru_utime.tv_sec + (double)ru.ru_utime.tv_usec / 1000000; }

#endif
