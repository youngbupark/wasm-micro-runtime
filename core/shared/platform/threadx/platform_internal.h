#ifndef _PLATFORM_INTERNAL_H
#define _PLATFORM_INTERNAL_H

#include "tx_api.h"

#include <inttypes.h>
#include <stdarg.h>
#include <ctype.h>
#include <limits.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#ifndef BH_PLATFORM_THREADX
#define BH_PLATFORM_THREADX
#endif

#define BH_APPLET_PRESERVED_STACK_SIZE (2 * BH_KB)

/* Default thread priority */
#define BH_THREAD_DEFAULT_PRIORITY 7

typedef TX_THREAD korp_thread;
typedef korp_thread *korp_tid;
typedef TX_MUTEX korp_mutex;
typedef TX_SEMAPHORE korp_sem;

struct os_thread_wait_node;
typedef struct os_thread_wait_node *os_thread_wait_list;
typedef struct korp_cond {
    korp_mutex wait_list_lock;
    os_thread_wait_list thread_wait_list;
} korp_cond;

/* ThreadX time conversion */
#define TX_US_TICK_VALUE        (1000000UL / (uint64)TX_TIMER_TICKS_PER_SECOND)
#define TX_MS_TICK_VALUE        (1000UL / TX_TIMER_TICKS_PER_SECOND)
#define TX_MS_TO_TICKS(ms)      ((ms) < TX_MS_TICK_VALUE ? 1 : (ms) / TX_MS_TICK_VALUE)

/* clang-format off */
size_t strspn(const char *s, const char *accept);
size_t strcspn(const char *s, const char *reject);

/* math functions which are not provided by os */
double atan(double x);
double atan2(double y, double x);
double sqrt(double x);
double floor(double x);
double ceil(double x);
double fmin(double x, double y);
double fmax(double x, double y);
double rint(double x);
double fabs(double x);
double trunc(double x);
float sqrtf(float x);
float floorf(float x);
float ceilf(float x);
float fminf(float x, float y);
float fmaxf(float x, float y);
float rintf(float x);
float fabsf(float x);
float truncf(float x);
int signbit(double x);
int isnan(double x);
double pow(double x, double y);
double scalbn(double x, int n);

unsigned long long int strtoull(const char *nptr, char **endptr, int base);
double strtod(const char *nptr, char **endptr);
float strtof(const char *nptr, char **endptr);

#endif
