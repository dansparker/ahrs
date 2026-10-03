/* test.h - minimal test harness */
#ifndef TEST_H
#define TEST_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>

extern int g_failures, g_checks;

#define CHECK(c)                                                         \
    do {                                                                 \
        ++g_checks;                                                      \
        if (!(c)) {                                                      \
            ++g_failures;                                                \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);        \
        }                                                                \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                       \
    do {                                                                                            \
        ++g_checks;                                                                                 \
        const double a_ = (double)(a), b_ = (double)(b);                                            \
        if (!(fabs(a_ - b_) <= (double)(tol))) {                                                    \
            ++g_failures;                                                                           \
            printf("  FAIL %s:%d: %s = %g, expected %g +- %g\n", __FILE__, __LINE__, #a, a_, b_, (double)(tol)); \
        }                                                                                           \
    } while (0)

#define RUN(fn)                   \
    do {                          \
        printf("%s\n", #fn);      \
        fn();                     \
    } while (0)

/* deterministic normal random numbers */
float randn(void);
void rand_seed(uint32_t s);

#endif
