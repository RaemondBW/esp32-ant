/*
 * test.h - Tiny zero-dependency unit test harness for the host build.
 */
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

extern int g_tests_run;
extern int g_tests_failed;
extern int g_checks;

#define CHECK(cond) do {                                                      \
        g_checks++;                                                           \
        if (!(cond)) {                                                        \
            g_tests_failed++;                                                 \
            printf("  FAIL %s:%d: CHECK(%s)\n", __FILE__, __LINE__, #cond);  \
            return -1;                                                        \
        }                                                                     \
    } while (0)

#define CHECK_EQ(a, b) do {                                                   \
        g_checks++;                                                           \
        long _va = (long)(a), _vb = (long)(b);                               \
        if (_va != _vb) {                                                     \
            g_tests_failed++;                                                 \
            printf("  FAIL %s:%d: %s (%ld) != %s (%ld)\n",                    \
                   __FILE__, __LINE__, #a, _va, #b, _vb);                     \
            return -1;                                                        \
        }                                                                     \
    } while (0)

#define CHECK_MEM(a, b, n) do {                                               \
        g_checks++;                                                           \
        if (memcmp((a), (b), (n)) != 0) {                                     \
            g_tests_failed++;                                                 \
            printf("  FAIL %s:%d: %s != %s (%d bytes)\n",                     \
                   __FILE__, __LINE__, #a, #b, (int)(n));                     \
            return -1;                                                        \
        }                                                                     \
    } while (0)

#define RUN(fn) do {                                                        \
        g_tests_run++;                                                        \
        printf("- %s\n", #fn);                                                \
        int _before = g_tests_failed;                                         \
        fn();                                                                 \
        if (g_tests_failed == _before) printf("  ok\n");                      \
    } while (0)

#endif /* TEST_H */
