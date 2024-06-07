#ifndef USER_CROBOT_ATOMIC_H
#define USER_CROBOT_ATOMIC_H

#include <stdbool.h>
#include <stdint.h>

static inline int atomic_read(const volatile int* v){
    int val;

    __asm__ __volatile__("ldrex %0, [%1]\n"
                         : "=&r"(val)
                         : "r"(v)
                         : "cc");

    return val;
}

static inline void atomic_set(volatile int* v, int val) {
    uint32_t status;

    do {
        __asm__ __volatile__("ldrex %0, [%1]\n"
                             "strex %0, %2, [%1]\n"
                             : "=&r"(status)
                             : "r"(v), "r"(val)
                             : "cc");
    } while (__builtin_expect(status != 0, 0));
}

static inline int atomic_add(volatile int* v, int add_val)
{
    int val;
    uint32_t status;

    do {
        __asm__ __volatile__("ldrex %1, [%2]\n"
                             "add %1, %1, %3\n"
                             "strex %0, %1, [%2]"
                             : "=&r"(status), "=&r"(val)
                             : "r"(v), "r"(add_val)
                             : "cc");
    } while (__builtin_expect(status != 0, 0));

    return val;
}

static inline bool atomic_cmp_and_set(volatile int* v, int val, int old_val) {
    int prev_val = 0;
    uint32_t status = 0;

    do {
        __asm__ __volatile__("ldrex %0, %2\n"
                             "mov %1, #0\n"
                             "cmp %0, %3\n"
                             "bne 1f\n"
                             "strex %1, %4, %2\n"
                             "1:"
                             : "=&r"(prev_val), "=&r"(status), "+Q"(*v)
                             : "r"(old_val), "r"(val)
                             : "cc");
    } while (__builtin_expect(status != 0, 0));

    return prev_val != old_val;
}

#endif // USER_CROBOT_ATOMIC_H
