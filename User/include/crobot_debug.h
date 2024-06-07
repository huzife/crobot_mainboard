#ifndef USER_CROBOT_DEBUG_H
#define USER_CROBOT_DEBUG_H

#include "dprintf.h"
#include <stdbool.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#define LOG_COMMON_LEVEL       0
#define LOG_ERR_LEVEL       (LOG_COMMON_LEVEL + 1)
#define LOG_DEBUG_LEVEL     (LOG_ERR_LEVEL + 1)

#ifdef CROBOT_DEBUG
#define PRINT_LEVEL         LOG_DEBUG_LEVEL
#else
#define PRINT_LEVEL         LOG_ERR_LEVEL
#endif

extern bool log_level_check(int level);
#define CROBOT_PRINTF(level, fmt, args...)   do { \
    if (log_level_check(level)) {                    \
        printf(fmt, ##args);                         \
    }                                                \
} while (0)

#define PRINT_DEBUG(fmt, args...)    CROBOT_PRINTF(LOG_DEBUG_LEVEL, fmt, ##args)
#define PRINT_ERR(fmt, args...)      CROBOT_PRINTF(LOG_ERR_LEVEL, fmt, ##args)
#define PRINTK(fmt, args...)         CROBOT_PRINTF(LOG_COMMON_LEVEL, fmt, ##args)

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */

#endif // USER_CROBOT_DEBUG_H
