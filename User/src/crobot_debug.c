#include "crobot_debug.h"
#include "FreeRTOS.h"
#include "task.h"

static const char* g_logString[] = {
    "COMMON",
    "ERR",
    "DEBUG"
};

bool log_level_check(int level)
{
    if (level > PRINT_LEVEL) {
        return false;
    }

    if ((level != LOG_COMMON_LEVEL) && (level <= LOG_DEBUG_LEVEL)) {
        PRINTK("[%s][%s]", g_logString[level], pcTaskGetTaskName(xTaskGetCurrentTaskHandle()));
    }

    return true;
}
