#ifndef CORE_PRINT_H
#define CORE_PRINT_H

#ifdef ENABLE_PRINT
extern int printf(const char* fmt, ...);
#define PRINT(format, ...) printf(format, ##__VA_ARGS__)
#else
#define PRINT(format, ...)
#endif

void print_init();

#endif // CORE_PRINT_H
