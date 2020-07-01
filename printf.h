#ifndef MYPRINTF_H
#define MYPRINTF_H

#define printf myprintf

#include <stdarg.h>

#ifdef __cplusplus
extern "C"{
#endif

void init_printf(void (*callback_t)(char*));
void myprintf(char* format, ...);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
