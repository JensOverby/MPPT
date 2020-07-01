#include "stdio.h"
#include "printf.h"

typedef void (*callback_t)(char*);
static callback_t stdout_callback_t;

void myprintf(char* format, ...)
{
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	len = vsnprintf(print_buffer, 255, format, arg);
	va_end (arg);

	if(len > 0)
	{
		stdout_callback_t(print_buffer);
	}
}

void init_printf(void (*callback_t)(char*))
{
	stdout_callback_t=callback_t;
}
