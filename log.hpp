#include <stdarg.h>
#include <curses.h>

int log_init(WINDOW *win);
int logf(const char *format, ...);
void log_refresh();
WINDOW *log_window();

int status_init(WINDOW *win);
int statusf(int line, const char *format, ...);
