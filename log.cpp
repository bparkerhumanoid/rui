#include <stdio.h>
#include <string.h>
#include <curses.h>

#include "log.hpp"

static char buf[2048];
static WINDOW *log_win;
static unsigned log_count = 0;

int log_init(WINDOW *win) {
    log_win = win;
    return 0;
}

WINDOW *log_window() {
    return log_win;
}

void log_refresh() {
    if (log_win) {
        wrefresh(log_win);
    }
}

int logf(const char *format, ...)
{
    va_list ap;
    int n;
    
    va_start(ap, format);
    n = vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);

    //printw("%s\r\n", buf);
    if (log_win) {
        //if (buf[n] != '\n') strcat(buf, "\n");
        wprintw(log_win, "%8d %s", log_count, buf);
        wrefresh(log_win);
        log_count++;
    }
    
    return 0;
}


static WINDOW *status_win;

int status_init(WINDOW *win) {
    status_win = win;
    return 0;
}

int statusf(int line, const char *format, ...)
{
    va_list ap;
    int n;
    
    va_start(ap, format);
    n = vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);

    if (status_win) {
        int len = strlen(buf);
        if (buf[len-1] == '\n') buf[len-1] = 0;
        
        mvwprintw(status_win, line, 1, "%s", buf);
        wrefresh(status_win);
    }
    
    return 0;
}
