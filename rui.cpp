
#include <stdlib.h>
#include <signal.h>
#include <curses.h>

#include <array>

#include "rui.hpp"
#include "move.hpp"
#include "threads.hpp"
#include "window.hpp"
#include "motor.hpp"
#include "log.hpp"

bool do_demo = false;
bool do_ui = true;

Wwindow logwindow(2, 1, 40, 120, false);
Wwindow statuswindow(41, 1, 10, 120, true);

std::array<Motor,64> motor;

extern int slave_count;

static void finish(int sig) {
    endwin();
    set_curses_inactive();
    
    printf("finish...\n");
    fflush(stdout);

    if (sig != 0) {
        exit(0);
    }
}

int setup_curses() {
    // arrange interrupts to terminate
    signal(SIGINT, finish);

    // initialize the curses library 
    initscr();

    // enable keyboard mapping
    keypad(stdscr, TRUE);

    // tell curses not to do NL->CR/NL on output
    nonl();

    // take input chars one at a time, no wait for \n
    cbreak();

    // echo input - in color
    noecho();

    clear();

    set_curses_active();
    
    return 0;
}

int setup_color() {
    if (has_colors()) {
        // simple assignment
        init_pair(1, COLOR_RED, COLOR_BLACK);
        init_pair(2, COLOR_GREEN, COLOR_BLACK);
    }
    return 0;
}

int make_slave_windows() {

    for (int s = 0; s < slave_count; s++) {
        int height = motor[s].get_win_height();
        motor[s].init(50+(height*s), 1);
    }
    
    return 0;
}

void input_loop() {
    unsigned count = 0;

    timeout(0);
    for (;;) {
        //int c = getch();
        int c = wgetch(log_window());
        if (c < 0) {
            run_loop(count++);
            continue;
        }
        
        if (run_input(c)) break;
    }
}

void demo() {
    for (int l = 0; l < 20; l++) {
        logf("line %d", l);
    }
}

void usage() {
    fprintf(stderr, "usage:\n");
    fprintf(stderr, " -d                   demo\n");
    fprintf(stderr, " -i <interface-name>  specify ethernet inteface\n");
    fprintf(stderr, " -n                   no-ui mode\n");
}

char *intf;

int main(int argc, char *argv[]) {
    int opt;

    printf("rui\n");

    while ((opt = getopt(argc, argv, "di:n")) != -1) {
        switch (opt) {
        case 'd':
            do_demo = true;
            break;
        case 'n':
            do_ui = false;
            break;
        case 'i':
            intf = optarg;
            break;
        }
    }

    if (do_ui) {
        setup_curses();
        setup_color();
    }

    logwindow.show();
    logwindow.scrolling();

    log_init(logwindow.window());
    nodelay(logwindow.window(), TRUE);
    
    statuswindow.show();
    status_init(statuswindow.window());

    if (do_demo) {
        demo();
        sleep(1);
        finish(1);
    }

    if (intf == nullptr) {
        finish(0);
        usage();
        exit(1);
    }
    
    start_threads();

    if (startup(intf) == 0) {

        make_slave_windows();

        if (do_ui) {
            input_loop();
        } else {
            run();
        }

    }

    nocbreak();
    logf("press any key\n");
    while (1) {
        int c = wgetch(log_window());
        if (c > 0) break;
    }

    shutdown();

    logwindow.unshow();
    statuswindow.unshow();

    if (do_ui) {
        finish(0);
    }
    
    return 0;
}
