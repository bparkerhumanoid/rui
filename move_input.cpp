#include <stdio.h>
#include <stdlib.h>

#include <array>

#include "rui.hpp"
#include "ethercat.h"
#include "log.hpp"
#include "motor.hpp"

extern std::array<Motor,64> motor;
extern int app_mode;
extern int selected_slave_no;
extern int slave_count;

static int input_state = 0;
#define ESC 27

void help()
{
}

const char *app_mode_text(int app_mode)
{
    switch (app_mode) {
    case APPMODE_IDLE: return "Idle";
    case APPMODE_SPIN: return "Spin";
    case APPMODE_INDIVIDUAL: return "Indv";
    case APPMODE_POSE: return "Pose";
    }

    return "?";
}

void input_status_update()
{
    char str[120];
    sprintf(str, "I%c M%s S%d\n", input_state ? input_state : ' ', app_mode_text(app_mode), selected_slave_no);
    statusf(2, str);
}

int state0_input(int ch)
{
    rxpdo_t *rxpdo_p = motor[selected_slave_no-1].get_rxpdo();
    txpdo_t *txpdo_p = motor[selected_slave_no-1].get_txpdo();

    switch (ch) {
    case 'q':
        return -1;
    case 'm':
        input_state = 'm';
        input_status_update();
        return 0;
    case 'a':
        input_state = 'a';
        input_status_update();
        return 0;

    case 'h':
    case '?':
        help();
        return 0;
        
    case 'u':
    case KEY_UP:
        if (app_mode == APPMODE_INDIVIDUAL) {
            rxpdo_p->target_position = txpdo_p->actual_position + 100;
            logf("up %u\n", rxpdo_p->target_position);
        }
        break;
        
    case 'd':
    case KEY_DOWN:
        if (app_mode == APPMODE_INDIVIDUAL) {
            rxpdo_p->target_position = txpdo_p->actual_position - 100;
            logf("down %u\n", rxpdo_p->target_position);
        }            
        break;

    case KEY_LEFT:
        if (selected_slave_no > 1) {
            selected_slave_no--;
            input_status_update();
        }
        break;
    case KEY_RIGHT:
        if (selected_slave_no < slave_count) {
            selected_slave_no++;
            input_status_update();
        }
        break;
    }
    
    return 0;
}

// @brief 'a' actuator
int statea_input(int ch)
{
    input_state = 0;

    if (ch >= '1' && ch <= '9') {
        selected_slave_no = ch - '0';
        input_status_update();
        return 0;
    }

    return 0;
}

// @brief 'm' mode
int statem_input(int ch)
{
    input_state = 0;

    switch (ch) {
    case 'i':
        app_mode = APPMODE_IDLE;
        break;
    case 's':
        app_mode = APPMODE_SPIN;
        break;
    case 'a':
        app_mode = APPMODE_INDIVIDUAL;
        break;
    case 'p':
        app_mode = APPMODE_POSE;
        break;
    }

    input_status_update();
    return 0;
}

// @brief 'p' pose
int statep_input(int ch)
{
    input_state = 0;

    switch (ch) {
    case 's':
        app_mode = APPMODE_SPIN;
        break;
    case 'i':
        app_mode = APPMODE_INDIVIDUAL;
        break;
    }

    input_status_update();
    return 0;
}

        

//
// @brief Called by main loop when keyboard input detecgted
//
int run_input(int ch)
{
    if (ch == ESC) {
        input_state = 0;
        input_status_update();
        return 0;
    }
    
    switch (input_state) {
    case 0: return state0_input(ch);
    case 'a': return statea_input(ch);
    case 'm': return statem_input(ch);
    case 'p': return statep_input(ch);
    }

    return 0;
}

