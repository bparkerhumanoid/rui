#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <array>

#include "rui.hpp"
#include "ethercat.h"
#include "log.hpp"
#include "motor.hpp"
#include "move.hpp"

static int step = 0;

extern std::array<Motor,64> motor;
extern int app_mode;

void ecat_reset_powerups()
{
    step = 0;

    extern void dump_slave_state();
    dump_slave_state();
    
#if 0
    for (int s = 0; s < ec_slavecount; s++) {
        int ec_index = s+1;
        ec_slave[ec_index].state = EC_STATE_SAFE_OP;
        ec_writestate(ec_index);
    }
#endif
}

// @brief called inside ecat thread, do power up sequence
// requires motor be in SAFEOP state
// returns non-zero when power up is done

int ecat_do_powerups()
{
    step++;

    // are we done?
    if (step >= 1200) return 1;
    
    // Send PDO data to the slaves
    for (int s = 0; s < ec_slavecount; s++) {
        int ec_index = s+1;
        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
        txpdo_t *txpdo_p = motor[s].get_txpdo();

        // power state machine control
        if (step <= 400) {
            rxpdo_p->controlword = 0x0080;
            rxpdo_p->target_position = 0;
            motor[s].set_powerstate(1);
        } else if (step <= 600) {
            rxpdo_p->controlword = 0x0006;
            rxpdo_p->target_position = txpdo_p->actual_position;
            motor[s].set_powerstate(2);
        } else if (step <= 800) {
            rxpdo_p->controlword = 0x0007;
            rxpdo_p->target_position = txpdo_p->actual_position;
            motor[s].set_powerstate(3);
        } else if (step <= 1000) {
            rxpdo_p->controlword = 0x000f;
            rxpdo_p->target_position = txpdo_p->actual_position;
            motor[s].set_powerstate(4);
        }

        if (true/*false*/) {
            if (step == 400 || step == 600 || step == 800 || step == 1000) {
                uint8_t *pi = ec_slave[ec_index].inputs;  // txpdo
                uint16_t status = (pi[1] << 8) | (pi[0] << 0);
                logf("slave%d: step%d, state %d, status %04x, cw %04x\n",
                     ec_index, step, ec_slave[ec_index].state, status,
                     rxpdo_p->controlword);
            }
        }

        memcpy(ec_slave[ec_index].outputs, rxpdo_p, sizeof(rxpdo_t));
    }

    return 0;
}

static inline int32 approx(int32 pos)
{
    return pos & 0xfffffff0;
}

void do_mode_idle()
{
    for (int s = 0; s < ec_slavecount; s++) {
        int ec_index = s+1;
        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
        txpdo_t *txpdo_p = motor[s].get_txpdo();
        if (approx(rxpdo_p->target_position) != approx(txpdo_p->actual_position)) {
            rxpdo_p->target_position = txpdo_p->actual_position;
        }
    }
}

// @brief Move all the motors slowly, back and forth
void do_mode_spin()
{
    for (int s = 0; s < ec_slavecount; s++) {
        int ec_index = s+1;
        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
        txpdo_t *txpdo_p = motor[s].get_txpdo();

        motor[s].set_powerstate(5);
                    
        // Update output PDO
        rxpdo_p->controlword = 0x000f;
        rxpdo_p->mode_of_operation = 8;

//#define MAX_DELTA 250
#define MAX_DELTA 200
//#define MAX_POS 524288
//#define MAX_DELTA 50
#define MAX_POS 100000
        //rxpdop->target_position = slave_requested_pos(slave);
        if (motor[s].get_dir() == 0) {
            if (txpdo_p->actual_position + MAX_DELTA >= MAX_POS) {
                rxpdo_p->target_position = txpdo_p->actual_position - 10;
                //if (rxpdo_p->target_position > 524288) rxpdo_p->target_position = 524288;
                motor[s].set_dir(-1);
            } else {
                rxpdo_p->target_position = txpdo_p->actual_position + motor[s].get_delta();
            }
        } else {
            if (txpdo_p->actual_position <= MAX_DELTA) {
                motor[s].set_dir(0);
                rxpdo_p->target_position = txpdo_p->actual_position - 10;
                if (rxpdo_p->target_position < 0) rxpdo_p->target_position = 0;
            } else {
                rxpdo_p->target_position = txpdo_p->actual_position - motor[s].get_delta();
            }
        }
    }
}

void ecat_do_motion()
{
    switch (app_mode) {
    case APPMODE_IDLE:
        do_mode_idle();
        break;
    case APPMODE_SPIN:
        do_mode_spin();
        break;
    }
}

        
