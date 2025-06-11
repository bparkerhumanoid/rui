#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>

#include <array>

#include "ethercat.h"
#include "log.hpp"
#include "motor.hpp"
#include "move.hpp"
#include "threads.hpp"

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

volatile int dorun_ecat = 0;
volatile int dorun_check = 0;

pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

volatile uint64 ecat_thread_loops;
volatile uint64 ecat_check_loops;

extern std::array<Motor,64> motor;
extern int app_mode;
extern int selected_slave_no;

// add ns to timespec
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if ( ts->tv_nsec >= NSEC_PER_SEC ) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

// PI calculation to get linux time synced to DC time
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;

    // set linux sync point 50us later than DC sync, just as example
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2)) { delta= delta - cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

// Add these global variables after the other global declarations
volatile int target_position = 0;
pthread_mutex_t target_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t target_position_cond = PTHREAD_COND_INITIALIZER;
bool target_updated = false;
int32_t received_target = 0;

/* 
 * RT EtherCAT thread function
 * This function handles the real-time processing of EtherCAT data. 
 * It sends and receives process data in a loop, synchronizing with the 
 * distributed clock if available, and ensuring timely execution based on 
 * the specified cycle time.
 */
OSAL_THREAD_FUNC_RT ecat_thread(void *ptr) {
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;
    int missed_cycles = 0;
    const int MAX_MISSED_CYCLES = 10;
    struct timespec cycle_start, cycle_end;
    long cycle_time_ns;
    bool need_powerup = true;
    
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (ts.tv_nsec >= NSEC_PER_SEC) {
        ts.tv_sec++;
        ts.tv_nsec -= NSEC_PER_SEC;
    }
    cycletime = *(int *)ptr * 1000;

    toff = 0;
    dorun_ecat = 0;
    dorun_check = 0;

    for (int s = 0; s < ec_slavecount; s++) {
        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
        int ec_index = s+1;

        rxpdo_p->controlword = 0x0080;
        rxpdo_p->target_position = 0;
        rxpdo_p->mode_of_operation = 8;
        rxpdo_p->padding = 0;

        memcpy(ec_slave[ec_index].outputs, motor[s].get_rxpdo(), sizeof(rxpdo_t));

        motor[s].set_powerstate(0);
    }
    
    // Send initial process data
    ec_send_processdata();

    bool need_update = false;
    int32_t new_target = 0;

    while (1) {
        ecat_thread_loops++;
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);
        
        add_timespec(&ts, cycletime + toff);
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft) != 0) {
            // If sleep is interrupted, record the error
            missed_cycles++;
            logf("WARNING: Clock sleep interrupted, missed cycles: %d\n", missed_cycles);
            if (missed_cycles >= MAX_MISSED_CYCLES) {
                logf("ERROR: Too many missed cycles, attempting recovery...\n");
                // Reset the counter
                missed_cycles = 0;
                // Resynchronize the clock
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
                if (ts.tv_nsec >= NSEC_PER_SEC) {
                    ts.tv_sec++;
                    ts.tv_nsec -= NSEC_PER_SEC;
                }
            }
        } else {
            missed_cycles = 0;
        }
        
        if (dorun_ecat > 0) {
            dorun_ecat++;

            // Receive process data
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            // packet loss?  or one/more motors is misconfigued or reset/no-op
            if (wkc >= 0 && wkc < expectedWKC) {
                if (!need_powerup) {
                    logf("WARNING: Working counter error (wkc: %d, expected: %d)\n", 
                         wkc, expectedWKC);
                    need_powerup = true;
                    ecat_reset_powerups();
                }
            }

            // copy input data to txpdo
            for (int s = 0; s < ec_slavecount; s++) {
                int ec_index = s+1;
                txpdo_t *txpdo_p = motor[s].get_txpdo();
                memcpy(txpdo_p, ec_slave[ec_index].inputs, sizeof(txpdo_t));
            }

            // motor needs to be in safe to turn on
            if (need_powerup) {
                if (ecat_do_powerups()) {
                    need_powerup = false;
                }
            } else {
                ecat_do_motion();
            }
            
            // copy output data to rxpdo
            for (int s = 0; s < ec_slavecount; s++) {
                int ec_index = s+1;
                rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
                memcpy(ec_slave[ec_index].outputs, rxpdo_p, sizeof(rxpdo_t));
            }

#if 0
            if (wkc >= expectedWKC) {
                // Retrieve the current motor status
                for (int s = 0; s < ec_slavecount; s++) {
                    int ec_index = s+1;
                    txpdo_t *txpdo_p = motor[s].get_txpdo();
                    memcpy(txpdo_p, ec_slave[ec_index].inputs, sizeof(txpdo_t));
                }

                // Send PDO data to the slaves
                if (step <= 1000) {
                    for (int s = 0; s < ec_slavecount; s++) {
                        int ec_index = s+1;
                        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
                        txpdo_t *txpdo_p = motor[s].get_txpdo();

                        // State machine control
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

                        if (false) {
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
                } else {
                    for (int s = 0; s < ec_slavecount; s++) {
                        int ec_index = s+1;
                        rxpdo_t *rxpdo_p = motor[s].get_rxpdo();
                        txpdo_t *txpdo_p = motor[s].get_txpdo();

                        motor[s].set_powerstate(5);
                    
                        // Update output PDO
                        rxpdo_p->controlword = 0x000f;
                        rxpdo_p->mode_of_operation = 8;

//#define MAX_DELTA 250
//#define MAX_POS 524288
#define MAX_DELTA 50
#define MAX_POS 100000
                        //rxpdop->target_position = slave_requested_pos(slave);
                        if (motor[s].get_dir() == 0) {
                            if (txpdo_p->actual_position + MAX_DELTA >= MAX_POS) {
                                rxpdo_p->target_position = txpdo_p->actual_position - 10;
                                //if (rxpdo_p->target_position > 524288) rxpdo_p->target_position = 524288;
                                motor[s].set_dir(-1);
                            } else {
                                rxpdo_p->target_position = txpdo_p->actual_position + MAX_DELTA;
                            }
                        } else {
                            if (txpdo_p->actual_position <= MAX_DELTA) {
                                motor[s].set_dir(0);
                                rxpdo_p->target_position = txpdo_p->actual_position - 10;
                                if (rxpdo_p->target_position < 0) rxpdo_p->target_position = 0;
                            } else {
                                rxpdo_p->target_position = txpdo_p->actual_position - MAX_DELTA;
                            }
                        }
                        
                        memcpy(ec_slave[ec_index].outputs, rxpdo_p, sizeof(rxpdo_t));
                    }
                }
                
                if (step < 1200) {
                    step++;
                }
            } else {
                logf("WARNING: Working counter error (wkc: %d, expected: %d)\n", 
                       wkc, expectedWKC);
            }
#endif
            
            // Clock synchronization
            if (ec_slave[0].hasdc) {
                ec_sync(ec_DCtime, cycletime, &toff);
            }

            // Send process data
            ec_send_processdata();
        }

        // Monitor cycle time
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        cycle_time_ns = (cycle_end.tv_sec - cycle_start.tv_sec) * NSEC_PER_SEC +
                       (cycle_end.tv_nsec - cycle_start.tv_nsec);
        
        if (cycle_time_ns > cycletime * 1.5) {
            logf("WARNING: Cycle time exceeded: %ld ns (expected: %ld ns)\n", 
                 cycle_time_ns, cycletime);
        }
    }
}

OSAL_THREAD_FUNC ecat_check( void *ptr )
{
    (void) ptr;

    ec_group[0].docheckstate = 1;
    
    while (1) {
        ecat_check_loops++;
        
        if (dorun_check > 0/*inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)*/) {
            // one ore more slaves are not responding
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();

            for (int s = 0; s < ec_slavecount; s++) {
                int ec_index = s+1;
                
                if (motor[s].get_powerstate() < 5) continue;

                if ((ec_slave[ec_index].group == currentgroup) && (ec_slave[ec_index].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;

                    motor[s].note_state(ec_slave[ec_index].state);
                    
                    if (ec_slave[ec_index].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        if (motor[s].squawk_state()) {
                            logf("ERROR: slave %d is in SAFE_OP + ERROR, attempting ack.\n", ec_index);
                        }
                        ec_slave[ec_index].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(ec_index);
                    } else if(ec_slave[ec_index].state == EC_STATE_SAFE_OP) {
                        if (motor[s].squawk_state()) {
                            logf("WARNING: slave %d is in SAFE_OP, change to OPERATIONAL.\n", ec_index);
                        }
                        ec_slave[ec_index].state = EC_STATE_OPERATIONAL;
                        ec_writestate(ec_index);
                    } else if(ec_slave[ec_index].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(ec_index, EC_TIMEOUTMON)) {
                            ec_slave[ec_index].islost = FALSE;
                            logf("MESSAGE: slave %d reconfigured\n", ec_index);
                            motor[s].reset_state();
                        }
                    } else if(!ec_slave[ec_index].islost) {
                        // re-check state
                        ec_statecheck(ec_index, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[ec_index].state == EC_STATE_NONE) {
                            ec_slave[ec_index].islost = TRUE;
                            logf("ERROR: slave %d lost\n", ec_index);
                            motor[s].reset_state();
                        }
                    }
                }

                if (ec_slave[ec_index].islost) {
                    if(ec_slave[ec_index].state == EC_STATE_NONE) {
                        if (ec_recover_slave(ec_index, EC_TIMEOUTMON)) {
                            ec_slave[ec_index].islost = FALSE;
                            logf("MESSAGE: slave %d recovered\n", ec_index);
                            motor[s].reset_state();
                        }
                    } else {
                        ec_slave[ec_index].islost = FALSE;
                        logf("MESSAGE: slave %d found\n", ec_index);
                        motor[s].reset_state();
                    }
                }
            }

            if (!ec_group[currentgroup].docheckstate) {
                if (0) logf("OK : all slaves resumed OPERATIONAL.\n");
            }
        }

        osal_usleep(10000*10);
    }
}

#define stack64k (64 * 1024)

int start_threads() {
    int cytime = 1000;
    
    // Set a higher real-time priority
    struct sched_param param;
    param.sched_priority = 99; // Maximum real-time priority
    if (sched_setscheduler(0, /*SCHED_FIFO*/SCHED_RR, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    // Set CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(10, &cpuset);
    CPU_SET(11, &cpuset);
    CPU_SET(12, &cpuset);

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) == -1) {
        perror("sched_setaffinity");
        return EXIT_FAILURE;
    }

    // create RT thread
    osal_thread_create_rt(&thread1, stack64k * 8, (void *)&ecat_thread, (void*) &cytime);

    // create thread to handle slave error handling in OP
    osal_thread_create(&thread2, stack64k * 8, (void *)&ecat_check, NULL);

    return 0;
}


