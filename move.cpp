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

#include "rui.hpp"
#include "ethercat.h"
#include "log.hpp"
#include "motor.hpp"

char IOmap[4096];

extern int expectedWKC;
extern volatile int dorun_ecat;
extern volatile int dorun_check;

extern volatile int wkc;
extern boolean inOP;

extern int64 toff, gl_delta;

extern volatile uint64 ecat_thread_loops;
extern volatile uint64 ecat_check_loops;

extern std::array<Motor,64> motor;
extern int app_mode;
extern int selected_slave_no;

int slave_count = 0;


const char *ec_state_text(int state) {
    switch (state) {
    case EC_STATE_NONE: return (char *)"NONE";
    case EC_STATE_INIT: return (char *)"INIT";
    case EC_STATE_PRE_OP: return (char *)"PRE_OP";
    case EC_STATE_BOOT: return (char *)"BOOT";
    case EC_STATE_SAFE_OP: return (char *)"SAFE_OP";
    case EC_STATE_OPERATIONAL: return (char *)"OP";
    case EC_STATE_ACK: return (char *)"ACK";
    }
    return (char *)"??";
}

void dump_slave_state() {
    for (int i = 1; i <= ec_slavecount; i++) {
        unsigned state = ec_slave[i].state & 0xf;
        
        logf("slave %d State=%x(%s%s) StatusCode=%04x (%s)\n",
             i, state, ec_state_text(state), (state & 0x10) ? "+ERR" : "",
             ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
    }
}

int erob_setup()
{
    int retval = 0;
    uint16 map_1c12; // Variable to hold the mapping for PDO
    uint8 zero_map = 0; // Variable to clear the PDO mapping
    uint32 map_object; // Variable to hold the mapping object
    uint16 clear_val = 0x0000; // Value to clear the mapping
    bool cfg_err = false;
    
    //3.- Map RXPOD
    logf("Map RXPOD\n");

    // Loop through each slave
    for (int i = 1; i <= ec_slavecount; i++) {
        // 1. First, disable PDO
        retval = ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(zero_map), &zero_map, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: disable PDO retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // 2. Configure new PDO mapping
        // Control Word
        map_object = 0x60400010;  // 0x6040:0 Control Word, 16 bits
        retval = ec_SDOwrite(i, 0x1600, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 6040 retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // Target Position
        map_object = 0x607A0020;  // 0x607A:0 Target Position, 32 bits
        retval = ec_SDOwrite(i, 0x1600, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 607a retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // Mode of Operation
        map_object = 0x60600008;  // 0x6060:0 Mode of Operation, 8 bits
        retval = ec_SDOwrite(i, 0x1600, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 6060 retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // Padding (8 bits)
        map_object = 0x00000008;  // 8 bits padding
        retval = ec_SDOwrite(i, 0x1600, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: padding retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // Set number of mapped objects
        uint8 map_count = 4;
        retval = ec_SDOwrite(i, 0x1600, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: mapped number retval %d\n", i, retval);
            cfg_err = true;
        }
        
        // 4. Configure RXPDO allocation

        // Clear the mapping
        clear_val = 0x0000;
        retval = ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: clear2 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Set the mapping to the new PDO
        map_1c12 = 0x1600;
        retval = ec_SDOwrite(i, 0x1c12, 0x01, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 1600 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Set the mapping index
        map_1c12 = 0x0001;
        retval = ec_SDOwrite(i, 0x1c12, 0x00, FALSE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 0001 retval %d\n", i, retval);
            cfg_err = true;
        }
    }

    logf("PDO mapping configuration result: %s\n", cfg_err ? "BAD" : "OK");
    if (cfg_err/*retval < 0*/) {
        logf("PDO mapping failed\n");
        return -1;
    }

    logf("RXPOD Mapping set correctly.\n");

    // Map TXPOD
    uint16 map_1c13;
    for (int i = 1; i <= ec_slavecount; i++) {
        // First, clear the TXPDO mapping
        clear_val = 0x0000;
        retval = ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: clean TXPDO mapping retval %d\n", i, retval);
            cfg_err = true;
        }

        // Configure TXPDO mapping entries
        // Status Word (0x6041:0, 16 bits)
        map_object = 0x60410010;
        retval = ec_SDOwrite(i, 0x1A00, 0x01, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 6041 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Actual Position (0x6064:0, 32 bits)
        map_object = 0x60640020;
        retval = ec_SDOwrite(i, 0x1A00, 0x02, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 6064 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Actual Velocity (0x606C:0, 32 bits)
        map_object = 0x606C0020;
        retval = ec_SDOwrite(i, 0x1A00, 0x03, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 606c retval %d\n", i, retval);
            cfg_err = true;
        }

        // Actual Torque (0x6077:0, 16 bits)
        map_object = 0x60770010;
        retval = ec_SDOwrite(i, 0x1A00, 0x04, FALSE, sizeof(map_object), &map_object, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 6077 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Set the number of mapped objects (4 objects)
        uint8 map_count = 4;
        retval = ec_SDOwrite(i, 0x1A00, 0x00, FALSE, sizeof(map_count), &map_count, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: map count retval %d\n", i, retval);
            cfg_err = true;
        }

        // Configure TXPDO assignment
        // First, clear the assignment
        clear_val = 0x0000;
        retval = ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(clear_val), &clear_val, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: clear assignment retval %d\n", i, retval);
            cfg_err = true;
        }

        // Assign TXPDO to 0x1A00
        map_1c13 = 0x1A00;
        retval = ec_SDOwrite(i, 0x1C13, 0x01, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: 1a00 retval %d\n", i, retval);
            cfg_err = true;
        }

        // Set the number of assigned PDOs (1 PDO)
        map_1c13 = 0x0001;
        retval = ec_SDOwrite(i, 0x1C13, 0x00, FALSE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
        if (retval <= 0) {
            logf("slave%d: map count2 retval %d\n", i, retval);
            cfg_err = true;
        }
    }

    logf("TXPDO mapping configuration result: %s\n", cfg_err ? "BAD" : "OK");

    if (cfg_err/*retval < 0*/) {
        logf("TXPDO Mapping failed\n");
        return -1;
    }

    logf("TXPDO Mapping set successfully\n");

    ecx_context.manualstatechange = 1; //Disable automatic state change
    osal_usleep(1e6); //Sleep for 1 second

    uint8 WA = 0; //Variable for write access
    uint8 my_RA = 0; //Variable for read access
    uint32 TIME_RA; //Variable for time read access

    // Print the information of the slaves found
    for (int i = 1; i <= ec_slavecount; i++) {
        // (void)ecx_FPWR(ecx_context.port, i, ECT_REG_DCSYNCACT, sizeof(WA), &WA, 5 * EC_TIMEOUTRET);
        logf("Name: %s\n", ec_slave[i].name);
        logf("Slave %d: Type %d, Address 0x%02x, State actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_INIT);

//        ecx_dcsync0(&ecx_context, i, TRUE, 500000, 0);  //Synchronize the distributed clock for the slave
        ecx_dcsync0(&ecx_context, i, TRUE, 1000000, 0);
    }

    logf("checking all slaves for pre-op\n");

    // Ensure all slaves are in PRE-OP state
    for (int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_PRE_OP) { // Check if the slave is not in PRE-OP state
            logf("Slave %d not in PRE-OP state. Current state: %d\n", i, ec_slave[i].state);
            return -1; // Return error if any slave is not in PRE-OP state
        }
    }

    return 0;
}

int startup(char *ifname)
{
    int i, j, oloop, iloop;

    logf("Starting test\n");
    dorun_ecat = 0;
    dorun_check = 0;

    // initialise SOEM, bind socket to ifname
    if (ec_init(ifname) == 0) {
        logf("No socket connection on %s\nExcecute as root\n",ifname);
        return -1;
    }
        
    logf("ec_init on %s succeeded.\n", ifname);

    // find and auto-config slaves
    if (ec_config(FALSE, &IOmap) == 0) {
        logf("No slaves found!\n");
        ec_close();
        return -1;
    }
    
    logf("%d slaves found.\n", ec_slavecount);
    slave_count = ec_slavecount;

    // read indevidual slave state and store in ec_slave[]
    ec_readstate();

    bool into_init = false;
    
    // Loop through each slave
    for (int i = 1; i <= ec_slavecount; i++) {
        // If the slave is not in PRE-OP state
        if(ec_slave[i].state != EC_STATE_PRE_OP) {
            // Print the current state and status code of the slave
            logf("Slave %d State=%x StatusCode=%04x : %s\n",
                   i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));

            logf("Request init state for slave %d\n", i);
            // Request to change the state to INIT
            ec_slave[i].state = EC_STATE_INIT;
            into_init = true;
        }
    }

    if (into_init) {
        // Set the first slave to PRE-OP state
        ec_slave[0].state = EC_STATE_PRE_OP;
        for (int i = 1; i <= ec_slavecount; i++) {
            ec_slave[i].state = EC_STATE_PRE_OP;
        }

        // Request EC_STATE_PRE_OP state for all slaves
        ec_writestate(0);

        // Wait for all slaves to reach the PRE-OP state
        if ((ec_statecheck(0, EC_STATE_PRE_OP,  3 * EC_TIMEOUTSTATE)) == EC_STATE_PRE_OP) {
            logf("State changed to EC_STATE_PRE_OP\n");
        } else {
            logf("State not changed to EC_STATE_PRE_OP\n");
            return -1; // Return error if state change fails
        }
    }

    if (erob_setup()) {
        return -1;
    }
    
    // Configure Distributed Clock (DC)
    ec_configdc();

    // Request to switch to SAFE-OP state
    ec_slave[0].state = EC_STATE_SAFE_OP; // Set the first slave to SAFE-OP state
    ec_writestate(0); // Write the state change to the slave

    // Wait for the state transition
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP) {
        logf("Successfully changed to SAFE_OP state\n"); // Confirm successful state change
    } else {
        logf("Failed to change to SAFE_OP state\n");
        //return -1; // Return error if state change fails
    }

    // Calculate the expected Work Counter (WKC)
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC; // Calculate expected WKC based on outputs and inputs
    logf("Calculated workcounter %d\n", expectedWKC);

    // Read and display basic status information of the slaves
    ec_readstate();

#if 0
    for(int i = 1; i <= ec_slavecount; i++) {
        logf("Slave %d: State %02x, ALStatusCode %04x, Delay %d, Has DC%d, DC Active%d, DC supported%d\n",
             i, 
             ec_slave[i].state, ec_slave[i].ALstatuscode, ec_slave[i].pdelay,
             ec_slave[i].hasdc, ec_slave[i].DCactive, ec_slave[i].hasdc);
    }
#else
    logf("AFTER INIT:\n");
    dump_slave_state();
#endif
    
    /* 
       This section attempts to bring all slaves to operational status. It does so
       by attempting to set the status of all slaves (ec_slave[0]) to operational,
       then proceeding through 40 send/recieve cycles each waiting up to 50 ms for a
       response about the status. 
    */
    // activate cyclic process data 
    dorun_check = 1;
    dorun_ecat = 1;
    usleep(1);

    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    ec_slave[0].state = EC_STATE_OPERATIONAL;
//ec_slave[1].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
//ec_writestate(1);

    logf("Request operational state for all slaves\n");

//    // activate cyclic process data 
//    dorun_check = 1;
//    dorun_ecat = 1;
//    usleep(1);
    
#if 0
    int chk = 10;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000); // 50 ms wait for state check
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
#endif

//    logf("AFTER OP:\n");
//    dump_slave_state();
    
#if 0
    // Checking all slaves achieved operational states
    if (ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL) {
        logf("OPERATIONAL state not set\n");
        //return -1;
    }
#endif

//    int ret = ec_statecheck(0, EC_STATE_OPERATIONAL, 2 * EC_TIMEOUTSTATE);
//    logf("move to op returns %d\n", ret);

//    // Wait for the state transition to complete
//    if ((ec_statecheck(0, EC_STATE_OPERATIONAL, 2 * EC_TIMEOUTSTATE)) == EC_STATE_OPERATIONAL) {
//        logf("State changed to EC_STATE_OPERATIONAL: %d\n", EC_STATE_OPERATIONAL); // Confirm successful state change
//    } else {
//        logf("State could not be changed to EC_STATE_OPERATIONAL\n"); // Error message if state change fails
//        for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
//            logf("slave%d: ALstatuscode: %d\n", cnt, ecx_context.slavelist[cnt].ALstatuscode); // Print AL status codes for each slave
//        }
//
//        logf("OP MOVE FAIL:\n");
//        dump_slave_state();
//        return -1;
//    }

    // Read and display the state of all slaves
    ec_readstate(); // Read the state of all slaves
    for (int i = 1; i <= ec_slavecount; i++) {
        logf("Slave %d: Type %d, Address 0x%02x, State actual %d, required %d\n", 
               i, ec_slave[i].eep_id, ec_slave[i].configadr, ec_slave[i].state, EC_STATE_OPERATIONAL); // Print slave information
        logf("Name: %s\n", ec_slave[i].name); // Print the name of the slave
    }

    dorun_ecat = 1;
    dorun_check = 1;
    
    logf("startup done\n");
    return 0;
}


int shutdown() {
    logf("Request safe operational state for all slaves\n");
    ec_slave[0].state = EC_STATE_SAFE_OP;
    
    // request SAFE_OP state for all slaves
    ec_writestate(0);

    // stop SOEM, close socket
    ec_close();

    return 0;
}

int run() {
    bool all_op = true;

    //sleep(2);
    logf("ecat thread loops %ld, check loops %ld\n", ecat_thread_loops, ecat_check_loops);

    for (int i = 1; i <= ec_slavecount; i++) {
        if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
            logf("slave%d State=%x(%s) StatusCode=%4x : %s\n",
                   i, ec_slave[i].state, ec_state_text(ec_slave[i].state),
                   ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            all_op = false;
        } else {
            logf("slave %d OP\n", i);
        }
    }
    
    if (!all_op) {
        logf("Not all slaves reached operational state.\n");
        dorun_ecat = 0;
        dorun_check = 0;

        ec_readstate();

        for (int i = 1; i <= ec_slavecount; i++) {
            if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
                logf("slave%d State=%x(%s) StatusCode=%04x(%s)\n",
                     i,
                     ec_slave[i].state, ec_state_text(ec_slave[i].state),
                     ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }

        return -1;
    }
    
    logf("Operational state reached for all slaves.\n");
    inOP = TRUE;

    // acyclic loop 5000 x 20ms = 10s
    for (int i = 1; i <= 5000; i++) {
        statusf(1, "cycle %5d, Wck %3d, DCtime %12ld, dt %12ld\n",
                dorun_ecat, wkc, ec_DCtime, gl_delta);

        osal_usleep(20000);
    }

    dorun_ecat = 0;
    dorun_check = 0;
    inOP = FALSE;

    return 0;
}

uint32_t slave_reported_pos(int slaven) {
    return motor[slaven].reported_pos();
}

uint32_t slave_requested_pos(int slaven) {
    return motor[slaven].requested_pos();
}

void slave_set_moving(int slaven, int yesno) {
    motor[slaven].set_moving(yesno);
}

int slave_moving(int slaven) {
    return motor[slaven].moving();
}

//
// @brief Called repeatedly by main loop
//
int run_loop(unsigned count)
{
    if (count == 0) {
        bool all_op = true;

        usleep(1);
        //sleep(2);
        logf("ecat thread loops %ld, check loops %ld\n", ecat_thread_loops, ecat_check_loops);

        for (int i = 1; i <= ec_slavecount; i++) {
            if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
                logf("slave%d State=%x(%s) StatusCode=%04x(%s)\n",
                     i,
                     ec_slave[i].state, ec_state_text(ec_slave[i].state),
                     ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                all_op = false;
            } else {
                logf("slave %d OP\n", i);
            }
        }
    
        if (!all_op) {
            logf("Not all slaves reached operational state.\n");
            ec_readstate();

            for (int i = 1; i <= ec_slavecount; i++) {
                if(ec_slave[i].state != EC_STATE_OPERATIONAL) {
                    logf("Slave%d State=%x(%s) StatusCode=%04x(%s)\n",
                         i, ec_slave[i].state, ec_state_text(ec_slave[i].state),
                         ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                }
            }

            while (1) {
                bool ok = true;
                usleep(1000);
                for (int i = 1; i <= ec_slavecount; i++) {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL) ok = false;
                }
                if (ok) break;
            }
            
            for (int i = 1; i <= ec_slavecount; i++) {
                logf("Slave%d State=%x(%s) StatusCode=%04x(%s)\n",
                     i, ec_slave[i].state, ec_state_text(ec_slave[i].state),
                     ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }

//            return -1;
        }
    
        logf("Operational state reached for all slaves.\n");
        
        {
            uint8 operation_mode = 8;
            uint16_t Control_Word = 128;

            for (int i = 1; i <= ec_slavecount; i++) {
                ec_SDOwrite(i, 0x6040, 0x00, FALSE, sizeof(Control_Word), &Control_Word, EC_TIMEOUTSAFE);
                ec_SDOwrite(i, 0x6060, 0x00, FALSE, sizeof(operation_mode), &operation_mode, EC_TIMEOUTSAFE);
            }
        }

        inOP = TRUE;
        count++;
        osal_usleep(20000);
        
        return 0;
    }
    
    statusf(1, "cycle %5d, Wck %3d, DCtime %12ld, dt %12ld\n",
            dorun_ecat, wkc, ec_DCtime, gl_delta);

    if (false) {
        for (int i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].outputs && i < 4) {
                uint8_t *po = ec_slave[i].outputs; // rxpdo
                statusf(2+(2*(i-1)), "O: %02x%02x %02x%02x%02x%02x %02x\n",
                        po[1], po[0],
                        po[5], po[4], po[3], po[2], 
                        po[6]);

                // PDO 0x1a00
                uint8_t *pi = ec_slave[i].inputs;  // txpdo
                statusf(3+(2*(i-1)), "I: %02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\n",
                        pi[1], pi[0],
                        pi[5], pi[4], pi[3], pi[2],
                        pi[9], pi[8], pi[7], pi[6],
                        pi[13], pi[12], pi[11], pi[10]);
            }
        }

        statusf(8, "req: %8u act: %8u\n", motor[0].requested_pos(), motor[0].reported_pos());
    }
    

    if ((count % 20) == 0) {
        // display motor status
        for (int s = 0; s < slave_count; s++) {
            int ec_index = s+1;
            if (ec_slave[ec_index].outputs) {
                uint8_t *po = ec_slave[ec_index].outputs; // rxpdo
                uint8_t *pi = ec_slave[ec_index].inputs;  // txpdo

                uint32_t req_pos = (po[5] << 24) | (po[4] << 16) | (po[3] << 8) | (po[2] << 0);

                uint16_t status = (pi[1] << 8) | (pi[0] << 0);
                uint32_t rep_pos = (pi[5] << 24) | (pi[4] << 16) | (pi[3] << 8) | (pi[2] << 0);
                int32_t rep_vel = (pi[9] << 24) | (pi[8] << 16) | (pi[7] << 8) | (pi[6] << 0);
                int32_t rep_torque = (pi[13] << 24) | (pi[12] << 16) | (pi[11] << 8) | (pi[10] << 0);
                
                if (rep_torque & 0x8000) rep_torque |= 0xffff0000;

                motor[s].set_ecat_state(ec_slave[ec_index].state);

                motor[s].set_requested_pos(req_pos);
                motor[s].set_reported_status(status);
                motor[s].set_reported_pos(rep_pos);
                motor[s].set_reported_vel(rep_vel);
                motor[s].set_reported_torque(rep_torque);
            }

            motor[s].set_index(s);
            motor[s].set_cycles(count);
        }

        for (int s = 0; s < slave_count; s++) {
            motor[s].display();
        }
    }
    
    osal_usleep(20000);

    return 0;
}

char *status_word_text(uint16_t sw) {
    static char buf[64];
    buf[0] = 0;

    if (sw & (1 << 0)) strcat(buf, "RSWO ");
    if (sw & (1 << 1)) strcat(buf, "SWON ");
    if (sw & (1 << 2)) strcat(buf, "OPEN ");
    if (sw & (1 << 3)) strcat(buf, "F ");
    if (sw & (1 << 4)) strcat(buf, "VE ");
    if (sw & (1 << 5)) strcat(buf, "QS ");
    if (sw & (1 << 6)) strcat(buf, "SWOD ");
    if (sw & (1 << 7)) strcat(buf, "W ");
    if (sw & (1 << 9)) strcat(buf, "R ");
    if (sw & (1 << 10)) strcat(buf, "TR ");
    return buf;
}


