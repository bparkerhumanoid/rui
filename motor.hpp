#pragma once

#include "motor.hpp"
#include "window.hpp"

// Structure for RXPDO (Control data sent to slave)
typedef struct {
    uint16_t controlword;      // 0x6040:0, 16 bits
    int32_t target_position;   // 0x607A:0, 32 bits
    uint8_t mode_of_operation; // 0x6060:0, 8 bits
    uint8_t padding;           // 8 bits padding for alignment
} __attribute__((__packed__)) rxpdo_t;

// Structure for TXPDO (Status data received from slave)
typedef struct {
    uint16_t statusword;      // 0x6041:0, 16 bits
    int32_t actual_position;  // 0x6064:0, 32 bits
    int32_t actual_velocity;  // 0x606C:0, 32 bits
    int16_t actual_torque;    // 0x6077:0, 16 bits
} __attribute__((__packed__)) txpdo_t;

class Motor {
public:
    Motor() {}

    int init(int l, int c) {

        m_window.set_pos_size(l, c, m_win_height, m_win_width);
        m_window.show();

        m_window.add_field(MF_STATUS,       1,  1, "status");
        m_window.add_field(MF_INDEX,        1, 20, "index");
        m_window.add_field(MF_CYCLES,       1, 60, "cycles");

        m_window.add_field(MF_ECAT_STATE,   2,  1, "state");
        m_window.add_field(MF_REQ_POS,      2, 20, "req-p");
        m_window.add_field(MF_REQ_VELOCITY, 2, 40, "req-v");
        m_window.add_field(MF_REQ_TORQUE,   2, 60, "req-t");

        m_window.add_field(MF_REP_STATUS,   3,  1, "rep-stat");
        m_window.add_field(MF_REP_POS,      3, 20, "rep-p");
        m_window.add_field(MF_REP_VELOCITY, 3, 40, "rep-v");
        m_window.add_field(MF_REP_TORQUE,   3, 60, "rep-t");

        return 0;
    }
    
    uint32_t reported_pos() const { return m_reported.pos; }
    uint32_t requested_pos() const { return m_requested.pos; }
    bool moving() const { return m_moving; }

    void set_moving(bool yn) { m_moving = yn; }

    void set_requested_pos(uint32_t pos) {
        m_requested.pos = pos;
        m_window.set_field_value_u32(MF_REQ_POS, pos);
        m_updated = true;
    }

    void set_reported_status(uint16_t status) {
        m_reported.status = status;
        m_window.set_field_value_u16(MF_REP_STATUS, status);
        m_updated = true;
    }

    void set_reported_pos(uint32_t pos) {
        m_reported.pos = pos;
        m_window.set_field_value_s32(MF_REP_POS, pos);
        m_updated = true;
    }
    
    void set_reported_vel(int32_t vel) {
        m_reported.vel = vel;
        m_window.set_field_value_s32(MF_REP_VELOCITY, vel);
        m_updated = true;
    }

    void set_reported_torque(int32_t torque) {
        m_reported.torque = torque;
        m_window.set_field_value_s32(MF_REP_TORQUE, torque);
        m_updated = true;
    }

    void set_cycles(uint32_t cycles) {
        m_window.set_field_value_u32(MF_CYCLES, cycles);
        m_updated = true;
    }
    
    void set_index(uint8_t index) {
        m_window.set_field_value_u8(MF_INDEX, index);
        m_updated = true;
    }

    void set_ecat_state(int state) {
        m_window.set_field_value_u8(MF_ECAT_STATE, state);
        m_updated = true;
    }

    void set_dir(int d) { m_dir = d; }

    void note_state(int state) {
        if (m_state != state) m_state_changed = true;
        m_state = state;
    }
    void reset_state() {
        m_state_changed = false;
        m_state = -1;
    }
    bool squawk_state() {
#if 0
        if (m_state_changed) {
            m_state_changed = false;
            return true;
        }
        return false;
#else
        return true;
#endif
    }
    
    int get_win_height() const { return m_win_height; }
    int get_dir() const { return m_dir; }
    
    void display() {
        if (!m_updated) return;

        m_window.display_update_list();

        m_updated = false;
    }

    rxpdo_t *get_rxpdo() { return &m_rxpdo; }
    txpdo_t *get_txpdo() { return &m_txpdo; }

    void set_powerstate(int ps) { m_powerstate = ps; }
    int get_powerstate() const { return m_powerstate; }
    
private:
    Wwindow m_window;
    int m_win_height = 5;
    int m_win_width = 120;
    
    struct {
        uint16_t status;
        int32_t pos;
        int32_t vel;
        uint32_t torque;
    } m_requested{};

    struct {
        uint16_t status;
        int32_t pos;
        int32_t vel;
        uint32_t torque;
    } m_reported{};

    bool m_moving = false;
    bool m_updated = false;
    int m_dir = 0;

    // Initialize PDO data
    rxpdo_t m_rxpdo{};
    txpdo_t m_txpdo{};

    int m_state = -1;
    bool m_state_changed = false;
    int m_powerstate = 0;
};


