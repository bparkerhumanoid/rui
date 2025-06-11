#pragma once

#include <vector>

#include <curses.h>

static bool curses_active = false;

typedef enum FieldNum {
    MF_STATUS = 1,
    MF_REQ_POS,
    MF_REQ_VELOCITY,
    MF_REQ_TORQUE,

    MF_REP_STATUS,
    MF_REP_POS,
    MF_REP_VELOCITY,
    MF_REP_TORQUE,

    MF_ECAT_STATE,
    MF_INDEX,
    MF_CYCLES,
    MF_MAX,
} FieldNum;

class Wfield {
public:
    Wfield(int l, int c, const char *name) {
        m_pos.line = l;
        m_pos.col = c;
        m_name = name;
    }

    Wfield() {}

    typedef enum FieldSize {
        FS_S32 = 1,
        FS_U32,
        FS_S16,
        FS_U16,
        FS_U8,
    } FieldSize;
    
    int init(int l, int c, const char *name) {
        m_pos.line = l;
        m_pos.col = c;
        m_name = name;
        return 0;
    }
    
    int set_value_s32(int32_t v) {
        m_value.s32 = v;
        m_fieldsize = FS_S32;
        return 0;
    }

    int set_value_u32(uint32_t v) {
        m_value.u32 = v;
        m_fieldsize = FS_U32;
        return 0;
    }

    int set_value_u16(uint16_t v) {
        m_value.u16 = v;
        m_fieldsize = FS_U16;
        return 0;
    }

    int set_value_s16(int16_t v) {
        m_value.s16 = v;
        m_fieldsize = FS_S16;
        return 0;
    }

    int set_value_u8(uint8_t v) {
        m_value.u8 = v;
        m_fieldsize = FS_U8;
        return 0;
    }

    void display_field() {
        int l = m_pos.line;
        int c1 = m_pos.col;
        int c2 = m_pos.col + 10;
        
        mvwprintw(m_win, l, c1, "%s", m_name);

        switch (m_fieldsize) {
        case FS_S32:
            mvwprintw(m_win, l, c2, "%-10d", m_value.s32);
            break;
        case FS_U32:
            mvwprintw(m_win, l, c2, "%-10u", m_value.u32);
            break;
        case FS_S16:
            mvwprintw(m_win, l, c2, "%-6d", m_value.u16);
            break;
        case FS_U16:
            mvwprintw(m_win, l, c2, "%04x", m_value.u16);
            break;
        case FS_U8:
            mvwprintw(m_win, l, c2, "%02x", m_value.u8);
            break;
        }
    }

    void set_window(WINDOW *win) { m_win = win; }
    
    Wfield *m_next_changed = nullptr;

private:
    struct {
        int line;
        int col;
    } m_pos{};

    union {
        int32_t s32;
        uint32_t u32;
        uint16_t u16;
        int16_t s16;
        uint8_t u8;
    } m_value{};
    
    bool m_changed = false;
    const char *m_name;

    FieldSize m_fieldsize;
    WINDOW *m_win;
};

class Wwindow {
public:
    Wwindow() {
    }

    Wwindow(int l, int c, int h, int w, bool boarder) {
        m_pos.line = l;
        m_pos.col = c;
        m_size.height = h;
        m_size.width = w;
        m_boarder = boarder;
    }

    ~Wwindow() {
        if (m_win && curses_active) {
            unshow();
        }
    }

    void set_pos_size(int l, int c, int h, int w) {
        m_pos.line = l;
        m_pos.col = c;
        m_size.height = h;
        m_size.width = w;
    }

    void show() {
        create_newwin();
        m_fields.resize(MF_MAX);
    }

    void scrolling() {
        if (m_win) {
            scrollok(m_win, TRUE);
        }
    }
    
    void unshow() {
        if (m_win) {
            wborder(m_win, ' ', ' ', ' ',' ',' ',' ',' ',' ');
            wrefresh(m_win);
            delwin(m_win);
            m_win = nullptr;
        }
    }

    void refresh() {
        wrefresh(m_win);
    }
    
    WINDOW *window() { return m_win; }

    int add_field(FieldNum fn, int l, int c, const char *name) {
        m_fields[fn].init(l, c, name);
        m_fields[fn].set_window(m_win);
        return 0;
    }

    int set_field_value_s32(FieldNum fn, int32_t v) {
        add_to_update_list(&m_fields[fn]);
        return m_fields[fn].set_value_s32(v);
    }
    int set_field_value_u32(FieldNum fn, uint32_t v) {
        add_to_update_list(&m_fields[fn]);
        return m_fields[fn].set_value_u32(v);
    }
    int set_field_value_s16(FieldNum fn, int16_t v) {
        add_to_update_list(&m_fields[fn]);
        return m_fields[fn].set_value_s16(v);
    }
    int set_field_value_u16(FieldNum fn, uint16_t v) {
        add_to_update_list(&m_fields[fn]);
        return m_fields[fn].set_value_u16(v);
    }
    int set_field_value_u8(FieldNum fn, uint8_t v) {
        add_to_update_list(&m_fields[fn]);
        return m_fields[fn].set_value_u8(v);
    }

    void display_update_list() {
        bool updated = false;
        if (m_field_changed_list) {
            for (Wfield *f = m_field_changed_list; f; ) {
                f->display_field();
                updated = true;
                Wfield *f_next = f->m_next_changed;
                f->m_next_changed = nullptr;
                f = f_next;
            }
        }
        m_field_changed_list = nullptr;

        if (updated) {
            wrefresh(m_win);
        }
    }
    
private:
    WINDOW *m_win{};
    std::vector<Wfield> m_fields;
    Wfield *m_field_changed_list = nullptr;
    
    void create_newwin() {
        m_win = newwin(m_size.height, m_size.width, m_pos.line, m_pos.col);

        if (!m_boarder) {
            wborder(m_win, ' ', ' ', ' ',' ',' ',' ',' ',' ');
        } else {
            // 0, 0 gives default characters for the vertical and horizontal lines
            box(m_win, 0, 0);
        }

        keypad(m_win, TRUE);
        
        // Show the box
        wrefresh(m_win);
    }

    void add_to_update_list(Wfield *f) {
        if (f->m_next_changed == nullptr) {
            f->m_next_changed = m_field_changed_list;
            m_field_changed_list = f;
        }
    }

    struct {
        int height;
        int width;
    } m_size = {0,0};
    
    struct {
        int line;
        int col;
    } m_pos;

    bool m_boarder = true;
};

void set_curses_active();
void set_curses_inactive();
