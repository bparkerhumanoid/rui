
int startup(char *ifname);
int shutdown();
int init_ecat();
int run();
int run_loop(unsigned count);
int run_input(int ch);
void run_set_outputs();

uint32_t slave_requested_pos(int slave);
void slave_set_moving(int slave, int yesno);
int slave_moving(int slave);




