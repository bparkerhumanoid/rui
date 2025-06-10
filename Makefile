
#

INCS = -I$(HOME)/git/SOEM/soem -I$(HOME)/git/SOEM/osal -I$(HOME)/git/SOEM/osal/linux -I$(HOME)/git/SOEM/oshw/linux -I.

HDR = log.hpp motor.hpp move.hpp rui.hpp threads.hpp window.hpp
SRC = rui.cpp threads.cpp move.cpp log.cpp window.cpp

rui: $(SRC) $(HDR)
	g++ -g -O2 -o rui $(INCS) $(SRC) -lncurses -L$(HOME)/git/SOEM/build -lsoem

clean:
	rm -f rui
