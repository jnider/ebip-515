# 2020-11-02 J.Nider
# apt-get install libsdl2-dev

CPP_SRC = main.cpp simulation.cpp robot.cpp
CPP_OBJS = $(CPP_SRC:%.cpp=%.o)
OBJS = $(CPP_OBJS)

INCLUDE_PATH = 
LIBRARY_PATH = 

CFLAGS = -O2

ifeq ($(DEBUG), 1)
CFLAGS+=-DDEBUG
endif

.PRECIOUS: *.o

.PHONY: tags

all: sim

sim: $(CPP_SRC)
	g++ $(CFLAGS) $(CPP_SRC) $(INCLUDE_PATH) $(LIBRARY_PATH) -lSDL2 -o sim

clean:
	rm -rf $(OBJS) sim

tags:
	ctags -R -f tags . /usr/local/include
