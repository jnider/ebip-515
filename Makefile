# 2020-11-02 J.Nider
# apt-get install libsdl2-dev libsdl2-gfx-dev  libsdl2_ttf

CPP_SRC = main.cpp simulation.cpp mysim.cpp
CPP_OBJS = $(CPP_SRC:%.cpp=%.o)
OBJS = $(CPP_OBJS)

INCLUDE_PATH = 
LIBRARY_PATH = 
DEBUG=1
VERSION=2

CFLAGS = -O2 -DVERSION=$(VERSION)
LIBRARIES = -lSDL2 -lSDL2_gfx -lSDL2_ttf

ifeq ($(DEBUG), 1)
CFLAGS+=-DDEBUG
endif

.PRECIOUS: *.o

.PHONY: tags

all: sim

sim: $(CPP_SRC)
	g++ $(CFLAGS) $(CPP_SRC) $(INCLUDE_PATH) $(LIBRARY_PATH) $(LIBRARIES) -o sim

clean:
	rm -rf $(OBJS) sim

tags:
	ctags -R -f tags . /usr/local/include
