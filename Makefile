# 2020-11-02 J.Nider
# apt-get install libsdl2-dev libsdl2-gfx-dev  libsdl2_ttf

CPP_SRC = main.cpp simulation.cpp mysim.cpp interaction.cpp bip.cpp
CPP_OBJS = $(CPP_SRC:%.cpp=%.o)
OBJS = $(CPP_OBJS)

INCLUDE_PATH = 
LIBRARY_PATH = 
DEBUG=1
VERSION=3

CFLAGS = -O2 -DVERSION=$(VERSION)
LIBRARIES = -lSDL2 -lSDL2_gfx -lSDL2_ttf -lopenblas -llapacke64

ifeq ($(DEBUG), 1)
CFLAGS+=-DDEBUG
endif

.PRECIOUS: *.o

.PHONY: tags

all: sim

sim: $(CPP_SRC)
	g++ $(CFLAGS) $(CPP_SRC) $(INCLUDE_PATH) $(LIBRARY_PATH) $(LIBRARIES) -o sim

example:
	gcc $(CFLAGS) example.c $(INCLUDE_PATH) $(LIBRARY_PATH) $(LIBRARIES) -o example

clean:
	rm -rf $(OBJS) sim

tags:
	ctags -R -f tags . /usr/local/include /usr/include/x86_64-linux-gnu
