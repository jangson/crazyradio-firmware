CFLAGS += $(shell pkg-config libftdi1 --cflags)
LDFLAGS = $(shell pkg-config libftdi1 --libs)

all: nrfProg

nrfProg: nrfProg.o nrfSpi.o commands.o
	cc -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o nrfProg *~
