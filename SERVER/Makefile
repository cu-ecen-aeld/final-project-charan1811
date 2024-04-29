LIBS = -lrt -lpthread -lc
LIB_DIRS =
CC = g++
CDEFS =
CFLAGS = -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LDFLAGS =
SRCS = capture_new.c
OBJS = $(SRCS:.c=.o)

all: capt

clean:
	rm -f *.o capt

capt: $(OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

depend:

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

