LIBS = -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpthread -lc
INCLUDE_DIRS = -I/usr/include/opencv4
LIB_DIRS =
CC = g++
CDEFS =
CFLAGS = -O0 -g $(INCLUDE_DIRS) $(CDEFS)
LDFLAGS =
SRCS = capture.c
OBJS = $(SRCS:.c=.o)

all: capt

clean:
	rm -f *.o capt

capt: $(OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ `pkg-config --libs opencv4` $(LIBS)

depend:

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

