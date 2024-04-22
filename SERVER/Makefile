#

#CC = gcc
#CFLAGS = -Wall -O2
#LDFLAGS = -lm -lpthread
#LIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpthread -lc
#
#TARGET = capture
#SRC = capture.c
#
#all: $(TARGET)
#
#$(TARGET): $(SRC)
#	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)
#
#clean:
#	rm -f $(TARGET)
#
LIBS = -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video -lrt -lpthread -lc
INCLUDE_DIRS = -I/usr/include/opencv4
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
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ `pkg-config --libs opencv4` $(LIBS)

depend:

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

