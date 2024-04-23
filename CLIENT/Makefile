CC=g++
CFLAGS?=-g -Wall -Wextra
LDFLAGS?= -lwiringPi
.PHONY: all
all: client

client : 
	g++ client.c -o client `pkg-config opencv4 --libs --cflags` -lwiringPi


.PHONY: clean
clean:
	rm -f *.o client
