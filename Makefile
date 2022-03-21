# the compiler: gcc for C program, define as g++ for C++
CC = g++

# compiler flags:
#  -g     - this flag adds debugging information to the executable file
#  -Wall  - this flag is used to turn on most compiler warnings
CFLAGS  = -g -Wall -O3

# The build target 
TARGET = DarpSolver

.PHONY: all clean

default: all clean

all: Darp

Darp: Darp.o
	$(CC) $(CFLAGS) -o $(TARGET) Darp.o

Darp.o: Darp.cpp
	$(CC) $(CFLAGS) -c Darp.cpp

clean:
	rm *.o
