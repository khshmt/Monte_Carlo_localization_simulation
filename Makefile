CC = g++
CFLAGS = -g -Wall -std=c++11
SRCS = main.cpp
PROG = app

all:
	@$(CC) $(CFLAGS) -o $(PROG) $(SRCS) -I/usr/include/python2.7 -lpython2.7

clean:
	@rm ${PROG}
	@rm ./Images/*