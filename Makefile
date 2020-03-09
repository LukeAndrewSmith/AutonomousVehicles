CC=g++
CFLAGS=-std=c++11

all:
	$(CC) $(CFLAGS) ./src/heat_thread.cc -o laplace_threads

clean:
	rm laplace_threads
