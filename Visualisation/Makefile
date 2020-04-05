CC=c++
CFLAGS=-std=c++11

single_lane: single_lane.o
	$(CC) $(CFLAGS) -o single_lane single_lane.o
single_lane.o: ./src/Single_Lane/single_lane.cc ./src/Single_Lane/Matrix2D.hpp
	$(CC) $(CFLAGS) -c ./src/Single_Lane/single_lane.cc

clean:
	rm *.o single_lane

clean_o:
	rm *.o
