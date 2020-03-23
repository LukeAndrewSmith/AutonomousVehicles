CC=c++
CFLAGS=-std=c++11

concertina: concertina.o model.o
	$(CC) $(CFLAGS) -o concertina concertina.o model.o
concertina.o: ./src/E1_Concertina/concertina.cc model.o
	$(CC) $(CFLAGS) -c ./src/E1_Concertina/concertina.cc -I ./src

model.o: ./src/Model/model.hpp ./src/Model/model.cc
	$(CC) $(CFLAGS) -c ./src/Model/model.cc

clean:
	rm *.o concertina

clean_o:
	rm *.o
