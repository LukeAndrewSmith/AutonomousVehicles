CC=c++
CFLAGS=-std=c++11

E1=./src/E1_Concertina
MODEL=./src/Model
DATA=./src/Data_Storage
VIS=./src/Visualisation

concertina: concertina.o model.o
	$(CC) $(CFLAGS) -o concertina concertina.o model.o
concertina.o: $(E1)/concertina.cc model.o
	$(CC) $(CFLAGS) -c $(E1)/concertina.cc -I ./src

model.o: $(MODEL)/model.hpp $(MODEL)/model.cc $(DATA)/write_data.hpp $(VIS)/visualisation.hpp
	$(CC) $(CFLAGS) -c $(MODEL)/model.cc -I $(DATA)

clean:
	rm *.o concertina

clean_o:
	rm *.o
