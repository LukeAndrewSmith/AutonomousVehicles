CC=c++
CFLAGS=-std=c++11

E1=./src/E1_Concertina
MODEL=./src/Model
DATA=./src/Data_Storage
VIS=./src/Visualisation

concertina: concertina.o model.o write_data.o visualisation.o
	$(CC) $(CFLAGS) -o concertina concertina.o model.o write_data.o visualisation.o
concertina.o: $(E1)/concertina.cc model.o
	$(CC) $(CFLAGS) -c $(E1)/concertina.cc -I $(MODEL) -I $(DATA)

model.o: $(MODEL)/model.hpp $(MODEL)/model.cc $(DATA)/write_data.hpp $(VIS)/visualisation.hpp
	$(CC) $(CFLAGS) -c $(MODEL)/model.cc -I $(DATA) -I $(VIS)

write_data.o: $(DATA)/write_data.cc $(DATA)/write_data.hpp
	$(CC) $(CFLAGS) -c $(DATA)/write_data.cc

 visualisation.o: $(VIS)/visualisation.cc $(VIS)/visualisation.hpp
	$(CC) $(CFLAGS) -c $(VIS)/visualisation.cc

clean:
	rm *.o concertina

clean_o:
	rm *.o
