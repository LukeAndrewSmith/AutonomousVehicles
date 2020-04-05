CC=c++
CFLAGS=-std=c++11

E1=./src/E1_Concertina
MODEL=./src/Model
MODEL_O= experiment.o road.o car.o
#
# concertina: concertina.o $(MODEL_O)
# 	$(CC) $(CFLAGS) -o concertina concertina.o $(MODEL_O)
# concertina.o: $(E1)/concertina.cc $(MODEL_O)
# 	$(CC) $(CFLAGS) -c $(E1)/concertina.cc -I ./src

# model.o: ./src/Model/model.hpp ./src/Model/model.cc
# 	$(CC) $(CFLAGS) -c ./src/Model/model.cc

experiment.o: $(MODEL)/experiment.hpp $(MODEL)/experiment.cc
	$(CC) $(CFLAGS) -c $(MODEL)/experiment.cc
# road.o: $(MODEL)/road.hpp $(MODEL)/road.cc
# 	$(CC) $(CFLAGS) -c $(MODEL)/road.cc
# car.o: $(MODEL)/car.hpp $(MODEL)/car.cc
# 	$(CC) $(CFLAGS) -c $(MODEL)/car.cc


clean:
	rm *.o concertina

clean_o:
	rm *.o
