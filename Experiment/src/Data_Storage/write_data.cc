#include <iostream>
#include <fstream>
#include "write_data.hpp"

void write_data(std::vector<std::vector<std::vector<int>>> const &data, int max_it, int road_length, std::string filename) {
  std::ofstream data_file;
  std::string file_name = filename + ".txt";
  data_file.open(file_name);

  data_file << road_length << " " << max_it << std::endl; // Indicate experiment conditions for the program that visualises the data

  for(auto iteration : data) {
    for (auto car_pos : iteration) {
      for (int pos : car_pos) {
        data_file << pos << " ";
      }
      data_file << std::endl;
    }
  }
  // for(std::vector<std::vector<int>>& iteration : data) {
  //   for (std::vector<int>& car_pos : iteration) {
  //     for (int pos : car_pos) {
  //       data_file << pos << " ";
  //     }
  //     data_file << std::endl;
  //   }
  // }

  data_file.close();
}
