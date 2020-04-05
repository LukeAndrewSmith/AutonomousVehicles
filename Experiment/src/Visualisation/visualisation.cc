#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "visualisation.hpp"

/*----------------------------------------------------------------------------*/
/*                             Single Lane                                    */
/*----------------------------------------------------------------------------*/
void write_space_time_Pbm(const std::vector<std::vector<std::vector<int>>> &data, std::string filename){
    std::ofstream file;
    file.open (filename+".pbm");
    file << "P1" << std::endl;
    file << d.get_n_cols() << " " << d.get_n_rows() << std::endl;
    for(int i=0; i<d.get_n_rows(); i++){
        for(int j=0; j<d.get_n_cols(); j++){
            file << d(i,j) << " ";
        }
        file << std::endl;
    }
    file.close();
}

void write_space_time_Pbm_from_data_file(std::string input_filename, std::string output_filename) {

  std::string line;
  std::ifstream data_file;
  data_file.open(input_filename);

  // Initialise array for space time diagram
  std::getline(data_file,line);
  std::istringstream ss_vals(line);
  int road_delim_x; // Length of road: will be number of columns of array
  int max_it; // Number of iterations: will be number of rows of array
  ss_vals >> road_delim_x;
  ss_vals >> max_it;
  Matrix2D<int> space_time_diagram(max_it, road_delim_x, 0);
  int i = 0;
  while ( std::getline (data_file,line) ) {
    std::istringstream ss(line); // Separate string by spaces
    do {
        float x_pos_float;
        ss >> x_pos_float;
        int x_pos = std::min(road_delim_x-1,static_cast<int>(x_pos_float)); // Make sure the x_pos is in bounds
        space_time_diagram(i,x_pos) = 1;
    } while (ss);
    i++;
  }
  data_file.close();
  write_space_time_Pbm(space_time_diagram,output_filename);
}


/*----------------------------------------------------------------------------*/
/*                              Multi Lane                                    */
/*----------------------------------------------------------------------------*/
void write_lane_changing_gif(const std::vector<std::vector<std::vector<int>>> &data, std::string filename){
  std::cout << "TODO: Implement write_lane_changing_gif"
};

void write_lane_changing_gif_from_data_file(std::string input_filename, std::string output_filename){
  std::cout << "TODO: Implement write_lane_changing_gif"
};
