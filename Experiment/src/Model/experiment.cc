// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <math.h>
// #include <string>
// // #include <limits>
// #include <cfloat>
#include "experiment.hpp"

/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/* Constructor */
Experiment::Experiment(init_experiment init_vals) {
  // Road
  road = Road(init_vals.init_road_delim_x, init_vals.init_lanes);
  // Experiment
  n_cars_per_lane = init_vals.init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
  spacing_protocol = init_vals.init_spacing_protocol;
  refill_protocol = init_vals.init_refill_protocol;
  max_it = init_vals.init_max_it;
  time_step = init_vals.init_time_step;
  // Cars
    // TODO: spacing_protocol, refill_protocol
  float x_pos = 0;
  for (int i=0; i<n_cars_per_lane.size(); i++) {
    x_pos = 0;
    for (int j=0; j<n_cars_per_lane[i]; j++) {
      // float init_velocity, float init_max_velocity, std::vector<float> init_pos, float init_accel_param, float init_length, float horizon, float init_max_delta_turning_angle
      Car new_car = Car(init_vals.init_velocity[j], init_vals.init_max_velocity, x_pos, float(i), init_vals.init_accel_param, init_vals.init_max_accel_param, init_vals.init_min_accel_param, init_vals.init_length, init_vals.horizon, init_vals.init_max_delta_turning_angle);
      x_pos = x_pos + 300;
      road.add_car(new_car);
    }
  }
}

/*
*/
void Experiment::main_loop(std::string experiment_num, bool multi_lane) {
  std::ofstream data_file;
  std::string file_name = "./Experiments/experiment" + experiment_num + ".txt";
  data_file.open(file_name);
  data_file << road.get_road_delim_x() << " " << max_it << std::endl; // Indicate experiment conditions for the program that visualises the data

  for(int i=0; i<max_it; i++) {
    for (Car &c : road.get_cars()) { // Write to file
      if ( multi_lane ) {
        data_file << c.get_x_pos() << " " << c.get_y_pos() << " ";
      } else { // Only store x position if single lange experiment
        data_file << c.get_x_pos() << " ";
      }
    }
    data_file << std::endl;
    road.update_car_decisions();  // Update after so that we include the initial positions
    road.update_car_positions(time_step);
  }
  data_file.close();
}
