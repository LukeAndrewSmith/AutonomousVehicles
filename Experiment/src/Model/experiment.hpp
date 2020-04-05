/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/*
Experiment init
*/

#ifndef EXPERIMENT_H_INCLUDED
#define EXPERIMENT_H_INCLUDED

#include <vector>
class Road;

struct init_experiment {
  // Car
  std::vector<float> init_velocity;
  float init_max_velocity;
  float init_accel_param;
  float init_max_accel_param;
  float init_min_accel_param;
  float init_length;
  float horizon;
  float init_max_delta_turning_angle;
  // Road
  float init_road_delim_x;
  std::vector<float> init_lanes;
  // Experiment
    // Initialise cars and indicate refill protocol (how cars are input into the system)
  std::vector<int> init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
  int init_spacing_protocol;
  int init_refill_protocol;
  int init_max_it;
  float init_time_step;
};
/*
Explanation of Experiment
*/
class Experiment {
private:
  Road road;
  std::vector<int> n_cars_per_lane;
  int spacing_protocol; // Initial position of the cars (densly packed or not)
  int refill_protocol; // How to intput INLET_PROTOCOL
  int max_it; // Number of iterations
  float time_step; // Time passed in each iteration (should be small as we have made small angle approximations)

public:
  Experiment(init_experiment init_vals);
  void main_loop(std::string experiment_num, bool multi_lane);
  void write_to_file(std::vector<float> cars);
};

#endif
