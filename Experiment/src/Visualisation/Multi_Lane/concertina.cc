#include <iostream>
#include <tuple>
#include <vector>
#include "Model/model.hpp"

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
void Car::update_decision() {
  // TODO: Implement
  // std::cout << "Updating decision";
}

// CAN HAVE MULTIPLE EXPERIMENTS IN THE THE EXPERIMENTS FILE SO THAT YOU CAN
// CHEK PREVIOUS EXPERIMENTS
// IF DEFS CAN BE USEFUL FOR 'LOGGING MODE' WHERE PRINT STATEMENTS ARE ENCAPSULATED

// NOTE FOR FINAL MERGING EXPRIMENT:
  // ATTEMPT WITH ALL CARS FROM STOP START
  // EASIER TO VISUALISE FROM STAND STILL

int main(int argc, char const *argv[]) {

  init_experiment init_vals = {
    // Car (tesla model 3)
    100, // float init_velocity;
    272, // float init_max_velocity;
    0,   // float init_accel_param;
    4.69,// float init_length;
    100, // float horizon;
    1,   // float init_max_delta_turning_angle;
    // Road
    1000,                  // float init_road_delim_x;
    std::vector<float>{0}, // std::vector<float> init_lanes;
    // Experiment
    std::vector<int>{1}, // std::vector<int> init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
    0,                   // int init_spacing_protocol;
    0,                   // int init_refill_protocol;
    1000,                // int init_max_it;
    0.01,                 // float init_time_step;
  };

  Experiment concertina(init_vals);
  concertina.main_loop("c_1",false);

  return 0;
}
