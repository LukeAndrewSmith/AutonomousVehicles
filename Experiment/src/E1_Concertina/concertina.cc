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
void Car::update_decision(Road* road) {
  float car_ahead_pos = road->get_car_ahead_pos(x_pos);
  float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
  float diff = car_ahead_pos-x_pos;
  if ( (diff) < (safety_distance) ) { // TODO: Implement Accelerating to max road speed: new argument max_road_speed
    accel_param = -velocity/diff;
    accel_param = std::max(min_accel_param,accel_param);

    // accel_param = (1-((car_ahead_pos-x_pos)/safety_distance))*min_accel_param; // TEST 1: reduce by proportion of min_accel_param
    // accel_param = std::max(min_accel_param,accel_param);

  } else {
    accel_param = (1-(velocity/max_velocity))*max_accel_param;
    // std::cout << accel_param << std::endl;
    // accel_param = 0;
  }
}

// CAN HAVE MULTIPLE EXPERIMENTS IN THE THE EXPERIMENTS FILE SO THAT YOU CAN
// CHECK PREVIOUS EXPERIMENTS
// IF DEFS CAN BE USEFUL FOR 'LOGGING MODE' WHERE PRINT STATEMENTS ARE ENCAPSULATED

// NOTE FOR FINAL MERGING EXPRIMENT:
  // ATTEMPT WITH ALL CARS FROM STOP START
  // EASIER TO VISUALISE FROM STAND STILL

int main(int argc, char const *argv[]) {

  init_experiment init_vals = { // See model.hpp for definition
    // Car (tesla model 3)
    std::vector<float> {30,0},  // std::vector<float> init_velocity; // TODO: Won't work for multi-lane
    35,  // float init_max_velocity;
    0,   // float init_accel_param;
    0.14,// float init_max_accel_param;
    0.69,// float init_min_accel_param;
    4.69,// float init_length;
    100, // float horizon;
    1,   // float init_max_delta_turning_angle;
    // Road
    1000,                  // float init_road_delim_x;
    std::vector<float>{0}, // std::vector<float> init_lanes;
    // Experiment
    std::vector<int>{2}, // std::vector<int> init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
    0,                   // int init_spacing_protocol;
    0,                   // int init_refill_protocol;
    1000,                // int init_max_it;
    0.01,                 // float init_time_step;
  };

  Experiment concertina(init_vals);
  concertina.main_loop("c_1",false);

  return 0;
}
