//
//  lane_merge.cc
//  Autonomous-Vehicles
//
//  Created by Luke Smith on 08.04.20.
//  Copyright © 2020 Luke Smith. All rights reserved.
//

// HELLO FILE SYSTEM
#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include "model.hpp"
#include "write_data.hpp"


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
    if ( ( id == 0 ) && (x_pos > road->get_road_delim_x()/2) && (x_pos < (road->get_road_delim_x()/2)+30 ) ) { // (x_pos == road->get_road_delim_x()/2) ) {
        accel_param = min_accel_param;
    } else if ( diff < safety_distance ) { // TODO: Smoother braking
        accel_param = -velocity/diff;
        accel_param = std::max(min_accel_param,accel_param);
        // accel_param = (1-((2*diff)/safety_distance))*min_accel_param; // TEST 1: reduce by proportion of min_accel_param
        // accel_param = std::max(min_accel_param,accel_param);

    } else { // TODO: Implement Accelerating to max road speed: new argument max_road_speed
        accel_param = (1-(velocity/max_velocity))*max_accel_param;
//        accel_param = 0;

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
      std::vector<float>(20,30), //{33,32,31,30,29,28,27,25,24,23,22,21,20,22,22},  // std::vector<float> init_velocity; // TODO: Won't work for multi-lane
    33,  // float init_max_velocity;
    0,   // float init_accel_param;
    0.14,// float init_max_accel_param;
    -0.69,// float init_min_accel_param;
    4.69,// float init_length;
    100, // float horizon;
    1,   // float init_max_delta_turning_angle;
    // Road
    10000,                  // float init_road_delim_x;
    1, // int init_n_lanes;
    1,                   // int init_inlet_protocol; // 0 = none, 1 = loop, 2 = new cars
    // Experiment
    std::vector<int>{20}, // std::vector<int> init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
    10,                   // int init_spacing_protocol; // TODO: Rename? Currently used as distance between cars
    10000,                // int init_max_it;
    0.01,                 // float init_time_step;
  };

  Experiment concertina(init_vals);
  std::string dir = "./Experiments/concertina";
  std::string experiment_name = get_file_name(dir);
  std::string experiment_file_name = dir + "/" + experiment_name;
  concertina.main_loop(experiment_file_name, false);

  return 0;
}
