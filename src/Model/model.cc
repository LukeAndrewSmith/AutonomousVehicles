/*
Luke Andrew Smith

Explanation of file
*/
#include <iostream>
#include <vector>
#include <math.h>
#include "model.hpp"

/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/* Constructor */
Car::Car(uint8_t init_time, float init_velocity, float init_max_velocity, std::vector<float> init_pos, float init_accel_param, float init_length, float horizon, float init_max_delta_turning_angle) {
  time = init_time;
  velocity = init_velocity;
  max_velocity = init_max_velocity;
  pos = init_pos;
  accel_param = init_accel_param;
  length = init_length;
  y_target = init_pos[1]; // We want to stay in the same lane at the beginning
  road_angle = 0;
  turning_angle = 0;
  max_delta_turning_angle = init_max_delta_turning_angle;
  indicators = std::vector<bool>{false, false};
}

/*
update_position
TODO: Explanation
*/
void Car::update_position(float time_step) {
  if (road_angle == 0 && turning_angle == 0) {
    if (accel_param == 0) {
      pos[0] = pos[0] + (velocity*time_step); // Velocity/y_pos unchanged, x_pos updated
    } else if (accel_param > 0) {
      pos[0] = pos[0] + ( (velocity/accel_param)*(1 - exp(accel_param*time_step)) );
      velocity = velocity*exp(accel_param*time_step);
    } else {
      pos[0] = pos[0] + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(accel_param*time_step)) );
    }
  } else {
    road_angle = road_angle + (((velocity*time_step)/length)*sin(turning_angle));
    pos[0] = pos[0] + (velocity*time_step*cos(road_angle)*cos(turning_angle));
    pos[1] = pos[1] + (velocity*time_step*cos(road_angle)*sin(turning_angle));
    float psi = atan((y_target-pos[1])/(horizon - (length*cos(road_angle))) - road_angle);
    float check = psi-turning_angle;
    if (check<-max_delta_turning_angle) {
      turning_angle = turning_angle - max_delta_turning_angle;
    } else if (abs(check)<max_delta_turning_angle) {
      turning_angle = psi;
    } else if (check>max_delta_turning_angle) {
      turning_angle = turning_angle + max_delta_turning_angle;
    }
  }
}

/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
/*
Road
  Explanation of road
*/
Road::Road(std::vector<float> init_road_delim_x, std::vector<float> init_lanes) {
  road_delim_x = init_road_delim_x;
  lanes = init_lanes;
}

/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/*
*/
Experiment::Experiment(init_experiment init_vals) {

}
/*
*/
void Experiment::main_loop() {
  for(int i=0; i<=max_it; i++) {
    for (Car &c : cars) {
      c.update_decision();
    }
    for (Car &c : cars) {
      c.update_position(time_step);
    }
  }
}
