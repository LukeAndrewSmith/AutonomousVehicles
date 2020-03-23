/*
Luke Andrew Smith

Explanation of file
*/
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string>
#include "model.hpp"

/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/* Constructor */
Car::Car(float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_length, float horizon, float init_max_delta_turning_angle) {
  velocity = init_velocity;
  max_velocity = init_max_velocity;
  x_pos = init_x_pos;
  y_pos = init_y_pos;
  accel_param = init_accel_param;
  length = init_length;
  y_target = init_y_pos; // We want to stay in the same lane at the beginning
  road_angle = 0;
  turning_angle = 0;
  max_delta_turning_angle = init_max_delta_turning_angle;
  indicators = std::vector<bool>{false, false};
}

/*
update_position
TODO: Explanation
*/
void Car::update_position(float time_step, float road_delim_x) { // UPDATE_MOVEMENT
  // TODO: Check if car is still in road, or input them back into the system if they off
  // the road depending on the refill protocol
  if ( x_pos <= road_delim_x ) { // For now do nothing upon car leaving road
    if (road_angle == 0 && turning_angle == 0) {
      if (accel_param == 0) {
        x_pos = x_pos + (velocity*time_step); // Velocity/y_pos unchanged, x_pos updated
      } else if (accel_param > 0) {
        x_pos = x_pos + ( (velocity/accel_param)*(1 - exp(accel_param*time_step)) );
        velocity = velocity*exp(accel_param*time_step);
      } else {
        x_pos = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(accel_param*time_step)) );
      }
    } else {
      road_angle = road_angle + (((velocity*time_step)/length)*sin(turning_angle));
      x_pos = x_pos + (velocity*time_step*cos(road_angle)*cos(turning_angle));
      y_pos = y_pos + (velocity*time_step*cos(road_angle)*sin(turning_angle));
      float psi = atan((y_target-y_pos)/(horizon - (length*cos(road_angle))) - road_angle);
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
}

/*
TODO: Explanation
*/
float Car::get_x_pos() { // UPDATE_MOVEMENT
  return x_pos;
}

/*
TODO: Explanation
*/
float Car::get_y_pos() { // UPDATE_MOVEMENT
  return y_pos;
}

/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
/* Default Constructor */
  // Included so that Experiment Constructor can initialise a random Road and we can then override this initialisation
  // Experiment cannot begin constructor without all it's members initialised so calls the default constructor on all members
Road::Road() {
  road_delim_x = 100;
  lanes = std::vector<float>{0};
}

/* Constructor */
Road::Road(float init_road_delim_x, std::vector<float> init_lanes) {
  road_delim_x = init_road_delim_x;
  lanes = init_lanes;
}

/*
*/
void Road::update_car_decisions() {
  for (Car &c : cars) {
    c.update_decision();
  }
}

/*
*/
void Road::update_car_positions(float time_step) {
  for (Car &c : cars) {
    c.update_position(time_step, road_delim_x);
  }
}

/*
*/
void Road::add_car(Car new_car) {
  cars.push_back(new_car);
}

/*
*/
std::vector<Car> Road::get_cars() {
  return cars;
}

/*
*/
float Road::get_road_delim_x() {
  return road_delim_x;
}

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
      Car new_car = Car(init_vals.init_velocity, init_vals.init_max_velocity, x_pos, float(i), init_vals.init_accel_param, init_vals.init_length, init_vals.horizon, init_vals.init_max_delta_turning_angle);
      x_pos = x_pos + 10;
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
  data_file << road.get_road_delim_x() << " " << std::endl; // Indicate experiment conditions for the program that visualises the data

  for(int i=0; i<=max_it; i++) {
    road.update_car_decisions();
    road.update_car_positions(time_step);
    for (Car &c : road.get_cars()) { // Write to file
      if ( multi_lane ) {
        data_file << c.get_x_pos() << " " << c.get_y_pos();
      } else { // Only store x position if single lange experiment
        data_file << c.get_x_pos();
      }
    }
    data_file << std::endl;
  }
  data_file.close();
}
