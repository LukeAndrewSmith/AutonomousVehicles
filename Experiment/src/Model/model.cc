#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string>
// #include <limits>
#include <cfloat>
#include "model.hpp"
#include "write_data.hpp"
// #include "visualisation.hpp"

/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/* Constructor */
Car::Car(float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_length, float init_horizon, float init_max_delta_turning_angle) {
  velocity = init_velocity;
  max_velocity = init_max_velocity;
  x_pos = init_x_pos;
  y_pos = init_y_pos;
  accel_param = init_accel_param;
  max_accel_param = init_max_accel_param;
  min_accel_param = init_min_accel_param;
  length = init_length;
  y_target = init_y_pos; // We want to stay in the same lane at the beginning
  road_angle = 0;
  turning_angle = 0;
  horizon = init_horizon;
  max_delta_turning_angle = init_max_delta_turning_angle;
  indicators = std::vector<bool>{false, false};
}

/*
update_position
TODO: Explanation
*/
void Car::update_position(float time_step, int road_delim_x) { // UPDATE_MOVEMENT
  // TODO: Check if car is still in road, or input them back into the system if they off
  // the road depending on the refill protocol
  if ( x_pos < road_delim_x ) { // For now do nothing upon car leaving road
    if (road_angle == 0 && turning_angle == 0) {
      if ( accel_param == 0 ) {
        x_pos = x_pos + (velocity*time_step); // Velocity/y_pos unchanged, x_pos updated
      } else if ( accel_param < 0 ) {
        x_pos = x_pos + ( (velocity/-accel_param)*(1 - exp(accel_param*time_step)) );
        velocity = velocity*exp(accel_param*time_step);
      } else if ( accel_param > 0 ) {
        x_pos = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(accel_param*time_step)) );
        velocity = max_velocity + (velocity-max_velocity)*exp(-accel_param*time_step);
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
    x_pos = fmin(x_pos,road_delim_x); // Ensure on road so that visualisation doesn't out_of_range when trying to access a position on the position_time diagram
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
float Road::get_car_ahead_pos(float my_x_pos) {
  float car_ahead_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
  for (Car c : cars) {
    if (c.get_x_pos() > my_x_pos) { // Cars ahead
      if ( (c.get_x_pos() - my_x_pos) < (car_ahead_x_pos - my_x_pos) ) { // Update x_pos if closer
        car_ahead_x_pos = c.get_x_pos();
      }
    }
  }
  return car_ahead_x_pos;
}

/*
*/
float Road::get_car_behind_pos(float my_x_pos) {
  float x_pos = FLT_MAX; // std::numeric_limits<float>::max;
  for (Car c : cars) {
    if (c.get_x_pos() < my_x_pos) { // Cars behind
      if ( (my_x_pos - c.get_x_pos()) < (my_x_pos - x_pos) ) { // Update x_pos if closer
        x_pos = c.get_x_pos();
      }
    }
  }
  return x_pos;
}

// /*
// */
// std::vector<float> Road::get_car_ahead_pos_otherlane() {
//   return std::vector<float>{0,0};
// }
//
// /*
// */
// std::vector<float> Road::get_car_behind_pos_otherlane() {
//   return std::vector<float>{0,0};
// }

/*
*/
void Road::update_car_decisions() {
  for (Car &c : cars) {
    c.update_decision(this);
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
int Road::get_road_delim_x() {
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
      Car new_car = Car(init_vals.init_velocity[j], init_vals.init_max_velocity, x_pos, float(i), init_vals.init_accel_param, init_vals.init_max_accel_param, init_vals.init_min_accel_param, init_vals.init_length, init_vals.horizon, init_vals.init_max_delta_turning_angle);
      x_pos = x_pos + 300;
      road.add_car(new_car);
    }
  }
}

/*
*/
void Experiment::main_loop(std::string experiment_num, bool multi_lane) {
  std::vector<std::vector<std::vector<int>>> data; // 1st leve of arrays = iterations
                                                 // For each iteration = list of cars
                                                 // For each car = array containing position

  for(int i=0; i<max_it; i++) {
    // Save results
    std::vector<std::vector<int>> iteration;
    for (Car &c : road.get_cars()) {
      std::vector<int> car_pos;
      if ( multi_lane ) {
        car_pos.push_back(c.get_x_pos());
        car_pos.push_back(c.get_y_pos());
      } else { // Only store x position if single lange experiment
        car_pos.push_back(c.get_x_pos());
      }
      iteration.push_back(car_pos);
    }
    data.push_back(iteration);

    // Update positions (update after save so that we include the initial positions)
    road.update_car_decisions();
    road.update_car_positions(time_step);
  }

  std::string text = "Hello there";
  // std::string file_name = "./Experiments/experiment" + experiment_num + ".txt";
  write_data(text);
  // write_data(data, max_it, road.get_road_delim_x(), file_name);
  // write_space_time_Pbm(&data, file_name);
}
