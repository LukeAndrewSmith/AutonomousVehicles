// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <math.h>
// #include <string>
// // #include <limits>
// #include <cfloat>
#include "road.hpp"

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
