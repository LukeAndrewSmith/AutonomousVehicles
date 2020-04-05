#include <vector>

#ifndef CAR_H_INCLUDED
#define CAR_H_INCLUDED

class Road;
/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/*
Explanation of Car
*/
class Car {
private:
  float velocity; // [m/s]
  float max_velocity;
  float x_pos; // [m]
  float y_pos; // lanes are integers
  float accel_param;
  float max_accel_param;
  float min_accel_param;
  float length;
  float y_target;
  float horizon; // To explain
  float road_angle;
  float turning_angle;
  float max_delta_turning_angle;
  std::vector<bool> indicators;
public:
  Car(float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_length, float init_horizon, float init_max_delta_turning_angle);
  void update_decision(Road* road);
  void update_position(float time_step, int road_delim_x);
  float get_x_pos();
  float get_y_pos();
};

#endif
