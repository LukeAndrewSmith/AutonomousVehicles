#include <math.h>
#include "car.hpp"

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
