#include <vector>

#ifndef ROAD_H_INCLUDED
#define ROAD_H_INCLUDED

class Car;
/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
/*
Explanation of Road
*/
class Road {
private:
  int road_delim_x; // Indicates end of road, we assume start = 0
  std::vector<float> lanes;
  std::vector<Car> cars;
public:
  Road();
  Road(float init_road_delim_x, std::vector<float> init_lanes);
  float get_car_ahead_pos(float my_x_pos);
  float get_car_behind_pos(float my_x_pos);
  // std::vector<float> get_car_ahead_pos_otherlane();
  // std::vector<float> get_car_behind_pos_otherlane();
  void update_car_decisions();
  void update_car_positions(float time_step);
  void add_car(Car new_car);
  std::vector<Car> get_cars();
  int get_road_delim_x();
};

#endif
