/*
Explanatin of file
*/
#include <vector>

/*
Explanation of Car
*/
class Car {
private:
  uint8_t time;
  float velocity;
  float max_velocity;
  std::vector<float> pos;
  float accel_param;
  float length;
  float y_target;
  float horizon; // To explain
  float road_angle;
  float turning_angle;
  float max_delta_turning_angle;
  std::vector<bool> indicators;
public:
  Car(uint8_t init_time, float init_velocity, float init_max_velocity, std::vector<float> init_pos, float init_accel_param, float init_length, float horizon, float init_max_delta_turning_angle);
  void update_decision();
  void update_position(float time_step);
};

/*
Explanation of Road
*/
class Road {
private:
  std::vector<float> road_delim_x;
  std::vector<float> lanes;
  std::vector<Car> cars;
public:
  Road(std::vector<float> init_road_delim_x, std::vector<float> init_lanes);
  void car_ahead_pos();
  void car_behind_pos();
  void car_ahead_pos_otherlane();
  void car_behind_pos_otherlane();
};

/*
Experiment init
*/
struct init_experiment {
  // TODO:
  // // Car
  // uint8_t init_time;
  // float init_velocity;
  // std::vector<float> init_pos;
  // float init_accel_param;
  // float init_length;
  // float init_max_delta_turning_angle;
  // // Road
  // std::vector<float> init_road_delim;
  // std::vector<float> init_lanes;
};
/*
Explanation of Experiment
*/
class Experiment {
private:
  float time_step;
  int max_it;
  std::vector<Car> cars;
public:
  Experiment(init_experiment init_vals);
  void main_loop();
};
