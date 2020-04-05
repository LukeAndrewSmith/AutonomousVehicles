
/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/*
Explanation of Car
*/

class Road; // Forward decleration of road as it is needed for 'update_decision(Road* road);' method

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


/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/*
Experiment init
*/
struct init_experiment {
  // Car
  std::vector<float> init_velocity;
  float init_max_velocity;
  float init_accel_param;
  float init_max_accel_param;
  float init_min_accel_param;
  float init_length;
  float horizon;
  float init_max_delta_turning_angle;
  // Road
  float init_road_delim_x;
  std::vector<float> init_lanes;
  // Experiment
    // Initialise cars and indicate refill protocol (how cars are input into the system)
  std::vector<int> init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
  int init_spacing_protocol;
  int init_refill_protocol;
  int init_max_it;
  float init_time_step;
};
/*
Explanation of Experiment
*/
class Experiment {
private:
  Road road;
  std::vector<int> n_cars_per_lane;
  int spacing_protocol; // Initial position of the cars (densly packed or not)
  int refill_protocol; // How to intput INLET_PROTOCOL
  int max_it; // Number of iterations
  float time_step; // Time passed in each iteration (should be small as we have made small angle approximations)

public:
  Experiment(init_experiment init_vals);
  void main_loop(std::string experiment_num, bool multi_lane);
  void write_to_file(std::vector<float> cars);
};
