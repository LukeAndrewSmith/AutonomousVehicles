#include <queue> // TODO: Why is this needed and not <vector>???
/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/*
Explanation of Car
*/

class Road; // Forward declaration of road as it is needed for 'update_decision(Road* road);' method

class Car {
private:
    int id;
    float velocity;                     // [m/s]
    float max_velocity;
    float x_pos;                        // [m]  Position of back wheels
    float y_pos;                        // lanes are integers
    float accel_param;
    float max_accel_param;
    float min_accel_param;
    float prev_car_ahead_x_pos;
    float car_length;
    float y_target;
    float horizon;                      // TODO: To explain
    float road_angle;
    float turning_angle;
    float max_delta_turning_angle;
    std::vector<bool> indicators;
    float reaction_time;                // [s]
    float last_reaction_it;
    std::queue<float> decision_buffer;  // acceleration_parameters offset by reaction time
public:
    int count;
    Car(int id, float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_car_length, float init_horizon, float init_max_delta_turning_angle, float reaction_time, float time_step);
    int update_decision(Road* road, float time_step, int iteration);
    void update_position(float time_step);
    float get_x_pos();
    float get_x_pos_front_of_car();
    float get_y_pos();
    float get_length();
    float get_accel_param();
    float get_velocity();
};


/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
/*
Explanation of Road
*/
class Road {
private:
    int road_length;
    int speed_limit;
    int n_lanes;              // Lane 0 = 0, Lane 1 = 1, cars will try to go from lane 0 to lane 1
    std::vector<Car> cars;
    float begin_event;        // Indicates at what position braking/lane merging should begin
public:
    Road();
    Road(int init_road_length, int speed_limit, int init_n_lanes, float begin_event);
    float get_car_ahead_pos(float my_x_pos);
    float get_car_behind_pos(float my_x_pos);
    // std::vector<float> get_car_ahead_pos_otherlane();
    // std::vector<float> get_car_behind_pos_otherlane();
    int update_car_decisions(float time_step, int iteration);
    void update_car_positions(float time_step);
    void add_car(Car new_car);
    bool cars_at_speed_limit(); // Check if all the cars have equal velocities
    std::vector<Car> get_cars();
    int get_road_length();
    int get_speed_limit();
    int get_begin_event();
};


/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/*
Experiment init
*/
struct init_experiment {
    // Car
    float init_velocity;                // All cars begin with the same velocity
    float init_max_velocity;
    float init_accel_param;
    float init_max_accel_param;
    float init_min_accel_param;
    float init_car_length;
    float init_horizon;
    float init_max_delta_turning_angle;
    float init_reaction_time;
    // Road
    int init_road_length;               // ( road_length == 0 ) => continue until experiment_finished()
                                        // and take road length to be the final x_pos of the front car
    int init_speed_limit;
    int init_n_lanes;
    float init_begin_event;                  // ( begin_event < 0 ) => no event
    // Experiment
    float init_n_cars_per_lane;         // All lanes begin with the same number of cars
    int init_car_spacing;
    int init_max_it;                    // ( max_it == 0 ) => continue until experiment_finished()
    float init_time_step;
};
/*
Explanation of Experiment
*/
class Experiment {
private:
    Road road;
    int n_cars_per_lane;  // Same initial number of cars per lane
    int car_spacing;      // Initial distance between the the cars
    int max_it;           // Number of iterations
    float time_step;      // Time passed in each iteration (should be small as we have made small angle approximations)
public:
    Experiment(init_experiment init_vals);
    void main_loop(std::string experiment_num, bool multi_lane, bool event); // event => whether braking/lane merging is happening
    bool experiment_finished();
};
