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
    int human;
    bool merging;                       // Flag to indicate that the current car is merging
    bool merged;                        // Flag to check if this car has merged
    bool accelerating;                  // Flag to indicate that the car should accelerate
    float target_velocity_merge;              // AV slows down to a target velocity before lane merging
    float target_velocity_merge_reached;
    float starting_lane;
    float wait;                         // The human model waits a random time before letting someone into their lane during lane merging
    bool waited;
    
public:
    Car(int id, float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_car_length, float init_horizon, float init_max_delta_turning_angle, float reaction_time, float time_step, int human, float target_velocity_merge);
    int update_decision(Road* road, float time_step, int iteration);
    void update_position(float time_step);
    float get_x_pos();
    float get_x_pos_front_of_car();
    float get_y_pos();
    float get_length();
    float get_accel_param();
    float get_velocity();
    int get_id();
    bool get_merged();
    bool get_merging();
    bool at_target_velocity(float speed_limit);  // Checks if close enough
    bool in_target_lane();      // Checks if close enough
    float accelerate(Road* road, int car_ahead_id);
    void merge();
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
    float get_car_ahead_pos(float my_x_pos, float my_y_pos);
    float get_car_ahead_velocity(float car_ahead_x_pos, float car_ahead_y_pos);
    float get_car_ahead_accel_param(float car_ahead_x_pos, float car_ahead_y_pos);
    bool get_car_ahead_merged(float car_ahead_x_pos, float car_ahead_y_pos);
    float get_car_behind_pos(float my_x_pos);
//    bool check_merging_space(float my_x_pos, float my_y_pos, float safety_distance, float car_length); //check_merging_space(float my_x_pos, float my_y_pos, float space_needed);
    bool check_merging_space(int my_id, int id_ahead, int id_behind, float car_length);
    float get_car_ahead_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_behind_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_ahead_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_behind_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_ahead_pos_anylane(float my_x_pos);
    float get_car_pos_id(int id);
    float get_velocity_id(int id);
    bool get_car_merging_id(int id);
    int update_car_decisions(float time_step, int iteration);
    void update_car_positions(float time_step);
    void add_car(Car new_car);
    bool cars_at_speed_limit(); // Check if all the cars have equal velocities
    bool cars_in_lane_1();      // Check if all the cars are in lane 1
    std::vector<Car> get_cars();
    int get_road_length();
    int get_speed_limit();
    int get_begin_event();
    int get_n_cars_per_lane();
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
    int init_human;
    float init_target_velocity_merge;
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
    void main_loop(std::string experiment_num, bool multi_lane, bool event, int visualise); // event => whether braking/lane merging is happening
    bool experiment_finished();
};
