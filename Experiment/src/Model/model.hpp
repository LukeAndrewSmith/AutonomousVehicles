#include <queue>

// Helpers
int random_num(int lower, int upper);

/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
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
    float car_length;
    float y_target;
    float horizon;                      // distance ahead to aim for during lane change
    float road_angle;
    float turning_angle;
    float max_delta_turning_angle;
    float braking_strategy_multiplier;
    bool overreacting;
    int overreaction_brake_iterations;
    int overreaction_pause_iterations;
    int overreaction_iterations;
    std::vector<int> ovverreaction_count;
    float previous_accel_param;
    float starting_velocity_accel;
    float accel_start_proportion; // Random parameters so that all humans have a different acceleration strategy
    float accel_max;
    // Concertina
    int event_iterations;               // The front car needs to count how many iterations to brake for
    // Below used for lane merging protocol
    int human;                          // Flag to indicate wether human or AV (human = 0 => AV, human = 1 => Human)
    bool merging;                       // Flag to indicate that the current car is merging
    bool merged;                        // Flag to check if this car has merged
    bool accelerating;                  // Flag to indicate that the car should accelerate
    float starting_lane;                // The protocol is dependent on the starting lane of the vehicle
    float wait;                         // The human model waits a random time before letting someone into their lane during lane merging
    bool waited;                        // Flag to indicate that the human model has waited
    
public:
    Car(int id, float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_car_length, float init_horizon, float init_max_delta_turning_angle, float init_overreaction_brake_time, float init_overreaction_pause_time, float braking_strategy_multiplier, float time_step, int human);
    int update_decision(Road* road, float time_step, int iteration, int n_iterations_event);
    void update_position(float time_step);
    float get_x_pos();
    float get_x_pos_front_of_car();
    float get_y_pos();
    float get_length();
    float get_accel_param();
    float get_velocity();
    int get_id();
    std::vector<int> get_ovvereaction_count();
    bool get_merged();
    bool get_merging();
    bool at_velocity(float speed_limit);  // Checks if close enough to target velocity => speeds up convergence
    bool greater_than_velocity(float target_velocity);
    bool in_target_lane();                       // Checks if close enough to target lane     => speeds up convergence
    float accelerate(Road* road, int car_ahead_id);     // Calculates the next accel_param
    void merge();                                       // Merges the car
};


/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
class Road {
private:
    int road_length;
    int speed_limit;
    int speed_limit_after_merge;
    int n_lanes;              // Lane 0 = 0, Lane 1 = 1, cars will try to go from lane 0 to lane 1
    std::vector<Car> cars;
    float begin_event;        // Indicates at what position braking/lane merging should begin
    float end_event;
    int av_only_merge_front; // Only allow the front two AV's to merge at a time
    float n_iterations_event; // Indicates for how long the car should brake during concertina
public:
    Road();
    Road(int init_road_length, int init_speed_limit, int init_speed_limit_after_merge, int init_n_lanes, float init_begin_event, float init_end_event, int init_av_only_merge_front, int init_n_iterations_event);
    float get_car_ahead_pos(float my_x_pos, float my_y_pos);
    float get_car_ahead_velocity(float car_ahead_x_pos, float car_ahead_y_pos);
    float get_car_ahead_accel_param(float car_ahead_x_pos, float car_ahead_y_pos);
    bool get_car_ahead_merged(float car_ahead_x_pos, float car_ahead_y_pos);
    float get_car_behind_pos(float my_x_pos);
    bool check_merging_space(int my_id, int id_ahead, int id_behind, float car_length);
    float get_car_ahead_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_behind_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_ahead_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_behind_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos);
    float get_car_ahead_pos_anylane(float my_x_pos);
    float get_car_pos_id(int id);
    float get_velocity_id(int id);
    bool get_car_merging_id(int id);
    bool get_car_merged_id(int id);
    int update_car_decisions(float time_step, int iteration);   // Updates for all cars on the road
    void update_car_positions(float time_step);                 // Updates for all cars on the road
    void add_car(Car new_car);
    bool cars_at_speed_limit();                                 // Check if all the cars are at the speed limit             (Experiment ending condition)
    bool cars_in_lane_1();                                      // Check if all the cars are in lane 1                      (Experiment ending condition)
    bool cars_at_safety_distance();                             // Check if all the cars are respecting the safety distance (Experiment ending condition)
    std::vector<Car> get_cars();
    int get_road_length();
    int get_speed_limit();
    int get_speed_limit_after_merge();
    int get_begin_event();
    int get_end_event();
    int get_av_only_merge_front();
    int get_n_cars_per_lane();
};


/*----------------------------------------------------------------------------*/
/*                             Init Experiment                                */
/*----------------------------------------------------------------------------*/
// Structure that contains all the initial parameters needed for an experiment
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
    float init_overreaction_brake_time;
    float init_overreaction_pause_time;
    float init_braking_strategy_multiplier;
    int init_human;
    // Road
    int init_road_length;               // ( road_length == 0 ) => continue until experiment_finished() and take road length to be the final x_pos of the front car
    int init_speed_limit;
    int init_speed_limit_after_merge;
    int init_n_lanes;
    float init_begin_event;             // ( begin_event < 0 ) => no event
    float init_end_event;
    int init_av_only_merge_front;
    int init_n_iterations_event;
    // Experiment
    float init_n_cars_per_lane;         // All lanes begin with the same number of cars
    int init_car_spacing;
    int init_max_it;                    // ( max_it == 0 ) => continue until 'experiment_finished()', otherwise stop at 'experiment_finished()' or when 'max_it' reached (whichever happens first)
    float init_time_step;
};


/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
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
