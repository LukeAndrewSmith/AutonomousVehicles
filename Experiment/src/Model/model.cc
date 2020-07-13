#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string>
#include <queue>
// #include <limits>
#include <cfloat>
#include <random>
#include "model.hpp"
#include "write_data.hpp"
#include "visualisation.hpp"

// Helpers
int random_num(int lower, int upper) {
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distrib(lower, upper);
    return distrib(gen);
}

//float random_num_normal(float mean, float dev) {
////    std::default_random_engine generator;
//    std::random_device rd;  // Will be used to obtain a seed for the random number engine
//    std::mt19937 generator(rd()); // Standard mersenne_twister_engine seeded with rd()
//    std::normal_distribution<double> distribution(mean,dev);
//    return distribution(generator);
//}

/*----------------------------------------------------------------------------*/
/*                                  Car                                       */
/*----------------------------------------------------------------------------*/
/* Constructor */
Car::Car(int init_id, float init_velocity, float init_max_velocity, float init_x_pos, float init_y_pos, float init_accel_param, float init_max_accel_param, float init_min_accel_param, float init_car_length, float init_horizon, float init_max_delta_turning_angle, float init_reaction_time, float time_step, int init_human, float init_target_velocity_merge) {
    id = init_id;
    velocity = init_velocity;
    max_velocity = init_max_velocity;
    x_pos = init_x_pos;
    y_pos = init_y_pos;
    accel_param = init_accel_param;
    max_accel_param = init_max_accel_param;
    min_accel_param = init_min_accel_param;
    car_length = init_car_length;
    y_target = init_y_pos; // We want to stay in the same lane at the beginning
    road_angle = 0;
    turning_angle = 0;
    horizon = init_horizon;
    max_delta_turning_angle = init_max_delta_turning_angle;
    if ( init_reaction_time == -1 ) {
        reaction_time = float(random_num(15, 35))/100; // Random reaction time on interval 0.15...0.35
//        reaction_time = random_num_normal(1, 0.2); // Random reaction time on interval 0.15...0.35
//        std::cout << reaction_time << std::endl;
    } else {
        reaction_time = init_reaction_time;
    }
    int n_its_delay = reaction_time/time_step;
    for (int i=0; i<n_its_delay; i++) decision_buffer.push(0); // Initialise delay (reaction time)
    human = init_human;
    merging = false;
    merged = false;
    accelerating = false;
    target_velocity_merge = init_target_velocity_merge;
    target_velocity_merge_reached = false;
    starting_lane = init_y_pos;
    wait = 0;
    waited = false;
}

void Car::update_position(float time_step) { // UPDATE_MOVEMENT
    if (road_angle == 0 && turning_angle == 0) {
        if ( accel_param == 0 ) {
            x_pos = x_pos + (velocity*time_step); // Velocity/y_pos unchanged, x_pos updated
        } else if ( accel_param < 0 ) { // Braking
            x_pos = x_pos + ( (velocity/-accel_param)*(1 - exp(accel_param*time_step)) );
            velocity = velocity*exp(accel_param*time_step);
        } else if ( accel_param > 0 ) { // Accelerating
            x_pos = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(-accel_param*time_step)) );
            velocity = max_velocity + (velocity-max_velocity)*exp(-accel_param*time_step);
        }
    } else {
        // TODO: Check this acceleration technique
        if ( accel_param < 0 ) {
            velocity = velocity*exp(accel_param*time_step);
        } else if ( accel_param > 0 ) {
            velocity = max_velocity + (velocity-max_velocity)*exp(-accel_param*time_step);
        }
        road_angle = road_angle + (((velocity*time_step)/car_length)*sin(turning_angle));
        x_pos = x_pos + (velocity*time_step*cos(turning_angle)*cos(road_angle));
        y_pos = y_pos + (velocity*time_step*cos(turning_angle)*sin(road_angle));
    }
}

float Car::get_x_pos() {
  return x_pos;
}

float Car::get_x_pos_front_of_car() {
  return x_pos+car_length;
}

float Car::get_y_pos() {
  return y_pos;
}

float Car::get_length() {
    return car_length;
}

float Car::get_accel_param() {
    return accel_param;
}

float Car::get_velocity() {
    return velocity;
}

bool Car::get_merged() {
    return merged;
}

bool Car::get_merging() {
    return merging;
}

int Car::get_id() {
    return id;
}

bool Car::at_target_velocity(float speed_limit) {
    // TODO: Note: 0.1 is an arbitrary choice
    return abs(velocity-speed_limit) < 0.1;
}

bool Car::in_target_lane() {
    return abs(y_pos-1) < 0.01;
}

/*----------------------------------------------------------------------------*/
/*                                  Road                                      */
/*----------------------------------------------------------------------------*/
/* Default Constructor */
  // Included so that Experiment Constructor can initialise a random Road and we can then override this initialisation
  // Experiment cannot begin constructor without all it's members initialised so calls the default constructor on all members
  // TODO: This doesn't seem very clean to me...
Road::Road() {}

Road::Road(int init_road_length, int init_speed_limit, int init_n_lanes, float init_begin_event) {
    road_length = init_road_length;
    speed_limit = init_speed_limit;
    n_lanes = init_n_lanes;
    begin_event = init_begin_event;
}

float Road::get_car_ahead_pos(float my_x_pos, float my_y_pos) {
    float car_ahead_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if (c.get_x_pos() > my_x_pos && abs(c.get_y_pos() - my_y_pos) < 0.5) { // Cars ahead  // previosuly: c.get_y_pos() == my_y_pos, now checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( (c.get_x_pos() - my_x_pos) < (car_ahead_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_ahead_x_pos = c.get_x_pos();
            }
        }
    }
    return car_ahead_x_pos;
}

float Road::get_car_ahead_accel_param(float car_ahead_x_pos, float car_ahead_y_pos) {
    float car_ahead_accel_param = 0;
    for (Car c : cars) {
        if ( (c.get_x_pos() == car_ahead_x_pos && c.get_y_pos() == car_ahead_y_pos) ) {
            car_ahead_accel_param = c.get_accel_param();
            break;
        }
    }
    return car_ahead_accel_param;
}

float Road::get_car_ahead_velocity(float car_ahead_x_pos, float car_ahead_y_pos) {
    float car_ahead_velocity = 0;
    for (Car c : cars) {
        if ( (c.get_x_pos() == car_ahead_x_pos && c.get_y_pos() == car_ahead_y_pos) ) {
            car_ahead_velocity = c.get_velocity();
            break;
        }
    }
    return car_ahead_velocity;
}

bool Road::get_car_ahead_merged(float car_ahead_x_pos, float car_ahead_y_pos) {
    float car_ahead_velocity = 0;
    for (Car c : cars) {
        if ( (c.get_x_pos() == car_ahead_x_pos && c.get_y_pos() == car_ahead_y_pos) ) {
            car_ahead_velocity = c.get_merged();
            break;
        }
    }
    return car_ahead_velocity;
    
}

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

float Road::get_car_ahead_pos_otherlane(float my_x_pos, float my_y_pos) {
    float other_y_pos = 1-my_y_pos;
    float car_ahead_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if (c.get_x_pos() > my_x_pos && abs(c.get_y_pos() - other_y_pos) < 0.5) { // Cars ahead:  Checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( (c.get_x_pos() - my_x_pos) < (car_ahead_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_ahead_x_pos = c.get_x_pos();
            }
        }
    }
    return car_ahead_x_pos;
}

float Road::get_car_behind_pos_otherlane(float my_x_pos, float my_y_pos) {
    float other_y_pos = 1-my_y_pos;
    float car_behind_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if (c.get_x_pos() < my_x_pos && abs(c.get_y_pos() - other_y_pos) < 0.5) { // Cars ahead:  Checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( abs(c.get_x_pos() - my_x_pos) < abs(car_behind_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_behind_x_pos = c.get_x_pos();
            }
        }
    }
    return car_behind_x_pos;
}

float Road::get_car_ahead_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos) { // ahead or next to mean 'c.get_x_pos() >= my_x_pos' not '>'
    float other_y_pos = 1-my_y_pos;
    float car_ahead_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if (c.get_x_pos() >= my_x_pos && abs(c.get_y_pos() - other_y_pos) < 0.5) { // Cars ahead:  Checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( (c.get_x_pos() - my_x_pos) < (car_ahead_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_ahead_x_pos = c.get_x_pos();
            }
        }
    }
    return car_ahead_x_pos;
}

float Road::get_car_behind_or_next_to_pos_otherlane(float my_x_pos, float my_y_pos) { // behind or next to mean 'c.get_x_pos() <= my_x_pos' not '<'
    float other_y_pos = 1-my_y_pos;
    float car_behind_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if (c.get_x_pos() <= my_x_pos && abs(c.get_y_pos() - other_y_pos) < 0.5) { // Cars ahead:  Checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( abs(c.get_x_pos() - my_x_pos) < abs(car_behind_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_behind_x_pos = c.get_x_pos();
            }
        }
    }
    return car_behind_x_pos;
}

float Road::get_car_ahead_pos_anylane(float my_x_pos) {
    float car_ahead_x_pos = FLT_MAX; // std::numeric_limits<float>::max;
    for (Car c : cars) {
        if ( c.get_x_pos() > my_x_pos ) { // Cars ahead  // previosuly: c.get_y_pos() == my_y_pos, now checks if car in the same lane, lane 0 or 1 => distance between my_y_pos and c.get_y_pos() needs to be less than 0.5
            if ( (c.get_x_pos() - my_x_pos) < (car_ahead_x_pos - my_x_pos) ) { // Update x_pos if closer
                car_ahead_x_pos = c.get_x_pos();
            }
        }
    }
    return car_ahead_x_pos;
}

float Road::get_car_pos_id(int id) {
    for (Car c : cars) {
        if ( c.get_id() == id ) {
            return c.get_x_pos();
            break;
        }
    }
    return FLT_MAX;
}

float Road::get_velocity_id(int id) {
    for (Car c : cars) {
        if ( c.get_id() == id ) {
            return c.get_velocity();
            break;
        }
    }
    return FLT_MAX;
}

bool Road::get_car_merging_id(int id) {
    for (Car c : cars) {
        if ( c.get_id() == id ) {
            return c.get_merging();
        }
    }
    return false;
}

int Road::update_car_decisions(float time_step, int iteration) {
    int experiment_began = 0;
    for (Car &c : cars) {
        experiment_began = std::max(c.update_decision(this, time_step, iteration), experiment_began);
    }
    return experiment_began;
}

void Road::update_car_positions(float time_step) {
    for (Car &c : cars) {
        c.update_position(time_step);
    }
}

void Road::add_car(Car new_car) {
    cars.push_back(new_car);
}

bool Road::cars_at_speed_limit() {
    if ( !cars.front().at_target_velocity(speed_limit) ) return false;   // First car
    if ( !cars.back().at_target_velocity(speed_limit) ) return false;    // Last car
    return true;
}

bool Road::cars_in_lane_1() {
    for ( Car c : cars ) {
        if ( c.get_y_pos() != 1 ) return false;
    }
    return true;
}


std::vector<Car> Road::get_cars() {
    return cars;
}

int Road::get_road_length() {
    return road_length;
}

int Road::get_speed_limit() {
    return speed_limit;
}

int Road::get_begin_event() {
    return begin_event;
}

int Road::get_n_cars_per_lane() {
    return (cars.size()/n_lanes);
}



/*----------------------------------------------------------------------------*/
/*                               Experiment                                   */
/*----------------------------------------------------------------------------*/
/* Constructor */
Experiment::Experiment(init_experiment init_vals) {
    // Road
    road = Road(init_vals.init_road_length, init_vals.init_speed_limit, init_vals.init_n_lanes, init_vals.init_begin_event);
    // Experiment
    n_cars_per_lane = init_vals.init_n_cars_per_lane; // n_cars_per_lane[0] = #cars in lane at lanes[0]
    car_spacing = init_vals.init_car_spacing;
    max_it = init_vals.init_max_it;
    time_step = init_vals.init_time_step;
    // Cars
    float x_pos = 0;
    for (int i=0; i<init_vals.init_n_lanes; i++) {
        x_pos = 0;
        for (int j=0; j<n_cars_per_lane; j++) {
            int id = (i*n_cars_per_lane) + j;       // e.g lane 0: 0 1 2 3,    lane 1: 4 5 6 7
            Car new_car = Car(id, init_vals.init_velocity, init_vals.init_max_velocity, x_pos, float(i), init_vals.init_accel_param, init_vals.init_max_accel_param, init_vals.init_min_accel_param, init_vals.init_car_length, init_vals.init_horizon, init_vals.init_max_delta_turning_angle, init_vals.init_reaction_time, init_vals.init_time_step, init_vals.init_human, init_vals.init_target_velocity_merge);
            x_pos += new_car.get_length() + init_vals.init_car_spacing;
            road.add_car(new_car);
        }
    }
}

/*
*/
void Experiment::main_loop(std::string experiment_file_name, bool multi_lane, bool event, int visualise) {
    std::vector<std::vector<std::vector<float>>> data;  // 1st level of arrays = iterations
                                                        // For each iteration = list of cars
                                                        // For each car = array containing position
    
    float experiment_start_it = -1;
    int i = 0;
    // if max_it==0 => carry on until !experiment_finished()
        // if event==true => wait until the event has started before checking experiment_finished()
    // otherwise loop until max_it
    while ( ( max_it==0 && ( ( event && experiment_start_it==-1 ) || !experiment_finished() ) )     // Experiment not finished
         || ( max_it!=0 && i<max_it ) ) {
//        std::cout << i << std::endl;
        // Save results
        std::vector<std::vector<float>> iteration;
        for (Car &c : road.get_cars()) {
            std::vector<float> car_pos;
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
        if ( road.update_car_decisions(time_step,i) == 1 ) { // Returns 1 upon starting of event
            if ( experiment_start_it == -1 ) experiment_start_it = i;
        }
        road.update_car_positions(time_step);
        i++;
    }
        
    float time_taken = (i-experiment_start_it)*time_step;
    float road_length = road.get_road_length();
    if ( road_length == 0 ) {
        road_length = road.get_cars().back().get_x_pos_front_of_car();
    }
    float iterations = max_it;
    if ( iterations == 0 ) {
        iterations = i-1;
    }
    std::cout << "Time taken:" << std::endl;
    std::cout << time_taken << std::endl;
    write_results(experiment_file_name+"_results", road.get_cars().size(), time_taken);
    if ( visualise ) {
        if ( multi_lane ) {
            write_lane_changing_gif(data, road_length, iterations, experiment_file_name, road.get_begin_event());
        } else {
            write_space_time_Pbm(data, road_length, iterations, experiment_file_name);
        }
    }
}
