#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include "model.hpp"
#include "write_data.hpp"

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
int Car::update_decision(Road* road, float time_step, int iteration) {
    float car_ahead_pos = road->get_car_ahead_pos(x_pos);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-x_pos;
    if ( x_pos < road->get_begin_event() ) {   // Before Lane Merging
        if ( diff < safety_distance ) { // TODO: Smoother braking
            accel_param = -velocity/diff;
            accel_param = std::max(min_accel_param,accel_param);
        } else { // TODO: Implement Accelerating to max road speed: new argument max_road_speed
            accel_param = (1-(velocity/max_velocity))*max_accel_param;
        }
    } else if ( x_pos >= road->get_begin_event() && y_pos < 1 ) {  // Begin Lane Merging
        y_target = 1;
        float psi = atan( (y_target-y_pos)/(horizon - (car_length*cos(road_angle))) ) - road_angle;
        float check = psi-turning_angle;
        if ( check < -max_delta_turning_angle ) {
            turning_angle = turning_angle - max_delta_turning_angle;
        } else if ( abs(check) < max_delta_turning_angle ) {
            turning_angle = psi;
        } else if ( check > max_delta_turning_angle ) {
            turning_angle = turning_angle + max_delta_turning_angle;
        }
        // TODO: Never actually gets to the other lane, this is cheating a bit
        if ( y_target-y_pos<0.001 && turning_angle<10e-13 ) {
            turning_angle = 0;
            road_angle = 0;
            y_pos = 1;
        }
    }
    return 0;
}

// Returns the number of the experiment
int main(int argc, char const *argv[]) {
    
    if (argc != 17+1) {
        std::cout << "Usage: concertina <init_velocity> <max_velocity> <init_accel_param> <max_accel_param> <min_accel_param> <car_length> <horizon> <max_delta_turning_angle> <reaction_time> <road_length> <speed_limit> <n_lanes> <n_cars_per_lane> <car_spacing> <max_it> <time_step>";
        std::exit(0);
    }
    
    float init_velocity                 = atof(argv[1]);
    float init_max_velocity             = atof(argv[2]);
    float init_accel_param              = atof(argv[3]);
    float init_max_accel_param          = atof(argv[4]);
    float init_min_accel_param          = atof(argv[5]);
    float init_car_length               = atof(argv[6]);
    float init_horizon                  = atof(argv[7]);
    float init_max_delta_turning_angle  = atof(argv[8]);
    float init_reaction_time            = atof(argv[9]);
    // Road
    int init_road_length                = atof(argv[10]);
    int init_speed_limit                = atoi(argv[11]);
    int init_n_lanes                    = atoi(argv[12]);
    float init_begin_event              = atof(argv[13]);
    // Experiment
    float init_n_cars_per_lane          = atof(argv[14]);
    int init_car_spacing                = atoi(argv[15]);
    int init_max_it                     = atoi(argv[16]);
    float init_time_step                = atof(argv[17]);
        
    init_experiment init_vals = {  // See model.hpp for definition
        init_velocity,
        init_max_velocity,
        init_accel_param,
        init_max_accel_param,
        init_min_accel_param,
        init_car_length,
        init_horizon,
        init_max_delta_turning_angle,
        init_reaction_time,
        // Road
        init_road_length,
        init_speed_limit,
        init_n_lanes,
        init_begin_event,
        // Experiment
        init_n_cars_per_lane,
        init_car_spacing,
        init_max_it,
        init_time_step,
    };
    
    Experiment concertina(init_vals);
    std::string dir = "./Experiments/lane_change";
    int experiment_num;
    std::string experiment_name = get_file_name(dir,&experiment_num);
    std::string experiment_file_name = dir + "/" + experiment_name;
    concertina.main_loop(experiment_file_name, init_n_lanes>1, init_begin_event>0);

    return experiment_num;
}
