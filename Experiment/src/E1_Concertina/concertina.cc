#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>
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
    float car_ahead_pos = road->get_car_ahead_pos(x_pos, y_pos);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-get_x_pos_front_of_car();
    int event_began = 0; // TODO: only call set to 1 once, at the beginning of the braking or at the end
    float temp_accel_param = 0;
    if ( ( id == 0 ) && ( road->get_begin_event()!=0 ) && ( x_pos > road->get_begin_event() ) && ( x_pos < road->get_begin_event()+40 ) ) {
        temp_accel_param = min_accel_param;
    } else if ( ( id == 0 ) && ( road->get_begin_event()!=0 ) && ( x_pos >= road->get_begin_event() + 40 ) && ( x_pos < road->get_begin_event()+80 ) )  {
        event_began = 1; // Only indicate that event_began here so that the cars have time to react and slow down (otherwise if reaction_time>0 the condition road.cars_at_speed_limit() will be met immediately and the experiment will end the iteration after it started)
        temp_accel_param = 0;
    } else if ( diff <= safety_distance ) { // TODO: CHECK BRAKING
        temp_accel_param = -velocity/diff;
        temp_accel_param = std::max(min_accel_param,temp_accel_param);

    } else if ( velocity < road->get_speed_limit() ) {
        
        // TODO: Previous attempts
        // ATTEMPT: Using car ahead's accel_param
        // ATTEMPT: Always setting to maximum velocity

        // TODO: Leave as this for now
        temp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
        if ( at_target_velocity(road->get_speed_limit()) ) accel_param = 0; // Ensure convergence to 0 in reasonable time
    }
    
    prev_car_ahead_x_pos = car_ahead_pos;

    decision_buffer.push(temp_accel_param);
    accel_param = decision_buffer.front();
    decision_buffer.pop();
        
    return event_began;
}

bool Experiment::experiment_finished() {
    return road.cars_at_speed_limit();
}

// Returns the number of the experiment
int main(int argc, char const *argv[]) {
    
    if (argc != 20+1) {
        std::cout << "Usage: concertina <init_velocity> <max_velocity> <init_accel_param> <max_accel_param> <min_accel_param> <car_length> <horizon> <max_delta_turning_angle> <reaction_time> <road_length> <speed_limit> <n_lanes> <n_cars_per_lane> <car_spacing> <max_it> <time_step> <visualisation>";
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
    int init_human                      = atoi(argv[10]);
    float init_target_velocity_merge    = atof(argv[11]);
    // Road
    int init_road_length                = atof(argv[12]);
    int init_speed_limit                = atoi(argv[13]);
    int init_n_lanes                    = atoi(argv[14]);
    float init_begin_event              = atof(argv[15]);
    // Experiment
    float init_n_cars_per_lane          = atof(argv[16]);
    int init_car_spacing                = atoi(argv[17]);
    int init_max_it                     = atoi(argv[18]);
    float init_time_step                = atof(argv[19]);
    // Visualisation
    int visualise                       = atof(argv[20]);
        
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
        init_human,
        init_target_velocity_merge,
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
    std::string dir = "./Experiments/concertina";
    int experiment_num;
    std::string experiment_name = get_file_name(dir,&experiment_num);
    std::string experiment_file_name = dir + "/" + experiment_name;
    concertina.main_loop(experiment_file_name, init_n_lanes>1, init_begin_event>0, visualise);

    return experiment_num;
}
