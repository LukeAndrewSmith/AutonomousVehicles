#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>
#include <math.h>
#include "model.hpp"
#include "write_data.hpp"

/*----------------------------------------------------------------------------*/
/*                     Experiment Specific Implementations                    */
/*----------------------------------------------------------------------------*/
int Car::update_decision(Road* road, float time_step, int iteration, int n_iterations_event) {
    int id_ahead = id+1;
    float car_ahead_pos = road->get_car_pos_id(id_ahead);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-(x_pos+car_length);
    int event_began = 0;
    float temp_accel_param = 0;
    if ( ( id == road->get_cars().size()-1 ) && ( road->get_begin_event()!=0 ) && ( x_pos > road->get_begin_event() ) && ( event_iterations <= n_iterations_event ) ) {
        event_iterations++;
        temp_accel_param = min_accel_param;
        if ( velocity <= 5 ) temp_accel_param = 0;
        if ( event_iterations == n_iterations_event ) event_began = 1;
    } else if ( diff < safety_distance ) {
        temp_accel_param = (std::abs(braking_strategy_multiplier*min_accel_param)/safety_distance)*diff + braking_strategy_multiplier*min_accel_param;
        temp_accel_param = std::max(min_accel_param,temp_accel_param);
    } else if ( velocity < road->get_speed_limit() ) {
        if ( accel_param <= 0 ) starting_velocity_accel = velocity;
        float v_scaled = (velocity - starting_velocity_accel) * (M_PI / (road->get_speed_limit() - starting_velocity_accel));
        temp_accel_param = accel_max * (std::sin(v_scaled)*(1-accel_start_proportion) + accel_start_proportion);
    }
    
    if ( human && id != road->get_cars().size()-1 ) {
        if ( !overreacting && accel_param < 0 && previous_accel_param < accel_param && accel_param < temp_accel_param ) {
            ovverreaction_count[0]++;
            overreacting = true;
        }
        if ( overreacting ) {
            if ( overreaction_iterations < overreaction_brake_iterations ) {
                if ( temp_accel_param < accel_param ) {
                    ovverreaction_count[1]++;
                    overreaction_iterations = 0;
                    overreacting = false;
                } else {
                    overreaction_iterations++;
                    temp_accel_param = accel_param;
                }
            } else if ( overreaction_iterations >= overreaction_brake_iterations && overreaction_iterations < overreaction_brake_iterations+overreaction_pause_iterations ) {
                if ( temp_accel_param < min_accel_param/2 ) {
                    ovverreaction_count[2]++;
                    overreaction_iterations = 0;
                    overreacting = false;
                } else {
                    overreaction_iterations++;
                    temp_accel_param = 0;
                }
            } else {
                overreaction_iterations = 0;
                overreacting = false;
            }
        }
    }
    
    previous_accel_param = accel_param;
    accel_param = temp_accel_param;
        
    // Collision
    if ( diff <= 0 ) {
        std::cout << "Collision" << ", ";
        exit(-1);
    }
    
    return event_began;
}

bool Experiment::experiment_finished() {
    return road.cars_at_speed_limit() && road.cars_at_safety_distance();
}


/*----------------------------------------------------------------------------*/
/*                                 Experiment                                 */
/*----------------------------------------------------------------------------*/
// Returns the number of the experiment
int main(int argc, char const *argv[]) {
    
    if (argc != 25+1) {
        std::cout << "Usage: See code...\n";
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
    float init_overreaction_brake_time  = atof(argv[9]);
    float init_overreaction_pause_time  = atof(argv[10]);
    float init_braking_strategy_multiplier = atof(argv[11]);
    int init_human                      = atoi(argv[12]);
    // Road
    int init_road_length                = atof(argv[13]);
    int init_speed_limit                = atoi(argv[14]);
    int init_speed_limit_after_merge    = atoi(argv[15]);
    int init_n_lanes                    = atoi(argv[16]);
    float init_begin_event              = atof(argv[17]);
    float init_end_event                = atof(argv[18]);
    int init_av_only_merge_front        = atoi(argv[19]);
    int init_n_iterations_event         = atoi(argv[20]);
    // Experiment
    float init_n_cars_per_lane          = atof(argv[21]);
    int init_car_spacing                = atoi(argv[22]);
    int init_max_it                     = atoi(argv[23]);
    float init_time_step                = atof(argv[24]);
    // Visualisation
    int visualise                       = atof(argv[25]);

    init_experiment init_vals = {  // See model.hpp for definition
        init_velocity,
        init_max_velocity,
        init_accel_param,
        init_max_accel_param,
        init_min_accel_param,
        init_car_length,
        init_horizon,
        init_max_delta_turning_angle,
        init_overreaction_brake_time,
        init_overreaction_pause_time,
        init_braking_strategy_multiplier,
        init_human,
        // Road
        init_road_length,
        init_speed_limit,
        init_speed_limit_after_merge,
        init_n_lanes,
        init_begin_event,
        init_end_event,
        init_av_only_merge_front,
        init_n_iterations_event,
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
