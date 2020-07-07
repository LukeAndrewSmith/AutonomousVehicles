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
        
//        float power = 1;
//        temp_accel_param = -velocity/(std::pow(diff, power)) + velocity/(std::pow(safety_distance, power));
//        temp_accel_param = std::max(min_accel_param,temp_accel_param);

    } else if ( velocity < road->get_speed_limit() ) {
        
        // ATTEMPT: Using car ahead's accel_param
//        if ( car_ahead_pos == FLT_MAX ) { // No cars ahead, accelerate as much as possible
//            temp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
//        } else {
//            temp_accel_param = std::max(0.0f, road->get_car_ahead_accel_param(car_ahead_pos));
////            std::cout << temp_accel_param << std::endl;
//        }
//        if ( temp_accel_param < 1e-4 ) {
//            temp_accel_param = 0;
//        }
        
//        // ATTEMPT: Using car ahead's current velocity
//        if ( car_ahead_pos == FLT_MAX ) { // No cars ahead, accelerate as much as possible
//            temp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
//        } else {
//            float car_ahead_velocity = road->get_car_ahead_velocity(car_ahead_pos);
//            float car_ahead_accel_param = road->get_car_ahead_accel_param(x_pos);
//            float car_ahead_pos_next_it = x_pos + (max_velocity*time_step) + (((car_ahead_velocity-max_velocity)/car_ahead_accel_param)*(1-exp(-car_ahead_accel_param*time_step)) );
//            float car_ahead_velocity_next_it = max_velocity + (car_ahead_velocity-max_velocity)*exp(-car_ahead_accel_param*time_step);
//            float safety_distance_next_it = 2*car_ahead_velocity_next_it; // NOT OPTIMAL, SHOULD BE USING OUR OWN VELOCITY NEXT IT BUT WE DON'T KNOW IT YET, SO IT'S A SAFE ASSUMPTION
//            float max_x_pos = car_ahead_pos_next_it - safety_distance_next_it - car_length;
//
//            float max_accel = (1-(velocity/road->get_speed_limit()))*max_accel_param; // Attempt maximum acceleration
//            float test_x_pos = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/max_accel)*(1-exp(-max_accel*time_step)) );
//
//            if ( test_x_pos >= max_x_pos ) {
//                float diff_x_pos = max_x_pos - x_pos;
//                float max_velocity_next_it = diff_x_pos/time_step; // NOT OPTIMAL, AS
//                temp_accel_param = std::log( (max_velocity_next_it - max_velocity) / (velocity-max_velocity) ) / -time_step;
//            } else {
//                temp_accel_param = max_accel;
//            }
//        }
        
        // TODO: Leave as this for now
        // ATTEMPT: Always setting to maximum velocity
        temp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
        
        // Ensure convergence to 0 in reasonable time
        if ( temp_accel_param < 1e-4 ) {
            temp_accel_param = 0;
        }
    }
    
    prev_car_ahead_x_pos = car_ahead_pos;

    decision_buffer.push(temp_accel_param);
    accel_param = decision_buffer.front();
    decision_buffer.pop();
        
    return event_began;
}

// possible braking strategy
//        accel_param = -velocity/diff;
//        accel_param = std::max(min_accel_param,accel_param);
        
        // Here we make the assumption that the car was previously travelling at the same velocity as us
//        float car_ahead_velocity = (car_ahead_pos-prev_car_ahead_x_pos)/time_step;
//        accel_param = log((car_ahead_velocity/velocity))/time_step;
//        std::cout << accel_param << std::endl;
//        if ( accel_param > 0 ) accel_param = 0;
//            accel_param = -velocity/diff;
//            accel_param = std::max(min_accel_param,accel_param);
//        }

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
