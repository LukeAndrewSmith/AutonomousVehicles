#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>
//#include <math.h>
#include "model.hpp"
#include "write_data.hpp"

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
int Car::update_decision(Road* road, float time_step) {
    float car_ahead_pos = road->get_car_ahead_pos(x_pos);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-get_x_pos_front_of_car();
    int event_began = 0; // TODO: only call set to 1 once, at the beginning of the braking or at the end
//     TODO: TRIGGER BRAKING FOR CONCERTINA EFFECT
//    if ( ( id == 0 ) && ( x_pos > road->get_begin_event() ) && ( x_pos < road->get_begin_event()+40 ) ) {
//        event_began = 1;
//        accel_param = min_accel_param; //min_accel_param/2;
//    } else if ( ( id == 0 ) && ( x_pos >= road->get_begin_event() + 40 ) && ( x_pos < road->get_begin_event()+80 ) )  {
//        accel_param = 0;
//    } else
    if ( diff <= safety_distance ) { // TODO: Smoother braking/ CHECK BRAKING
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
        
        accel_param = -velocity/diff;//*1.3;
        accel_param = std::max(min_accel_param,accel_param);
        
//        if ( id == 1 ) {
//            std::cout << "v: " << velocity << std::endl;
//            std::cout << "s: " << safety_distance << std::endl;
//            std::cout << "d: " << diff << std::endl;
//        }
        
    } else if (velocity<road->get_speed_limit()) { // TODO: Implement Accelerating to max road speed: new argument max_road_speed

        
//        ((1-(x./speedlimit)).*maxaccel).*(1-(min(x,speedlimit)/speedlimit))
//        accel_param = ((1-(velocity/road->get_speed_limit()))*max_accel_param) * (1-fmin(velocity,road->get_speed_limit())/road->get_speed_limit());
//        std::cout << "a: " << accel_param << std::endl;
//        std::cout << "v: " << velocity << std::endl;
        
        accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
        if ( accel_param < 1e-4 ) {
            accel_param = 0;
        }
        if ( id == 1 ) {
            std::cout << accel_param << std::endl;
//            std::cout << diff << std::endl;
        }
//        if ( accel_param < 1e-2 ) {
//            // This should hold true: road()->speed_limit = max_velocity + (velocity-max_velocity)*exp(-accel_param*time_step);
//            accel_param = -log( (road->get_speed_limit()-max_velocity)/(velocity-max_velocity) ) / time_step;
//            std::cout << "a: " << accel_param << std::endl;
//        }
        
//        if ( id == 0 ) {
//            std::cout << "a: " << accel_param << std::endl;
//            std::cout << "v: " << velocity << std::endl;
//        }
        
//        if ( diff > 2*safety_distance ) {
//            accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
//            if ( accel_param < 1e-4 ) {
//                accel_param = 0;
//            }
//        } else {
//            // Estimate car aheads acceleration, assumption: his current acceleration value is the same as ours
//            float car_ahead_velocity = (car_ahead_pos-prev_car_ahead_x_pos)/time_step;
//            float car_ahead_accel_param = log((car_ahead_velocity/velocity))/time_step;
//            float car_ahead_pos_next_it = car_ahead_pos + (max_velocity*time_step) + ( ((car_ahead_velocity-max_velocity)/car_ahead_accel_param)*(1-exp(-car_ahead_accel_param*time_step)) );
//            float x_pos_next_it = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(-accel_param*time_step)) );
//            accel_param = log((((x_pos_next_it-x_pos)/time_step)/velocity))/time_step;
//            if ( accel_param < 1e-4 ) {
//                accel_param = 0;
//            }
    //        x_pos = x_pos + (max_velocity*time_step) + ( ((velocity-max_velocity)/accel_param)*(1-exp(-accel_param*time_step)) );
//        }
    }
    prev_car_ahead_x_pos = car_ahead_pos;
    
    return event_began;
}

bool Experiment::experiment_finished() {
    return road.cars_at_speed_limit();
}

// Returns the number of the experiment
int main(int argc, char const *argv[]) {
    
    if (argc != 16+1) {
        std::cout << "Usage: concertina <init_velocity> <max_velocity> <init_accel_param> <max_accel_param> <min_accel_param> <car_length> <horizon> <max_delta_turning_angle> <road_length> <speed_limit> <n_lanes> <n_cars_per_lane> <car_spacing> <max_it> <time_step>";
        std::exit(0);
    }
    
    float init_velocity = atof(argv[1]);                    // float init_velocity
    float init_max_velocity = atof(argv[2]);                // float init_max_velocity;
    float init_accel_param = atof(argv[3]);                 // float init_accel_param;
    float init_max_accel_param = atof(argv[4]);             // float init_max_accel_param;
    float init_min_accel_param = atof(argv[5]);             // float init_min_accel_param;
    float init_car_length = atof(argv[6]);                      // float init_car_length;
    float horizon = atof(argv[7]);                          // float horizon;
    float init_max_delta_turning_angle = atof(argv[8]);     // float init_max_delta_turning_angle;
    // Road
    int init_road_length = atof(argv[9]);                   // float init_road_length;
    int init_speed_limit = atoi(argv[10]);                  // int init_n_lanes;
    int init_n_lanes = atoi(argv[11]);                      // int init_n_lanes;
    float init_begin_event = atof(argv[12]);
    // Experiment
    float init_n_cars_per_lane = atof(argv[13]);            // int init_n_cars_per_lane;
    int init_car_spacing = atoi(argv[14]);                  // int init_car_spacing;
    int init_max_it = atoi(argv[15]);                       // int init_max_it;
    float init_time_step = atof(argv[16]);                  // float init_time_step;
        
    init_experiment init_vals = {           // See model.hpp for definition
        init_velocity,                      // float init_velocity
        init_max_velocity,                  // float init_max_velocity;
        init_accel_param,                   // float init_accel_param;
        init_max_accel_param,               // float init_max_accel_param;
        init_min_accel_param,               // float init_min_accel_param;
        init_car_length,                        // float init_car_length;
        horizon,                            // float horizon;
        init_max_delta_turning_angle,       // float init_max_delta_turning_angle;
        // Road
        init_road_length,                   // float init_road_length;
        init_speed_limit,                   // int init_speed_limit;
        init_n_lanes,                       // int init_n_lanes;
        init_begin_event,              // int init_begin_event;
        // Experiment
        init_n_cars_per_lane,               // int init_n_cars_per_lane;
        init_car_spacing,                   // int init_car_spacing;
        init_max_it,                        // int init_max_it;
        init_time_step,                     // float init_time_step;
    };
    
    Experiment concertina(init_vals);
    std::string dir = "./Experiments/concertina";
    int experiment_num;
    std::string experiment_name = get_file_name(dir,&experiment_num);
    std::cout << "no: " << experiment_name << std::endl;
    std::string experiment_file_name = dir + "/" + experiment_name;
    concertina.main_loop(experiment_file_name, init_n_lanes>1, init_begin_event>0);

    return experiment_num;
}
