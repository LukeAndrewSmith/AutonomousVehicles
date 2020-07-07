#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <cfloat>
#include "model.hpp"
#include "write_data.hpp"

void id_says(std::string to_say, int target_id, int id) {
    if ( id == target_id ) {
        std::cout << to_say << std::endl;
    }
}

//float fast_acceleration(float velocity, float speed_limit, float max_accel_param) {
//    float tmp = (1-exp(-5*(speed_limit-velocity)))*max_accel_param;
//    if ( tmp < 1e-4 ) {     // Ensure convergence to 0 in reasonable time
//        tmp = 0;
//    }
//    return tmp;
//}

float fast_acceleration(float velocity, float speed_limit, float max_accel_param) {
    float tmp = (1-(velocity/speed_limit))*max_accel_param;
    temp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;

    if ( tmp < 1e-4 ) { // Ensure convergence to 0 in reasonable time
        tmp = 0;
    }
    return tmp;
}

//float fast_acceleration(float velocity, float speed_limit, float max_accel_param) {
//    if (velocity < speed_limit) return max_accel_param;
//    return 0;
//}

// original : accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;

//                if ( diff < safety_distance ) { // TODO: Smoother braking
//                    accel_param = -velocity/diff;
//                    accel_param = std::max(min_accel_param,accel_param);
//                } else { // TODO: Implement Accelerating to max road speed: new argument max_road_speed
//                    accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
//                }

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
int Car::update_decision(Road* road, float time_step, int iteration) {
    float car_ahead_pos = road->get_car_ahead_pos(x_pos, y_pos);
    float car_ahead_merged = road->get_car_ahead_merged(car_ahead_pos, y_pos);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-x_pos;
//    if ( x_pos < road->get_begin_event() ) {   // Before Lane Merging
//        if ( diff < safety_distance ) { // TODO: Smoother braking
//            accel_param = -velocity/diff;
//            accel_param = std::max(min_accel_param,accel_param);
//        } else { // TODO: Implement Accelerating to max road speed: new argument max_road_speed
//            accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
//        }
//    } else
    // TODO: Anything before event? -> currently the cars get too close together due to the event
    if ( x_pos >= road->get_begin_event() ) { //}|| ( id > 1 && x_pos >= road->get_begin_event()-20 ) ) {                                        // Begin Lane Merging
        if ( !human ) {                                                                     // Autonomous Vehicles Protocol:
            if ( velocity > target_velocity_merge && !target_velocity_merge_reached ) {     // 1. Slow down to target_velocity
                accel_param = -velocity/target_velocity_merge;
                accel_param = std::max(min_accel_param,accel_param);
                if ( abs(velocity - target_velocity_merge) < 0.1 ) {                          // 1.1 Indicate slow down completed
                    accel_param = 0;
                    target_velocity_merge_reached = true;
                }
            } else if ( merging ) {                                                         // 2. Merging, change lanes
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
                if ( y_pos > 0.5 ) {                             // So that next cars can begin merging, indicate merged once in target lane (but not necessarily with y_pos == 1)
                    merged = true;
                }
                // TODO: Never actually gets to the other lane, this is cheating a bit
                if ( y_target-y_pos<0.001 && turning_angle<10e-13 ) {
                    turning_angle = 0;
                    road_angle = 0;
                    y_pos = 1;
                    merging = false;
                }
                // TODO: Accelerate during lane merge??
                accel_param = fast_acceleration(velocity, road->get_speed_limit(), max_accel_param);
            } else if ( ( merged || accelerating ) && velocity < road->get_speed_limit() ) {                    // 3. Finished merging, accelerate to speed limit
                accel_param = fast_acceleration(velocity, road->get_speed_limit(), max_accel_param);
            } else {                                                                        // 4. AV Decisions
                // AV Merging protocol PROTOCOL
                if ( starting_lane == 0 ) {                                                 // top lane == lane to merge from
                    if ( car_ahead_pos == FLT_MAX && road->check_merging_space(x_pos, y_pos, safety_distance, car_length) ) { // No cars ahead
                        merging = true;
                    }
                } else if ( starting_lane == 1 ) {                                          // bottom lane == lane to merge to
                    if ( car_ahead_pos == FLT_MAX || ( road->get_car_ahead_pos_anylane(x_pos) - x_pos) > safety_distance ) {  // Space ahead
                        accelerating = true;
                    }
                }
            }
            // TODO: factor in safety_distance during all of the above
            // TODO: unused l_merged
            // TODO: don't use circular road
            // TODO: brake more efficiently, get better metric for seeing if at target velocity
        } else { // Humans
            
        }
    }
    return 0;
}

// Initial test
//y_target = 1;
//float psi = atan( (y_target-y_pos)/(horizon - (car_length*cos(road_angle))) ) - road_angle;
//float check = psi-turning_angle;
//if ( check < -max_delta_turning_angle ) {
//    turning_angle = turning_angle - max_delta_turning_angle;
//} else if ( abs(check) < max_delta_turning_angle ) {
//    turning_angle = psi;
//} else if ( check > max_delta_turning_angle ) {
//    turning_angle = turning_angle + max_delta_turning_angle;
//}
//// TODO: Never actually gets to the other lane, this is cheating a bit
//if ( y_target-y_pos<0.001 && turning_angle<10e-13 ) {
//    turning_angle = 0;
//    road_angle = 0;
//    y_pos = 1;
//}

bool Experiment::experiment_finished() {
    return road.cars_at_speed_limit();
}

// Returns the number of the experiment
int main(int argc, char const *argv[]) {
    
    if (argc != 20+1) {
        std::cout << "Usage: lane_merge <init_velocity> <max_velocity> <init_accel_param> <max_accel_param> <min_accel_param> <car_length> <horizon> <max_delta_turning_angle> <reaction_time> <road_length> <speed_limit> <n_lanes> <n_cars_per_lane> <car_spacing> <max_it> <time_step> <visualisation>";
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
    
    Experiment lange_merge(init_vals);
    std::string dir = "./Experiments/lane_change";
    int experiment_num;
    std::string experiment_name = get_file_name(dir,&experiment_num);
    std::string experiment_file_name = dir + "/" + experiment_name;
    lange_merge.main_loop(experiment_file_name, init_n_lanes>1, init_begin_event>0, visualise);

    return experiment_num;
}
