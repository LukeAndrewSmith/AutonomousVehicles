#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <cfloat>
#include <random>
#include "model.hpp"
#include "write_data.hpp"


// Helpers

void id_says(std::string to_say, int target_id, int id) {
    if ( id == target_id ) {
        std::cout << to_say << std::endl;
    }
}

// TODO: Check safety distance ahead doesn't include the length of the car!!!!!
float Car::accelerate(Road* road, int car_ahead_id) {
    float diff = road->get_car_pos_id(car_ahead_id)-x_pos;
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float tmp;
    if ( diff < safety_distance ) {
        tmp = -velocity/diff;
        tmp = std::max(min_accel_param,tmp);
    } else {
        tmp = (1-(velocity/road->get_speed_limit()))*max_accel_param;
        if ( at_target_velocity(road->get_speed_limit()) ) tmp = 0; // Ensure convergence to 0 in reasonable time
    }
    return tmp;
}

// TODO: Decision buffer not used for turning_angle
void Car::merge() {
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
    if ( in_target_lane() ) {                                           // Ensure convergence in reasonable time
        turning_angle = 0;
        road_angle = 0;
        y_pos = 1;
        merging = false;
    }
    
}

bool Road::check_merging_space(int my_id, int id_ahead, int id_behind, float car_length) {
    float safety_distance = 2*get_velocity_id(my_id);
    float safety_distance_2 = 2*get_velocity_id(id_behind);
    float ahead_space = get_car_pos_id(id_ahead) - get_car_pos_id(my_id);
    float behind_space = get_car_pos_id(my_id) - get_car_pos_id(id_behind);
    if ( ahead_space >= safety_distance && behind_space >= safety_distance_2 ) return true;
    return false;
}

bool Experiment::experiment_finished() {
//    std::cout << road.cars_at_speed_limit() <<  road.cars_in_lane_1() << std::endl;
    return road.cars_at_speed_limit() && road.cars_in_lane_1();
}

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
update_decision
TODO: Explanation
*/
int check = 0;
int Car::update_decision(Road* road, float time_step, int iteration) {
    float car_ahead_pos = road->get_car_ahead_pos(x_pos, y_pos);
    float car_ahead_merged = road->get_car_ahead_merged(car_ahead_pos, y_pos);
    float safety_distance = 2*velocity; // 2[s] * speed in [m/s]
    float diff = car_ahead_pos-x_pos;
    int event_began = 0;
    float tmp_accel_param = 0; // Only used for humans
    // TODO: factor in safety_distance
    // TODO: brake more efficiently, get better metric for seeing if at target velocity
    if ( x_pos < road->get_begin_event() ) {   // Before Lane Merging
        if ( !human ) {
            int n_cars_per_lane = road->get_n_cars_per_lane();                          // Cars keep side by side before merging so both lanes follow the speed of the car in front in lane 0
            int car_to_match_id;                                                        // See thesis for explanation TODO: Include explanation in thesis
            if ( starting_lane == 0 ) car_to_match_id = id+1;
            if ( starting_lane == 1 ) car_to_match_id = id-(n_cars_per_lane-1);
            if ( id == n_cars_per_lane-1 || id == (2*n_cars_per_lane)-1 ) car_to_match_id = -1; // No cars ahead of front to vehicles
            float diff_merging = road->get_car_pos_id(car_to_match_id)-x_pos;
            if ( diff_merging < safety_distance ) {
                tmp_accel_param = -(velocity/diff_merging);
                tmp_accel_param = std::max(min_accel_param,tmp_accel_param);
                tmp_accel_param = std::min(-0.001f,tmp_accel_param);                    // Ensure that if the cars are side by side the car brakes
            } else {
                tmp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
            }
        } else if ( human ) {
            if ( diff < safety_distance ) {
                tmp_accel_param = -velocity/diff;
                tmp_accel_param = std::max(min_accel_param,tmp_accel_param);
            } else {
                tmp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
            }
        }
    } else if ( x_pos >= road->get_begin_event() ) {                                        // Begin Lane Merging
        if ( !human ) {                                                                     // Autonomous Vehicles Protocol:
            if ( id == road->get_cars().size() - 1 ) event_began = 1; // Indicate that the lane merge has begun so that main_loop() can begin testing for the ending condition of the experiment
            if ( !target_velocity_merge_reached && velocity >= target_velocity_merge ) {    // 1. Everyong Slow down to target_velocity
                tmp_accel_param = -velocity/target_velocity_merge;                              // Aim to slow down to target velocity
                tmp_accel_param = std::max(min_accel_param,tmp_accel_param);
                if ( abs(velocity - target_velocity_merge) < 0.1 ) {                        // 1.1 Indicate slow down completed
                    tmp_accel_param = 0;
                    target_velocity_merge_reached = true;
                }
            } else {
                // AV Merging protocol PROTOCOL
                if ( starting_lane == 0 ) {                                                 // 2. Top lane == lane to merge from
                    int id_beside = id + road->get_n_cars_per_lane();
                    if ( !merged && !merging && car_ahead_pos == FLT_MAX && road->get_car_pos_id(id_beside)-x_pos > safety_distance ) { // 2.1 Begin merging
                        merging = true;
                    }
                    if ( merging ) merge();                                                 // 2.2 Merging
                    if ( merging || merged ) tmp_accel_param = accelerate(road, id_beside);
                } else if ( starting_lane == 1 ) {                                           // bottom lane == lane to merge to
                    int id_ahead_other_lane = id - road->get_n_cars_per_lane() + 1;
                    if ( id_ahead_other_lane == road->get_n_cars_per_lane() ) id_ahead_other_lane = -1; // Front car should have invalid id as there aren't any cars ahead
                    if ( !accelerating && ( car_ahead_pos == FLT_MAX || ( road->get_car_merging_id(id_ahead_other_lane) && road->get_car_pos_id(id_ahead_other_lane)-x_pos > safety_distance ) ) ) accelerating = true; // Space ahead
                    if ( accelerating ) tmp_accel_param = accelerate(road, id_ahead_other_lane); // 3. Finished merging, accelerate to speed limit
                }
            }
        } else if ( human ) {                                                           // Humans
            if ( id == road->get_cars().size() - 1 ) event_began = 1; // Indicate that the lane merge has begun so that main_loop() can begin testing for the ending condition of the experiment
            if ( starting_lane == 0 ) {                                                 // top lane == lane to merge from
                if ( !merging && !merged ) {   // Space => Merge
                    int ahead_id = id+road->get_n_cars_per_lane()+1;                     // See thesis for explanation TODO: Include explanation in thesis
                    int behind_id = id+road->get_n_cars_per_lane();
                    if ( road->check_merging_space(id, ahead_id, behind_id, car_length) ) merging = true;
                }
                if ( merging ) merge();                                                         // 2. Merging, change lanes
                int car_to_match_id = id+road->get_n_cars_per_lane()+1; // Else Match speed of car ahead other lane, See thesis for explanation TODO: Include explanation in thesis
                tmp_accel_param = accelerate(road, car_to_match_id);
            } else if ( starting_lane == 1 ) {                                           // bottom lane == lane to merge to
                int car_ahead_id = id-road->get_n_cars_per_lane();                                  // See thesis for explanation TODO: Include explanation in thesis
                if ( !road->get_car_merging_id(car_ahead_id) ) {
                   if ( wait != 0 ) {
                       wait--;
                       if ( wait == 0 ) waited = true;
                       tmp_accel_param = accelerate(road, id+1);
                   } else if ( wait == 0 && !waited ) {
//                       wait = random_num(10,20);
                       wait = random_num(100,400);
                       tmp_accel_param = accelerate(road, id+1);
                   } else if ( wait == 0 && waited ) {
                       int car_to_match_id = id-road->get_n_cars_per_lane();                     // See thesis for explanation TODO: Include explanation in thesis
                       float diff_merging = road->get_car_pos_id(car_to_match_id)-x_pos;
                       if ( diff_merging < safety_distance ) {
                           tmp_accel_param = -(velocity/diff_merging);
                           tmp_accel_param = std::max(min_accel_param,tmp_accel_param);  // min_accel_param/2 for more leisurly braking which is common when lane merging
                           tmp_accel_param = std::min(-0.001f,tmp_accel_param);
                       } else {
                           tmp_accel_param = (1-(velocity/road->get_speed_limit()))*max_accel_param;
                           if ( at_target_velocity(road->get_speed_limit()) ) tmp_accel_param = 0; // Ensure convergence to 0 in reasonable time
                       }
                   }
                } else {
                    tmp_accel_param = accelerate(road, car_ahead_id);
                }
            }
        }
    }
    
    decision_buffer.push(tmp_accel_param);
    accel_param = decision_buffer.front();
    decision_buffer.pop();
    
    return event_began;
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
