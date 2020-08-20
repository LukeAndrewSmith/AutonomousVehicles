#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>
#include <cfloat>
#include <random>
#include "model.hpp"
#include "write_data.hpp"


// TODO: Check safety distance ahead doesn't include the length of the car!!!!! -> Shouldn't change much so haven't changed yet
float Car::accelerate(Road* road) {
    // I've fucked something up here
    // TODO: Maybe put back 'int car_ahead_id'
    int n_cars_per_lane = road->get_n_cars_per_lane();
    int id_same_lane = id+1;
    int id_other_lane = id+n_cars_per_lane;
    if ( starting_lane == 1 ) id_other_lane = id-n_cars_per_lane;
    if ( id == n_cars_per_lane-1 || id == road->get_cars().size()-1 ) id_same_lane = -1;
//    if ( id == n_cars_per_lane-1 ) id_other_lane = -1;
//    if ( human ) {
//        if ( starting_lane == 0 ) id_other_lane = id+n_cars_per_lane+1;
//        if ( starting_lane == 0 ) id_other_lane = id-n_cars_per_lane+1;
//    }


    float diff_ahead_same_lane = road->get_car_pos_id(id_same_lane)-x_pos+car_length;
    float diff_ahead_other_lane = road->get_car_pos_id(id_other_lane)-x_pos+car_length;
    float diff = std::min(diff_ahead_same_lane,diff_ahead_other_lane); // Match the closer car to avoid crashing and to allow space for the car in front to merge

    float safety_distance = 2*velocity;  // 2[s] * speed in [m/s]
    float tmp = 0;
    float diff_end_event = (road->get_end_event()-safety_distance)-x_pos+car_length;
    float diff_end_event_lane_0 = (road->get_end_event())-x_pos+car_length;
    float speed_limit;
    if ( x_pos < road->get_begin_event() || ( !human && id == road->get_cars().size()-1 && x_pos < road->get_end_event() ) ) {
        speed_limit = road->get_speed_limit();
    } else {
        speed_limit = road->get_speed_limit_after_merge();
    }

    if ( human && starting_lane==1 && x_pos<road->get_end_event() && diff_end_event<safety_distance && !(road->get_car_merging_id(id_other_lane) ||  road->get_car_merged_id(id_other_lane) ) ) diff = diff_end_event;
    if ( human && starting_lane==0 && x_pos<road->get_end_event() && diff_end_event_lane_0<safety_distance && !(merging || merged) ) diff = diff_end_event_lane_0;
    if ( diff < safety_distance ) {
        tmp = (std::abs(braking_strategy_multiplier*min_accel_param)/safety_distance)*diff + braking_strategy_multiplier*min_accel_param; // TODO: Remove previously added '- 0.1'; // TODO: Added -0.1 EXPLAIN
        tmp = std::max(min_accel_param,tmp);
    } else if ( velocity < speed_limit ) {
        if ( accel_param <= 0 ) starting_velocity_accel = velocity;
        float v_scaled = (velocity - starting_velocity_accel) * (M_PI / (speed_limit - starting_velocity_accel));
        tmp = accel_max * (std::sin(v_scaled)*(1-accel_start_proportion) + accel_start_proportion);
    } else if ( velocity > speed_limit+1 ) {
        // TODO: CHECK/OPTIMISE BRAKING, seems to always brake at min_accel_param
        tmp = min_accel_param - std::max(min_accel_param, (std::abs(min_accel_param)/(2*speed_limit))*velocity) + min_accel_param;
        tmp = std::max(min_accel_param,tmp);
    }

    // Overreaction
    if ( human && id != road->get_cars().size()-1 ) {
        if ( !overreacting && accel_param < 0 && previous_accel_param < accel_param && accel_param < tmp ) {
            ovverreaction_count[0]++;
            overreacting = true;
        }
        if ( overreacting ) {
            if ( overreaction_iterations < overreaction_brake_iterations ) {
                if ( tmp < accel_param ) {
                    ovverreaction_count[1]++;
                    overreaction_iterations = 0;
                    overreacting = false;
                } else {
                    overreaction_iterations++;
                    tmp = accel_param;
                }
            } else if ( overreaction_iterations >= overreaction_brake_iterations && overreaction_iterations < overreaction_brake_iterations+overreaction_pause_iterations ) {
                if ( tmp < min_accel_param/2 ) {
                    ovverreaction_count[2]++;
                    overreaction_iterations = 0;
                    overreacting = false;
                } else {
                    overreaction_iterations++;
                    tmp = 0;
                }
            } else {
                overreaction_iterations = 0;
                overreacting = false;
            }
        }
    }

    // Collision detection
    if ( diff <= 0 ) {
        std::cout << "id: " << id << std::endl;
        std::cout << diff_ahead_same_lane << ", " << diff_ahead_other_lane << ", " << diff << ", " << diff_end_event << ", " << diff_end_event_lane_0 << std::endl;
        std::cout << "Collision" << ", ";
        exit(-1);
    }

    return tmp;
}

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
    if ( ahead_space >= safety_distance && behind_space >= safety_distance_2*0.75 ) return true;
    return false;
}

bool Experiment::experiment_finished() {
    return road.cars_at_speed_limit() && road.cars_in_lane_1() && road.cars_at_safety_distance();
//    return road.cars_in_lane_1() && road.cars_at_safety_distance();
}

/*----------------------------------------------------------------------------*/
/*                         Experiment Parameters                              */
/*----------------------------------------------------------------------------*/
/*
NOTE: See Experiment constructor for example of how id's are assigned so that the id manipulation below makes sense
*/
int Car::update_decision(Road* road, float time_step, int iteration, int n_iterations_event) { // NOTE:' n_iterations_event' unused for lane_merge experiments, only used for concertina
//    float car_ahead_pos = road->get_car_ahead_pos(x_pos, y_pos);
//    float diff = car_ahead_pos-x_pos+car_length; // TODO: Just added car length might break everything
    float safety_distance = 2*velocity;
    int event_began = 0;
    float tmp_accel_param = 0;
    // Before Lane Merging
    if ( ( !human && road->get_cars().back().get_x_pos() < road->get_begin_event() ) || ( ( human && road->get_cars().back().get_x_pos() < road->get_begin_event()) && road->has_noticed_merge(id) ) ) {
        int n_cars_per_lane = road->get_n_cars_per_lane();
        int car_to_match_id = id+1;
        if ( starting_lane == 0 ) car_to_match_id = id+1;
        if ( starting_lane == 1 ) car_to_match_id = id-(n_cars_per_lane-1); //TODO: Changed from 'id-(n_cars_per_lane-1)' // Stay next to the car directly next to us
        if ( id == n_cars_per_lane-1 || id == road->get_cars().size()-1 ) car_to_match_id = -1; // No cars ahead of front two vehicles => set invalid id so that the distance to the car ahead is set to FLT_MAX
        tmp_accel_param = accelerate(road);
    // Lane Merging
    } else if ( ( human && x_pos >= road->get_begin_event() ) || !human ) {
        if ( !human ) {
            if ( id == road->get_cars().size() - 1 ) event_began = 1;
            if ( starting_lane == 0 ) {
                int id_to_follow = id + road->get_n_cars_per_lane();
                tmp_accel_param = accelerate(road);
                int av_only_merge_front = road->get_av_only_merge_front();
                int car_ahead_id = id+1;
                if ( id == road->get_n_cars_per_lane()-1 ) car_ahead_id = -1;
//                std::cout << (road->get_car_pos_id(id_to_follow)-x_pos+car_length >= safety_distance) << std::endl;
                std::cout << road->get_car_pos_id(id_to_follow)-x_pos+car_length << ", " << safety_distance << std::endl;
                if ( !merged && !merging && road->get_car_pos_id(id_to_follow)-x_pos+car_length >= safety_distance && ( !av_only_merge_front || ( av_only_merge_front && road->get_car_pos_id(car_ahead_id)==FLT_MAX ) ) ) { // TODO: Extra condition (car_ahead_pos == FLT_MAX) to make only front cars merge
                    merging = true;
                    std::cout << merging << std::endl;
                }
                if ( merging ) merge();
            } else if ( starting_lane == 1 ) {
                int id_to_follow = id - road->get_n_cars_per_lane() ;//+ 1;
                if ( id_to_follow == road->get_n_cars_per_lane() ) id_to_follow = -1; // Front car should have invalid id as there aren't any cars ahead
                tmp_accel_param = accelerate(road);
            }
        } else if ( human ) {
            if ( id == road->get_cars().size() - 1 ) event_began = 1;
            if ( starting_lane == 0 ) {
                if ( !merging && !merged ) {
                    int ahead_id = id+road->get_n_cars_per_lane()+1;
                    int behind_id = id+road->get_n_cars_per_lane();
                    if ( road->check_merging_space(id, ahead_id, behind_id, car_length) ) merging = true;
                }
                if ( merging ) merge();
                tmp_accel_param = accelerate(road);
            } else if ( starting_lane == 1 ) {
                int car_to_match_id = id-road->get_n_cars_per_lane();
                float diff_end_event = (road->get_end_event()-safety_distance)-x_pos+car_length;
                if ( x_pos < road->get_end_event() && ( diff_end_event < safety_distance && !(road->get_car_merging_id(car_to_match_id) ||  road->get_car_merged_id(car_to_match_id)) ) ) {
                    tmp_accel_param = accelerate(road);
                } else
                if ( wait != 0 ) {
                    wait--;
                    if ( wait == 0 ) waited = true;
                    tmp_accel_param = accelerate(road);
                } else if ( wait == 0 && !waited ) {
                    wait = random_num(100,400);
                    tmp_accel_param = accelerate(road);
                } else if ( wait == 0 && waited ) {
                    tmp_accel_param = accelerate(road);
                }
            }
        }
    }

    previous_accel_param = accel_param;
    accel_param = tmp_accel_param;

    return event_began;
}

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

    Experiment lange_merge(init_vals);
    std::string dir = "./Experiments/lane_change";
    int experiment_num;
    std::string experiment_name = get_file_name(dir,&experiment_num);
    std::string experiment_file_name = dir + "/" + experiment_name;
    lange_merge.main_loop(experiment_file_name, init_n_lanes>1, init_begin_event>0, visualise);

    return experiment_num;
}
