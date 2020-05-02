#!/bin/sh

#  lanemerge.sh
#  Autonomous-Vehicles
#
#  Created by Luke Smith on 25.04.20.
#  Copyright © 2020 Luke Smith. All rights reserved.

make lane_merge

# Values for init_experiment init_vals, See model.hpp for definition
    # Car values based on # Car (tesla model 3)
# Car
VELOCITY=30                 # float init_velocity       // All car begin with the same velocity
MAX_VELOCITY=70             # float init_max_velocity   // 70 m/s =~ 250 km/h
ACCEL_PARAM=0               # float init_accel_param
MAX_ACCEL_PARAM=0.14        # float init_max_accel_param
MIN_ACCEL_PARAM=-0.69       # float init_min_accel_param
CAR_LENGTH=4.69             # float init_length
HORIZON=50                  # float horizon
MAX_DELTA_TURNING_ANGLE=1   # float init_max_delta_turning_angle
# Road
ROAD_LENGTH=1000            # float init_road_length
SPEED_LIMIT=33              # float init_road_length    // 33 m/s =~ 120km/h
N_LANES=2                   # int init_n_lanes
BEGIN_EVENT=250             # float begin_event
# Experiment
N_CARS_PER_LANE=5           # int init_n_cars_per_lane  // All lanes begin with the same number of cars
CAR_SPACING=50              # int init_car_spacing
MAX_IT=5000                 # int init_max_it
TIME_STEP=0.01              # float init_time_step

./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $ROAD_LENGTH $SPEED_LIMIT $N_LANES $BEGIN_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP

EXP_NUM=$? # Get the return value = experiment_num, 0 => failure (not conventional but there we go)
echo "Lane_Merge experiment "$EXP_NUM": Completed Succesfully"
