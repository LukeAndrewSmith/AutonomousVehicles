#!/bin/sh

#  concertina.sh
#  Autonomous-Vehicles
#
#  Created by Luke Smith on 08.04.20.
#  Copyright © 2020 Luke Smith. All rights reserved.

make concertina

# Values for init_experiment init_vals, See model.hpp for definition
    # Car values based on # Car (tesla model 3)
# Car
VELOCITY=33                 # float init_velocity       // All car begin with the same velocity
MAX_VELOCITY=70             # float init_max_velocity   // 70 m/s =~ 250 km/h
ACCEL_PARAM=0               # float init_accel_param
MAX_ACCEL_PARAM=0.14        # float init_max_accel_param
MIN_ACCEL_PARAM=-0.69       # float init_min_accel_param
CAR_LENGTH=4.69             # float init_car_length
HORIZON=100                 # float horizon             // UNUSED HERE AS ONLY 1 LANE
MAX_DELTA_TURNING_ANGLE=1   # float init_max_delta_turning_angle
REACTION_TIME=0 #.25           # float init_reaction_time  // 0 for autonomous vehicles
# Road
ROAD_LENGTH=0               # float init_road_length
SPEED_LIMIT=33              # float init_road_length    // 33 m/s =~ 120km/h
N_LANES=1                   # int init_n_lanes
#BEGIN_EVENT below as uses variables not yet declared
# Experiment
N_CARS_PER_LANE=20          # int init_n_cars_per_lane  // All lanes begin with the same number of cars
CAR_SPACING=$(( 2*$VELOCITY ))              # int init_car_spacing
MAX_IT=0                    # int init_max_it
TIME_STEP=0.01              # float init_time_step

BEGIN_EVENT=$(( ($N_CARS_PER_LANE+1)*$CAR_SPACING ))            # float begin_event
#BEGIN_EVENT=0 # No event

./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $REACTION_TIME $ROAD_LENGTH $SPEED_LIMIT $N_LANES $BEGIN_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP

EXP_NUM=$? # Get the return value = experiment_num, 0 => failure (not conventional but there we go)
if [ $EXP_NUM > 0 ]
then
    magick convert ./Experiments/concertina/experiment_$EXP_NUM.pbm ./Experiments/concertina/experiment_$EXP_NUM.png # Convert to png as pbm files become massive, but they are much easier to write from c++
    rm ./Experiments/concertina/experiment_$EXP_NUM.pbm
    echo "Concertina experiment "$EXP_NUM": Completed Succesfully"
fi
