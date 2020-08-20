#!/bin/zsh

#  concertina.sh
#  Autonomous-Vehicles
#
#  Created by Luke Smith on 08.04.20.
#  Copyright © 2020 Luke Smith. All rights reserved.

make concertina || exit 1

# Values for init_experiment init_vals, See model.hpp for definition
    # Car values based on # Car (tesla model 3)
# Car
VELOCITY=33                 # float init_velocity       // All car begin with the same velocity
MAX_VELOCITY=70             # float init_max_velocity   // 70 m/s =~ 250 km/h
ACCEL_PARAM=0               # float init_accel_param
MAX_ACCEL_PARAM=0.14        # float init_max_accel_param
MIN_ACCEL_PARAM=-0.69       # float init_min_accel_param
CAR_LENGTH=4.69             # float init_car_length
HORIZON=100                 # UNUSED HERE AS ONLY 1 LANE
MAX_DELTA_TURNING_ANGLE=1   # float init_max_delta_turning_angle
OVERREACTION_BRAKE_TIME=2.0 # float init_overreaction_brake_time
OVERREACTION_PAUSE_TIME=2.0 # float init_overreaction_pause_time
BRAKING_STRATEGY_MULTIPLIER=1       # float init_braking_strategy_multiplier
HUMAN=0                     # UNUSED HERE AS CONCERTINA
# Road
ROAD_LENGTH=0               # float init_road_length
SPEED_LIMIT=33              # float init_road_length    // 33 m/s =~ 120km/h
SPEED_LIMIT_AFTER_MERGE=$SPEED_LIMIT  # float init_speed_limit_after_merge
N_LANES=1                   # int init_n_lanes
#BEGIN_EVENT below as uses variables not yet declared
END_EVENT=0                 # UNUSED HERE AS ONLY 1 LANE
AV_ONLY_MERGE_FRONT=0       # UNUSED HERE AS ONLY 1 LANE
#N_ITERATIONS_EVENT below as uses variables not yet declared
# Experiment
N_CARS_PER_LANE=5 #10 #0          # int init_n_cars_per_lane  // All lanes begin with the same number of cars
CAR_SPACING=$(( 2*$VELOCITY ))              # int init_car_spacing
MAX_IT=15000                    # int init_max_it
TIME_STEP=0.01              # float init_time_step

BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))            # float begin_event, ( 0 ==> No event )
N_ITERATIONS_EVENT=$(( 5/$TIME_STEP ));   # Number of seconds for event / time_step => number of iterations
# Visualisation
VISUALISE=0 #1

EXP_NUM=$? # Get the return value = experiment_num, 0 => failure (not conventional but there we go)

# FULL TEST
#----------
#mult=(1.5 2 3 4)
#for BRAKING_STRATEGY_MULTIPLIER in $mult;
#do
#    echo $'\n'
#    echo $BRAKING_STRATEGY_MULTIPLIER
#    echo $'\n'
#    for ((REACTION_TIME=0; REACTION_TIME <= 1.25; REACTION_TIME=REACTION_TIME+0.01)); # 0.005
#    do
#        ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
#    done
#done


# TEST ONLY ONE BRAKING STRATEGY
#-------------------------------
#BRAKING_STRATEGY_MULTIPLIER=4
#for ((REACTION_TIME=0; REACTION_TIME <= 1.25; REACTION_TIME=REACTION_TIME+0.1)); # 0.005
#do
#    ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
#done

# TEST MULTIPLE BRAKING STRATEGIES BUT LESS REACTION TIMES
#---------------------------------------------------------
#HUMAN=1
#BRAKING_STRATEGY_MULTIPLIER=4
#mult=4
#for ((REACTION_TIME=0.1; REACTION_TIME <= 1.25; REACTION_TIME=REACTION_TIME+0.1)); # 0.005
#do
#    ./concertina $VELOCITY $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
#done
