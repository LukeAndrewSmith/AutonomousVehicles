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
HORIZON=100                 # float horizon             // UNUSED HERE AS ONLY 1 LANE
MAX_DELTA_TURNING_ANGLE=1   # float init_max_delta_turning_angle
#OVERREACTION_BRAKE_TIME=2.0 # float init_overreaction_brake_time
#OVERREACTION_PAUSE_TIME=2.0 # float init_overreaction_pause_time
BRAKING_STRATEGY_MULTIPLIER=4 # float init_braking_strategy_multiplier
#HUMAN=0                     # int init_human
# Road
ROAD_LENGTH=0               # float init_road_length
SPEED_LIMIT=33              # float init_road_length    // 33 m/s =~ 120km/h
SPEED_LIMIT_AFTER_MERGE=$SPEED_LIMIT  # float init_speed_limit_after_merge
N_LANES=1                   # int init_n_lanes
#BEGIN_EVENT below as uses variables not yet declared
#END_EVENT=0(set below)     # float end_event
AV_ONLY_MERGE_FRONT=0       # int av_only_merge_front
#N_ITERATIONS_EVENT below as uses variables not yet declared
# Experiment
N_CARS_PER_LANE=2          # int init_n_cars_per_lane  // All lanes begin with the same number of cars
CAR_SPACING=$(( 2*$VELOCITY )) # int init_car_spacing
MAX_IT=0 #0                 # int init_max_it
TIME_STEP=0.01              # float init_time_step

BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
END_EVENT=0
N_ITERATIONS_EVENT=$(( 2/$TIME_STEP ));   # Number of seconds for event / time_step => number of iterations
# Visualisation
VISUALISE=0

echo "------------------------------------------------------"
echo "                Concertina Experiments"
echo "------------------------------------------------------"

echo "\n---------------------------------------"
echo "Varying Overreaction Times with Nº Cars"
echo "---------------------------------------"
N_CARS_PER_LANE=20
for (( EXTRA_SPACING=0; EXTRA_SPACING<21; EXTRA_SPACING+=2 ));
do
    echo "\nExtra Spacing: "$EXTRA_SPACING
    CAR_SPACING=$(( 2*$VELOCITY + $EXTRA_SPACING ))
    BEGIN_EVENT=$(( $N_CARS_PER_LANE * ($CAR_SPACING+$CAR_LENGTH) ))
    echo "AV"
    HUMAN=0
    OVERREACTION_BRAKE_TIME=0
    OVERREACTION_PAUSE_TIME=0
    ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE

    echo "\nHuman"
    HUMAN=1
    for ((OVERREACTION_BRAKE_TIME=0.1; OVERREACTION_BRAKE_TIME <= 2.1; OVERREACTION_BRAKE_TIME=OVERREACTION_BRAKE_TIME+0.1));
    do
        echo "\nOVERREACTION_BRAKE_TIME: "$OVERREACTION_BRAKE_TIME
        for ((i=0; i<5; i=i+1));
        do
            ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_BRAKE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
        done
    done
done


#!!!!!!!!!!!!!!!!!!!
# TODO: Currently proving conclusion
# When the cars are as close together as possible, the benifit of the autonomous vehicle is minimal as the concertina will propagate back. The AV comes into it's own however when the vehicle density is greater than the minimum as the AV is able to resolve the concertina before it propagates back.

# Based on the above experiment I have chosen 10m spacing as it has a nice threashold, we will now look at the varying number of cars
EXTRA_SPACING=10
CAR_SPACING=$(( 2*$VELOCITY + $EXTRA_SPACING ))

echo "\n---------------------------------------"
echo "Varying Overreaction Times with Nº Cars"
echo "---------------------------------------"

for ((N_CARS_PER_LANE=10; N_CARS_PER_LANE<=41; N_CARS_PER_LANE=N_CARS_PER_LANE+10));
do
    echo "\nNº Cars: "$N_CARS_PER_LANE
    BEGIN_EVENT=$(( $N_CARS_PER_LANE * ($CAR_SPACING+$CAR_LENGTH) ))
    echo "AV"
    HUMAN=0
    OVERREACTION_BRAKE_TIME=0
    OVERREACTION_PAUSE_TIME=0
    ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE

    echo "\nHuman"
    HUMAN=1
    for ((OVERREACTION_BRAKE_TIME=0.1; OVERREACTION_BRAKE_TIME <= 2.1; OVERREACTION_BRAKE_TIME=OVERREACTION_BRAKE_TIME+0.1));
    do
        echo "\nOVERREACTION_BRAKE_TIME: "$OVERREACTION_BRAKE_TIME
        for ((i=0; i<5; i=i+1));
        do
            ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_BRAKE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
        done
    done
done

echo "\n------------------------------------------------------"
echo "Fine Grain Variation of Overreaction Times for 20 cars"
echo "------------------------------------------------------"

N_CARS_PER_LANE=20
BEGIN_EVENT=$(( $N_CARS_PER_LANE * ($CAR_SPACING+$CAR_LENGTH) ))

echo "\AV"
HUMAN=0
OVERREACTION_BRAKE_TIME=0
OVERREACTION_PAUSE_TIME=0
./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE

echo "\nHuman"
HUMAN=1
for ((OVERREACTION_BRAKE_TIME=0.1; OVERREACTION_BRAKE_TIME <= 2.01; OVERREACTION_BRAKE_TIME=OVERREACTION_BRAKE_TIME+0.02));
do
    echo "\nOVERREACTION_BRAKE_TIME: "$OVERREACTION_BRAKE_TIME
    for ((i=0; i<5; i=i+1));
    do
        ./concertina $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_BRAKE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
    done
done
