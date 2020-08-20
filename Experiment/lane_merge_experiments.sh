#!/bin/zsh

make lane_merge || exit 1

# Values for init_experiment init_vals, See model.hpp for definition
    # Car values based on # Car (tesla model 3)
# Car
VELOCITY=33                 # float init_velocity       // All car begin with the same velocity
MAX_VELOCITY=70             # float init_max_velocity   // 70 m/s =~ 250 km/h
ACCEL_PARAM=0               # float init_accel_param
MAX_ACCEL_PARAM=0.14        # float init_max_accel_param
MIN_ACCEL_PARAM=-0.69       # float init_min_accel_param
CAR_LENGTH=4.69             # float init_length
HORIZON=50                  # float horizon
MAX_DELTA_TURNING_ANGLE=1   # float init_max_delta_turning_angle
#OVERREACTION_BRAKE_TIME(set below) # float init_overreaction_brake_time
#OVERREACTION_PAUSE_TIME(set below) # float init_overreaction_pause_time
BRAKING_STRATEGY_MULTIPLIER=4       # float init_braking_strategy_multiplier
#HUMAN(set below)                     # int init_human
# Road
ROAD_LENGTH=0               # float init_road_length
SPEED_LIMIT=33              # float init_road_length    // 33 m/s =~ 120km/h
SPEED_LIMIT_AFTER_MERGE=16  # float init_speed_limit_after_merge
N_LANES=2                   # int init_n_lanes
#BEGIN_EVENT(set below)     # float begin_event
#END_EVENT(set below)     # float end_event
AV_ONLY_MERGE_FRONT=0       # int av_only_merge_front
N_ITERATIONS_EVENT=0;       # UNUSED HERE, EVENT BASED ON DISTANCE NOT ITERATIONS IN LANE_MERGE
# Experiment
#N_CARS_PER_LANE(set below)           # int init_n_cars_per_lane  // All lanes begin with the same number of cars
CAR_SPACING=$(( (2*$VELOCITY) ))              # int init_car_spacing
MAX_IT=0                    # int init_max_it
TIME_STEP=0.01              # float init_time_step
# Visualisation
VISUALISE=0
# Road cont
#BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
#END_EVENT=$(( $BEGIN_EVENT+200 ))


echo "------------------------------------------------------"
echo "                Lane Merge Experiments"
echo "------------------------------------------------------"

echo "\n---------------"
echo "Varying Nº Cars"
echo "---------------"

echo "\n---------------------------"
echo "Human"
HUMAN=1
OVERREACTION_BRAKE_TIME=1
OVERREACTION_PAUSE_TIME=1
for ((N_CARS_PER_LANE=5; N_CARS_PER_LANE <= 30.1; N_CARS_PER_LANE=N_CARS_PER_LANE+5));
do
    echo "\nNº Cars per lane: "$N_CARS_PER_LANE
    for ((i=0; i<5; i=i+1))
    do
        BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
        END_EVENT=$(( $BEGIN_EVENT+200 ))
        ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
    done
done

echo "\n---------------------------"
echo "AV"
HUMAN=0
OVERREACTION_BRAKE_TIME=0
OVERREACTION_PAUSE_TIME=0
for ((N_CARS_PER_LANE=5; N_CARS_PER_LANE <= 30.2; N_CARS_PER_LANE=N_CARS_PER_LANE+5));
do
    echo "\nNº Cars per lane: "$N_CARS_PER_LANE
    echo "AV: Front Car Merging"
    AV_ONLY_MERGE_FRONT=1
    BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
    END_EVENT=$(( $BEGIN_EVENT+200 ))
    ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
    echo "\nAV: All Cars Merging"
    AV_ONLY_MERGE_FRONT=0
    BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
    END_EVENT=$(( $BEGIN_EVENT+200 ))
    ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
done


echo "\n------------------------------------"
echo "Varying Speed Limit After Lane Merge"
echo "------------------------------------"
N_CARS_PER_LANE=10
BEGIN_EVENT=$(( $N_CARS_PER_LANE*($CAR_SPACING+$CAR_LENGTH) ))
END_EVENT=$(( $BEGIN_EVENT+200 ))

for (( SPEED_LIMIT_AFTER_MERGE=11; SPEED_LIMIT_AFTER_MERGE<=34; SPEED_LIMIT_AFTER_MERGE+=2 ))
do
    echo "\n\nSpeed limit after merge: "$SPEED_LIMIT_AFTER_MERGE
    echo "Human"
    HUMAN=1
    OVERREACTION_BRAKE_TIME=1
    OVERREACTION_PAUSE_TIME=1
    for ((i=0; i<5; i=i+1))
    do
        ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
    done
    echo "\nAV"
    HUMAN=0
    OVERREACTION_BRAKE_TIME=0
    OVERREACTION_PAUSE_TIME=0
    echo "AV: Front Car Merging"
    AV_ONLY_MERGE_FRONT=1
    ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
    echo "\nAV: All Cars Merging"
    AV_ONLY_MERGE_FRONT=0
    ./lane_merge $VELOCITY $MAX_VELOCITY $ACCEL_PARAM $MAX_ACCEL_PARAM $MIN_ACCEL_PARAM $CAR_LENGTH $HORIZON $MAX_DELTA_TURNING_ANGLE $OVERREACTION_BRAKE_TIME $OVERREACTION_PAUSE_TIME $BRAKING_STRATEGY_MULTIPLIER $HUMAN $ROAD_LENGTH $SPEED_LIMIT $SPEED_LIMIT_AFTER_MERGE $N_LANES $BEGIN_EVENT $END_EVENT $AV_ONLY_MERGE_FRONT $N_ITERATIONS_EVENT $N_CARS_PER_LANE $CAR_SPACING $MAX_IT $TIME_STEP $VISUALISE
done


#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#TODO:
#Lane merge effects: change human image, now they stop completely :)
#Go through TODO's in code
#Remove unused references in bib
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
