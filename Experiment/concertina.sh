#!/bin/sh

#  concertina.sh
#  Autonomous-Vehicles
#
#  Created by Luke Smith on 08.04.20.
#  Copyright © 2020 Luke Smith. All rights reserved.


./concertina
# Values for init_experiment init_vals, See model.hpp for definition
# Car (tesla model 3)
  std::vector<float>(20,30)#{33,32,31,30,29,28,27,25,24,23,22,21,20,22,22} # std::vector<float> init_velocity; # TODO: Won't work for multi-lane
  33                    # float init_max_velocity;
  0                     # float init_accel_param;
  0.14                  # float init_max_accel_param;
  -0.69                 # float init_min_accel_param;
  4.69                  # float init_length;
  100                   # float horizon;
  1                     # float init_max_delta_turning_angle;
  # Road
  10000                 # float init_road_delim_x;
  1                     # int init_n_lanes;
  1                     # int init_inlet_protocol; # 0 = none, 1 = loop, 2 = new cars
  # Experiment
  20,0  # std::vector<int> init_n_cars_per_lane; # n_cars_per_lane[0] = #cars in lane at lanes[0] // TODO: make sure inputworks
  10                    # int init_spacing_protocol; # TODO: Rename? Currently used as distance between cars
  10000                 # int init_max_it;
  0.01                   # float init_time_step;


magick convert ./Experiments/concertina/experiment_3.pbm experiment_3.png
