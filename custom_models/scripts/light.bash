#!/bin/bash

green_light_sdf='/home/luqman/ros2_workspace/src/custom_models/models/lights/green_light/model.sdf'
red_light_sdf='/home/luqman/ros2_workspace/src/custom_models/models/lights/red_light/model.sdf'
yellow_light_sdf='/home/luqman/ros2_workspace/src/custom_models/models/lights/yellow_light/model.sdf'

ros2 run custom_models sdf_spawner_node $red_light_sdf red_light 0 0 0
sleep 7.5
ros2 run custom_models sdf_spawner_node $yellow_light_sdf yellow_light 0 0 0
sleep 1
ros2 run custom_models sdf_spawner_node $green_light_sdf  green_Light 0 0 0

