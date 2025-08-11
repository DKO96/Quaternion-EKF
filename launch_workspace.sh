#!/bin/bash

echo "Starting ros2 jazzy imu workspace"
xhost +
docker compose up -d sim
docker compose exec sim bash