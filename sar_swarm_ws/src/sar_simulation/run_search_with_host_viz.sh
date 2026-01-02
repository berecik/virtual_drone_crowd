#!/bin/bash

# Configuration
WS_ROOT="/root/sar_swarm_ws"
SIM_DIR="$WS_ROOT/src/sar_simulation"
CONTROL_DIR="$WS_ROOT/src/sar_swarm_control"

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
if [ -f $WS_ROOT/install/setup.bash ]; then
    source $WS_ROOT/install/setup.bash
fi

# Kill any existing python processes on exit
trap "pkill -f python3" EXIT

echo "Starting Mock Drone Simulation for 4 drones..."
python3 $SIM_DIR/mock_drone_sim.py > /tmp/mock_drone_sim.log 2>&1 &
sleep 2

echo "Starting Swarm Controllers..."
# Drone 1 follows global goal
python3 $CONTROL_DIR/swarm_controller.py 1 > /tmp/swarm_controller_1.log 2>&1 &
# Drone 2 follows Drone 1 with offset [2, 0, 0]
python3 $CONTROL_DIR/swarm_controller.py 2 1 > /tmp/swarm_controller_2.log 2>&1 &
# Drone 3 follows Drone 1 with offset [0, 2, 0]
python3 $CONTROL_DIR/swarm_controller.py 3 1 > /tmp/swarm_controller_3.log 2>&1 &
# Drone 4 follows Drone 1 with offset [2, 2, 0]
python3 $CONTROL_DIR/swarm_controller.py 4 1 > /tmp/swarm_controller_4.log 2>&1 &
sleep 2

echo "Starting UDP Relay..."
python3 $SIM_DIR/visualize_udp_relay.py > /tmp/udp_relay.log 2>&1 &
sleep 2

echo "Starting Search Task..."
python3 $SIM_DIR/search_task.py
