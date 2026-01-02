#!/bin/bash

# Configuration
WS_ROOT="/root/sar_swarm_ws"
SIM_DIR="$WS_ROOT/src/sar_simulation"
CONTROL_DIR="$WS_ROOT/src/sar_swarm_control"

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source $WS_ROOT/install/setup.bash

# Export DISPLAY if not set (should be inherited from env)
export DISPLAY=${DISPLAY:-:0}

# Kill any existing python processes on exit
trap "pkill -f python3" EXIT

echo "Starting Mock Drone Simulation..."
python3 $SIM_DIR/mock_drone_sim.py > /tmp/mock_drone_sim.log 2>&1 &
sleep 2

echo "Starting Swarm Controllers..."
python3 $CONTROL_DIR/swarm_controller.py 1 > /tmp/swarm_controller_1.log 2>&1 &
python3 $CONTROL_DIR/swarm_controller.py 2 1 > /tmp/swarm_controller_2.log 2>&1 &
sleep 2

echo "Starting Graphical Visualizer..."
# We run the visualizer in the background but it needs to stay alive
python3 $SIM_DIR/visualize_swarm_graphical.py &
VIZ_PID=$!
sleep 2

echo "Starting Flight Test..."
python3 $SIM_DIR/test_swarm_flight.py

echo "Flight Test Finished. Waiting for visualizer window to be closed or timeout..."
# Wait for the visualizer to finish (user closes window) or just exit after test
wait $VIZ_PID
