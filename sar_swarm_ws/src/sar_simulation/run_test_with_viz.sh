#!/bin/bash
source /opt/ros/humble/setup.bash
if [ -f /root/sar_swarm_ws/install/setup.bash ]; then
    source /root/sar_swarm_ws/install/setup.bash
fi

# Function to cleanup background processes
cleanup() {
    echo "Cleaning up..."
    pkill -f python3
    exit
}

trap cleanup SIGINT SIGTERM

echo "Starting Mock Simulation..."
python3 /root/sar_swarm_ws/src/sar_simulation/mock_drone_sim.py > /tmp/mock_sim.log 2>&1 &
sleep 2

echo "Starting Swarm Controllers..."
python3 /root/sar_swarm_ws/src/sar_swarm_control/swarm_controller.py 1 > /tmp/swarm_1.log 2>&1 &
sleep 2
python3 /root/sar_swarm_ws/src/sar_swarm_control/swarm_controller.py 2 1 > /tmp/swarm_2.log 2>&1 &
sleep 2

echo "Starting Visualizer..."
python3 /root/sar_swarm_ws/src/sar_simulation/visualize_swarm.py &
V_PID=$!
sleep 2

echo "Running Test Flight..."
python3 /root/sar_swarm_ws/src/sar_simulation/test_swarm_flight.py > /tmp/test_flight.log 2>&1

# Check test result
if grep -q "TEST PASSED" /tmp/test_flight.log; then
    echo "TEST PASSED"
else
    echo "TEST FAILED (see /tmp/test_flight.log)"
fi

echo "Test finished. Waiting 5 seconds for visualizer to show final state..."
sleep 5

kill $V_PID
pkill -f python3
