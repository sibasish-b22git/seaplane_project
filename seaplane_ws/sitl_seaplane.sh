#!/bin/bash  # OS shebang line for bash execution in Linux terminal

# ==============================================================================  # SECTION: USER PATH CONFIGURATION
# INSTRUCTIONS: Replace bracketed placeholders below with your absolute paths.    # Setup instructions for successors
# Example: ARDUPILOT_DIR="/home/developer/ardupilot" or "~/ardupilot"             # Concrete example of absolute path
# ==============================================================================  # End of configuration block

ARDUPILOT_DIR="[PATH_TO_YOUR_ARDUPILOT_CLONE]"    # Absolute path to ArduPilot repository root (here, ~/ardupilot)
WORKSPACE_DIR="[PATH_TO_YOUR_SEAPLANE_WS]"        # Absolute path to ROS 2 seaplane workspace (here, ~/seaplane_ws)
MODELS_DIR="[PATH_TO_YOUR_GAZEBO_FOLDER]"    # Absolute path to Gazebo custom aircraft/world models (path tothe "gazebo" folder provided with the manual)

echo "Booting Autonomous Seaplane Simulation Environment..."  # Print startup banner to console

cleanup() {  # Function definition to cleanly terminate background processes upon exit
    echo ""                                     # Print empty line for visual terminal spacing
    echo "Shutting down all systems..."         # Print shutdown initiation message
    
    # Force-kill all simulation, bridge, and physics processes; silence missing process errors
    killall -9 sim_vehicle.py mavproxy.py ruby gz-sim-server gz-sim-gui mavros_node parameter_bridge 2>/dev/null
    
    exit 0  # Exit shell script cleanly with success status code 0
}  # End of cleanup function definition

trap cleanup SIGINT  # Trap Ctrl+C (SIGINT) from keyboard and route execution to cleanup()

# 1. Launch ArduPilot SITL in a new window
echo "-> Starting ArduPilot Brain..."  # Status update indicating SITL startup
# Launch ArduPlane SITL with JSON physics backend, ground station console, and UDP telemetry output
gnome-terminal --title="ArduPilot SITL" -- bash -ic "cd ${ARDUPILOT_DIR}/Tools/autotest && sim_vehicle.py -v ArduPlane -f plane --model JSON --console --map --out udp:127.0.0.1:14551"

echo "-> Waiting for SITL to initialize..."  # Status update indicating script pause
sleep 10                                     # Pause 10 seconds to allow MAVProxy and UDP ports to open

# 2. Launch Gazebo Harmonic in a new window
echo "-> Starting Gazebo Physics Engine..."  # Status update indicating Gazebo startup
# Export custom plugin/model paths and launch Gazebo Harmonic with Cessna world
gnome-terminal --title="Gazebo Harmonic" -- bash -ic "export GZ_SIM_SYSTEM_PLUGIN_PATH=${WORKSPACE_DIR}/src/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH && export GZ_SIM_RESOURCE_PATH=${MODELS_DIR}/models:${MODELS_DIR}/worlds:\$GZ_SIM_RESOURCE_PATH && gz sim -v4 -r cessna_runway_ground_effect.sdf"

echo "-> Waiting for Physics Sync..."  # Status update indicating pause for network sync
sleep 2                                # Pause 2 seconds to allow ArduPilot and Gazebo bridge to lock in

# 3. Launch MAVROS in a new window (Using Binary Install syntax)
echo "-> Starting MAVROS Bridge..."  # Status update indicating MAVROS startup
# Source binary ROS 2 environment and launch MAVROS node connected via TCP to ArduPilot SITL
gnome-terminal --title="MAVROS" -- bash -ic "source /opt/ros/jazzy/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:='tcp://127.0.0.1:5762'"

# 4. Launch Gazebo-to-ROS 2 Camera Bridge
echo "-> Starting Camera Bridge..."  # Status update indicating camera bridge startup
# Source ROS 2/workspace overlays and launch parameter bridge using custom YAML configuration file
gnome-terminal --title="Camera Bridge" -- bash -ic "source /opt/ros/jazzy/setup.bash && source ${WORKSPACE_DIR}/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=${WORKSPACE_DIR}/src/auto_plane/config/ros_camera_bridge.yaml"

echo "✅ All systems launched successfully!"  # Final confirmation that all simulation windows booted

while true; do  # Infinite while loop to keep master script running in foreground
    sleep 1     # Sleep 1 second per loop to conserve CPU cycles while waiting for Ctrl+C
done            # End of infinite loop
