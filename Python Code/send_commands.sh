#!/bin/bash

# === Configuration ===
# Remote device SSH credentials and ROS environment setup
REMOTE_HOST="192.168.228.249"
USERNAME="ubuntu"
REMOTE_ROS_SETUP="/opt/ros/humble/setup.bash"   # Change 'humble' to your ROS2 distro if needed

# File containing commands (each line: keyword value)
FILE="commands.txt"

# === Function to send a ROS2 command via SSH ===
send_cmd() {
  local cmd="$1"
  # Execute the command on the remote device in a bash shell,
  # sourcing the ROS2 environment first.
  ssh "${USERNAME}@${REMOTE_HOST}" "bash -c 'source ${REMOTE_ROS_SETUP} && ${cmd}'"
}

# === Read commands from file and publish them ===
while IFS= read -r line || [[ -n "$line" ]]; do
  # Skip empty lines
  [[ -z "$line" ]] && continue

  # Assume the line is formatted as: keyword value
  keyword=$(echo "$line" | awk '{print $1}')
  value=$(echo "$line" | awk '{print $2}')

  # Build the ROS2 command based on the keyword
  case "$keyword" in
    forward)
      # Set linear.x = value
      ros_command="ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: ${value}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -1"
      ;;
    backward)
      # Set linear.x = -value
      ros_command="ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: -${value}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -1"
      ;;
    left)
      # Set angular.z = value
      ros_command="ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${value}}}\" -1"
      ;;
    right)
      # Set angular.z = -value
      ros_command="ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -${value}}}\" -1"
      ;;
    stop)
      # Stop motion: set both linear and angular to zero
      ros_command="ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -1"
      ;;
    *)
      echo "Unrecognized command: $line"
      continue
      ;;
  esac

  echo "Sending: $ros_command"
  send_cmd "$ros_command"

  # Optional: wait a moment between commands
  sleep 1

done < "$FILE"
