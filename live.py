import paramiko
import re
import time

# SSH Credentials (Update these)
HOST = "192.168.228.249"  # Raspberry Pi IP
USERNAME = "ubuntu"
PASSWORD = "turtlebot"

# Define movement commands and velocity mapping
commands = {
    "forward": (0.2, 0),   # (linear_x, angular_z)
    "backward": (-1, 0),
    "left": (0, 1),
    "right": (0, -1),
    "stop": (0, 0),
}

# Function to parse command lines
def parse_command(line):
    match = re.search(r"(forward|backward|left|right|stop)\s*([\d\.]*)", line, re.IGNORECASE)
    if match:
        keyword = match.group(1).lower()
       # value = float(match.group(2)) if match.group(2) else 0
        value =1.0
        return keyword, value
    return None, None

# Connect to Raspberry Pi via SSH
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(HOST, username=USERNAME, password=PASSWORD)

# Monitor the file for new lines
file_path = r"C:\Users\AMREEN\OneDrive - Indian Institute of Technology Jodhpur\Desktop\SEM 4\BCI\commands.txt"
print(f"Monitoring {file_path} for new commands...")

try:
    with open(file_path, "r") as file:
        file.seek(0, 2)  # Move to the end of file

        while True:
            line = file.readline()
            if not line:
                time.sleep(0.5)  # Wait for new data
                continue

            keyword, value = parse_command(line)
            if keyword:
                linear_x = commands[keyword][0] * value
                angular_z = commands[keyword][1] * value

                # Construct ROS command
               # ros_command = 'ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'-1
                ros_command = "source /opt/ros/humble/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" -1"

                
                # Execute command on Raspberry Pi via SSH
                stdin, stdout, stderr = ssh.exec_command(ros_command)
                print(f"Sent: {ros_command}")

except FileNotFoundError:
    print(f"Error: {file_path} not found!")

finally:
    ssh.close()
