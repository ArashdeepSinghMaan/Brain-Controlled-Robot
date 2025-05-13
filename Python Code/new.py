import paramiko
import time
import os

# SSH connection setup
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.228.249', username='ubuntu', password='turtlebot')

# Function to transfer file
def transfer_file():
    sftp = ssh.open_sftp()
    sftp.put(r"C:\Users\AMREEN\OneDrive - Indian Institute of Technology Jodhpur\Desktop\SEM 4\BCI\commands.txt", '/home/ubuntu/file.txt')
    sftp.close()

# Watch for file changes and transfer
def watch_file():
    last_modified = os.path.getmtime(r"C:\Users\AMREEN\OneDrive - Indian Institute of Technology Jodhpur\Desktop\SEM 4\BCI\commands.txt")
    while True:
        current_modified = os.path.getmtime(r"C:\Users\AMREEN\OneDrive - Indian Institute of Technology Jodhpur\Desktop\SEM 4\BCI\commands.txt")
        if current_modified != last_modified:
            transfer_file()
            last_modified = current_modified
        time.sleep(1)  # Adjust the sleep time as needed

watch_file()