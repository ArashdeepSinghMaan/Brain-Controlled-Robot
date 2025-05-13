# import rclpy
# import pandas as pd
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class CmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.01, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands...')
  
#         self.data = pd.read_csv('/home/aditya/Downloads/NIPRA.csv') 
#         self.index = 0
#         self.threshold = 31.0
#         self.value = 0.0
    
#     def publish_velocity(self):
#         if self.index < len(self.data):
#             self.value = self.data.iloc[self.index]['Envelope']
#             self.index += 1
#         else:
#             self.value = 0.0

#         msg = Twist()
#         if self.value < self.threshold:
#             msg.linear.x = 0.2  
#             msg.angular.z = 0.0 
#         else:
#             msg.linear.x = 0.0 
#             msg.angular.z = 0.0 
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}, envelope={self.value}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = CmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import time

# class EMGCmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('emg_cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.001, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands based on EMG data...')

#         # Serial setup
#         self.PORT = '/dev/ttyACM0'  # Update with actual port
#         self.BAUD_RATE = 115200
#         self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

#         self.threshold = 31.0
#         self.value = 0.0
#         self.led_level = 0

#     def publish_velocity(self):
#         line = self.ser.readline().decode('utf-8').strip()
#         if line and ',' in line:
#             parts = line.split(',')
#             if len(parts) == 4:
#                 try:
#                     _, _, envelope, led_level = map(float, parts)  # Extract values
#                     self.value = envelope
#                     self.led_level = led_level
#                 except ValueError:
#                     self.get_logger().warn('Received invalid data format')
#                     return
        
#         msg = Twist()
#         if self.led_level > 5:  # Clenching detected
#             msg.linear.x = 0.0  # Stop the robot
#             msg.angular.z = 0.0 
#         else:
#             msg.linear.x = 0.2  # Move forward
#             msg.angular.z = 0.0
        
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}, envelope={self.value}, LED_Level={self.led_level}')

#     def destroy_node(self):
#         super().destroy_node()
#         self.ser.close()  # Ensure serial port is closed


# def main(args=None):
#     rclpy.init(args=args)
#     node = EMGCmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




















import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class EMGCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('emg_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.001, self.publish_velocity)
        self.get_logger().info('Publishing velocity commands based on EMG data...')

        self.PORT = '/dev/ttyACM0' 
        self.BAUD_RATE = 115200
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

        self.threshold = 31.0
        self.value = 0.0
        self.led_level = 0

        self.count = 0
        self.flag3 = True
        self.current_state = None 

    def turn_classifier(self):
        # placeholder
        # it returns 0 for right and 1 for left
        return 0
    
    def publish_turn(self):
        flag = self.turn_classifier()
        if flag == 0:
            self.publish_right()
        if flag == 1:
            self.publish_left()
    
    def publish_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.2
        self.publisher_.publish(msg)

    def publish_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.2
        self.publisher_.publish(msg)  

    def publish_straight(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def publish_brakes(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)  
    
    def publish_velocity(self):
        line = self.ser.readline().decode('utf-8').strip()
        if line and ',' in line:
            parts = line.split(',')
            if len(parts) == 4:
                try:
                    _, _, envelope, led_level = map(float, parts) 
                    self.value = envelope
                    self.led_level = led_level
                except ValueError:
                    self.get_logger().warn('Received invalid data format')
                    return

        if self.led_level > 5:
            self.publish_brakes()
            self.flag3 = True 
            self.current_state = 'brake'
        else:
            if self.flag3:
                if self.count % 2 == 0:
                    self.current_state = 'straight'
                else:
                    self.current_state = 'turn'
                self.count += 1
                self.flag3 = False
            
            if self.current_state == 'straight':
                self.publish_straight()
            elif self.current_state == 'turn':
                self.publish_turn()
    
    def destroy_node(self):
        super().destroy_node()
        self.ser.close()  

def main(args=None):
    rclpy.init(args=args)
    node = EMGCmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
