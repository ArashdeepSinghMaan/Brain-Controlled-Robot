# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import numpy as np
# from scipy.signal import welch
# import joblib
# import pylsl

# class EMGCmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('emg_cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.001, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands based on EMG data...')

#         # === Serial Setup ===
#         self.PORT = '/dev/ttyACM0'
#         self.BAUD_RATE = 115200
#         self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

#         self.threshold = 31.0
#         self.value = 0.0
#         self.led_level = 0

#         self.count = 0
#         self.flag3 = True
#         self.current_state = None

#         # === EEG Classifier Setup ===
#         self.fs = 250
#         self.window_size = 750
#         self.channel_indices = [3, 5, 6, 7]  # Use indices directly
#         self.bands = {'mu': (8, 12), 'beta': (13, 30)}

#         # === Load Model and Encoder ===
#         self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
#         self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")

#         # === Connect to EEG LSL Stream ===
#         self.get_logger().info("🔍 Resolving LSL stream...")
#         streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
#         if not streams:
#             self.get_logger().error("❌ No EEG stream found.")
#             self.inlet = None
#         else:
#             self.inlet = pylsl.StreamInlet(streams[0])
#             self.get_logger().info("✅ Connected to EEG stream.")

#     # === This method makes EEG-based prediction ===
#     def turn_classifier(self):
#         if not self.inlet:
#             return -1  # stream not connected

#         eeg_buffer = []
#         self.get_logger().info(f"📡 Collecting {self.window_size} samples...")
#         while len(eeg_buffer) < self.window_size:
#             sample, timestamp = self.inlet.pull_sample()
#             if sample:
#                 eeg_buffer.append(sample)

#         eeg_window = np.array(eeg_buffer)  # shape: (750, 8)
#         band_feats = []

#         for ch_idx in self.channel_indices:
#             signal = eeg_window[:, ch_idx]
#             f, psd = welch(signal, fs=self.fs, nperseg=128)
#             for band_range in self.bands.values():
#                 band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
#                 band_feats.append(band_power)

#         X_input = np.array(band_feats).reshape(1, -1)
#         pred = self.clf.predict(X_input)[0]
#         self.get_logger().info(f"🧠 BCI Prediction: {self.le.inverse_transform([pred])[0]} (class={pred})")
#         return pred  # 0 = right, 1 = left

#     def publish_turn(self):
#         flag = self.turn_classifier()
#         if flag == 0:
#             self.publish_right()
#         elif flag == 1:
#             self.publish_left()

#     def publish_right(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = -0.2
#         self.publisher_.publish(msg)

#     def publish_left(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.2
#         self.publisher_.publish(msg)

#     def publish_straight(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

#     def publish_brakes(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

#     def publish_velocity(self):
#         # === Read EMG Serial Data ===
#         line = self.ser.readline().decode('utf-8').strip()
#         if line and ',' in line:
#             parts = line.split(',')
#             if len(parts) == 4:
#                 try:
#                     _, _, envelope, led_level = map(float, parts)
#                     self.value = envelope
#                     self.led_level = led_level
#                 except ValueError:
#                     self.get_logger().warn('Received invalid data format')
#                     return

#         # === EMG Logic ===
#         if self.led_level > 5:
#             self.publish_brakes()
#             self.flag3 = True
#             self.current_state = 'brake'
#         else:
#             if self.flag3:
#                 if self.count % 2 == 0:
#                     self.current_state = 'straight'
#                 else:
#                     self.current_state = 'turn'
#                 self.count += 1
#                 self.flag3 = False

#             if self.current_state == 'straight':
#                 self.publish_straight()
#             elif self.current_state == 'turn':
#                 self.publish_turn()

#     def destroy_node(self):
#         super().destroy_node()
#         self.ser.close()

# def main(args=None):
#     rclpy.init(args=args)
#     node = EMGCmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




'''test2'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import numpy as np
from scipy.signal import welch
import joblib
import pylsl
from collections import deque

class EMGCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('emg_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.001, self.publish_velocity)
        self.get_logger().info('Publishing velocity commands based on EMG data...')

        # === Serial Setup ===
        self.PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 115200
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

        self.threshold = 31.0
        self.value = 0.0
        self.led_level = 0

        self.count = 0
        self.flag3 = True
        self.current_state = None

        # === EEG Classifier Setup ===
        self.fs = 250
        self.window_size = 750
        self.channel_indices = [3, 5, 6, 7]
        self.bands = {'mu': (8, 12), 'beta': (13, 30)}

        # === Persistent EEG Buffer ===
        self.eeg_buffer = deque(maxlen=self.window_size)

        # === Load Model and Encoder ===
        self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
        self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")

        # === Connect to EEG LSL Stream ===
        self.get_logger().info("🔍 Resolving LSL stream...")
        streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
        if not streams:
            self.get_logger().error("❌ No EEG stream found.")
            self.inlet = None
        else:
            self.inlet = pylsl.StreamInlet(streams[0])
            self.get_logger().info("✅ Connected to EEG stream.")

    def turn_classifier(self):
        if not self.inlet:
            return -1  # No stream connected

        # Collect new EEG sample without resetting buffer
        sample, timestamp = self.inlet.pull_sample()
        if sample:
            self.eeg_buffer.append(sample)

        # Ensure enough data is available
        if len(self.eeg_buffer) < self.window_size:
            self.get_logger().warn(f"⚠️ Not enough EEG samples yet ({len(self.eeg_buffer)}/{self.window_size})")
            return -1

        eeg_window = np.array(self.eeg_buffer)
        band_feats = []

        for ch_idx in self.channel_indices:
            signal = eeg_window[:, ch_idx]
            f, psd = welch(signal, fs=self.fs, nperseg=128)
            for band_range in self.bands.values():
                band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
                band_feats.append(band_power)

        X_input = np.array(band_feats).reshape(1, -1)
        pred = self.clf.predict(X_input)[0]
        self.get_logger().info(f"🧠 BCI Prediction: {self.le.inverse_transform([pred])[0]} (class={pred})")
        return pred

    def publish_turn(self):
        flag = self.turn_classifier()
        if flag == 0:
            self.publish_right()
        elif flag == 1:
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
        # === Read EMG Serial Data ===
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

        # === Clenching Detection ===
        if self.led_level > 5:
            self.get_logger().info("🛑 Clenching detected - Braking!")
            self.publish_brakes()
            self.flag3 = True  # Reset state for next detection
            self.current_state = 'brake'
        else:
            if self.flag3:  # Execute only on new state
                self.count += 1
                self.current_state = 'turn' if self.count % 2 else 'straight'
                self.flag3 = False  # Prevent re-triggering until next clench

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




''''test3'''
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import numpy as np
# from scipy.signal import welch
# import joblib
# import pylsl
# from collections import deque

# class EMGCmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('emg_cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.001, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands based on EMG data...')

#         # === Serial Setup ===
#         self.PORT = '/dev/ttyACM0'
#         self.BAUD_RATE = 115200
#         self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

#         self.threshold = 31.0
#         self.value = 0.0
#         self.led_level = 0

#         self.count = 0
#         self.flag3 = True
#         self.current_state = None

#         # === EEG Classifier Setup ===
#         self.fs = 250
#         self.window_size = 750
#         self.channel_indices = [3, 5, 6, 7]
#         self.bands = {'mu': (8, 12), 'beta': (13, 30)}
#         self.eeg_buffer = deque(maxlen=self.window_size)

#         # === Load Model and Encoder ===
#         self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
#         self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")
#         self.get_logger().info(f"🧠 Label mapping: {list(self.le.classes_)}")

#         # === Connect to EEG LSL Stream ===
#         self.get_logger().info("🔍 Resolving LSL stream...")
#         streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
#         if not streams:
#             self.get_logger().error("❌ No EEG stream found.")
#             self.inlet = None
#         else:
#             self.inlet = pylsl.StreamInlet(streams[0])
#             self.get_logger().info("✅ Connected to EEG stream.")

#     def turn_classifier(self):
#         if not self.inlet:
#             return None

#         sample, timestamp = self.inlet.pull_sample()
#         if sample:
#             self.eeg_buffer.append(sample)

#         if len(self.eeg_buffer) < self.window_size:
#             self.get_logger().warn(f"⚠️ Not enough EEG samples yet ({len(self.eeg_buffer)}/{self.window_size})")
#             return None

#         eeg_window = np.array(self.eeg_buffer)
#         band_feats = []

#         for ch_idx in self.channel_indices:
#             signal = eeg_window[:, ch_idx]
#             f, psd = welch(signal, fs=self.fs, nperseg=128)
#             for band_range in self.bands.values():
#                 band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
#                 band_feats.append(band_power)

#         X_input = np.array(band_feats).reshape(1, -1)
#         pred = self.clf.predict(X_input)[0]
#         pred_label = self.le.inverse_transform([pred])[0]
#         self.get_logger().info(f"🧠 BCI Prediction: {pred_label} (class={pred})")
#         return pred_label

#     def publish_turn(self):
#         label = self.turn_classifier()
#         if label == "right":
#             self.publish_right()
#         elif label == "left":
#             self.publish_left()

#     def publish_right(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = -0.2
#         self.publisher_.publish(msg)
#         self.get_logger().info("↪️ Publishing: RIGHT TURN")

#     def publish_left(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.2
#         self.publisher_.publish(msg)
#         self.get_logger().info("↩️ Publishing: LEFT TURN")

#     def publish_straight(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info("⬆️ Publishing: STRAIGHT")

#     def publish_brakes(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info("🛑 Publishing: BRAKE")

#     def publish_velocity(self):
#         # === Read EMG Serial Data ===
#         line = self.ser.readline().decode('utf-8').strip()
#         if line and ',' in line:
#             parts = line.split(',')
#             if len(parts) == 4:
#                 try:
#                     _, _, envelope, led_level = map(float, parts)
#                     self.value = envelope
#                     self.led_level = led_level
#                 except ValueError:
#                     self.get_logger().warn('Received invalid data format')
#                     return

#         # === Clenching Detection ===
#         if self.led_level > 5:
#             self.publish_brakes()
#             self.flag3 = True
#             self.current_state = 'brake'
#         else:
#             if self.flag3:
#                 self.count += 1
#                 self.current_state = 'turn' if self.count % 2 else 'straight'
#                 self.flag3 = False

#             if self.current_state == 'straight':
#                 self.publish_straight()
#             elif self.current_state == 'turn':
#                 self.publish_turn()

#     def destroy_node(self):
#         super().destroy_node()
#         self.ser.close()

# def main(args=None):
#     rclpy.init(args=args)
#     node = EMGCmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



'''test4'''
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import numpy as np
# from scipy.signal import welch
# import joblib
# import pylsl
# from collections import deque

# class EMGCmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('emg_cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.001, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands based on EMG data...')

#         # === Serial Setup ===
#         self.PORT = '/dev/ttyACM0'
#         self.BAUD_RATE = 115200
#         self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

#         self.threshold = 31.0
#         self.value = 0.0
#         self.led_level = 0

#         self.count = 0
#         self.flag3 = True
#         self.current_state = None

#         # === EEG Classifier Setup ===
#         self.fs = 250
#         self.window_size = 750
#         self.channel_indices = [3, 5, 6, 7]
#         self.bands = {'mu': (8, 12), 'beta': (13, 30)}

#         # === Persistent EEG Buffer ===
#         self.eeg_buffer = deque(maxlen=self.window_size)

#         # === Load Model and Encoder ===
#         self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
#         self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")

#         # === Connect to EEG LSL Stream ===
#         self.get_logger().info("🔍 Resolving LSL stream...")
#         streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
#         if not streams:
#             self.get_logger().error("❌ No EEG stream found.")
#             self.inlet = None
#         else:
#             self.inlet = pylsl.StreamInlet(streams[0])
#             self.get_logger().info("✅ Connected to EEG stream.")

#     def turn_classifier(self):
#         if not self.inlet:
#             return None

#         sample, timestamp = self.inlet.pull_sample()
#         if sample:
#             self.eeg_buffer.append(sample)

#         if len(self.eeg_buffer) < self.window_size:
#             self.get_logger().warn(f"⚠️ Not enough EEG samples yet ({len(self.eeg_buffer)}/{self.window_size})")
#             return None

#         eeg_window = np.array(self.eeg_buffer)
#         band_feats = []

#         for ch_idx in self.channel_indices:
#             signal = eeg_window[:, ch_idx]
#             f, psd = welch(signal, fs=self.fs, nperseg=128)
#             for band_range in self.bands.values():
#                 band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
#                 band_feats.append(band_power)

#         X_input = np.array(band_feats).reshape(1, -1)
#         pred = self.clf.predict(X_input)[0]
#         pred_label = self.le.inverse_transform([pred])[0]
#         self.get_logger().info(f"🧠 EEG Prediction: {pred_label.upper()} (class={pred})")
#         return pred_label

#     def publish_turn(self):
#         flag = self.turn_classifier()
#         if flag == 0:
#             self.publish_right()
#         elif flag == 1:
#             self.publish_left()

#     def publish_right(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = -0.2
#         self.publisher_.publish(msg)

#     def publish_left(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.2
#         self.publisher_.publish(msg)

#     def publish_straight(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

#     def publish_brakes(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

#     def publish_velocity(self):
#     # === Read EMG Serial Data ===
#         line = self.ser.readline().decode('utf-8').strip()
#         if line and ',' in line:
#             parts = line.split(',')
#             if len(parts) == 4:
#                 try:
#                     _, _, envelope, led_level = map(float, parts)
#                     self.value = envelope
#                     self.led_level = led_level
#                 except ValueError:
#                     self.get_logger().warn('Received invalid data format')
#                     return

#         # === Clenching triggers turn detection ===
#         if self.led_level > 3:
#             self.get_logger().info("📥 Clenching detected — Gathering EEG data for prediction...")
#             label = self.turn_classifier()

#             if label == "right":
#                 self.publish_right()
#             elif label == "left":
#                 self.publish_left()
#             else:
#                 self.get_logger().info("❓ Not enough data — no prediction.")
#         else:
#             # No clench: go straight
#             self.publish_straight()

#     def destroy_node(self):
#         super().destroy_node()
#         self.ser.close()

# def main(args=None):
#     rclpy.init(args=args)
#     node = EMGCmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

'''test5'''

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import serial
# import numpy as np
# from scipy.signal import welch
# import joblib
# import pylsl
# from collections import deque

# class EMGCmdVelPublisher(Node):
#     def __init__(self):
#         super().__init__('emg_cmd_vel_publisher')
#         self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.timer = self.create_timer(0.001, self.publish_velocity)
#         self.get_logger().info('Publishing velocity commands based on EMG data...')

#         self.PORT = '/dev/ttyACM0'
#         self.BAUD_RATE = 115200
#         self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

#         self.fs = 250
#         self.window_size = 750
#         self.channel_indices = [3, 5, 6, 7]
#         self.bands = {'mu': (8, 12), 'beta': (13, 30)}
#         self.eeg_buffer = deque(maxlen=self.window_size)

#         self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
#         self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")

#         self.get_logger().info("🔍 Resolving LSL stream...")
#         streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
#         if not streams:
#             self.get_logger().error("❌ No EEG stream found.")
#             self.inlet = None
#         else:
#             self.inlet = pylsl.StreamInlet(streams[0])
#             self.get_logger().info("✅ Connected to EEG stream.")

#         # === State Control ===
#         self.clench_ready = True  # allow clench detection
#         self.current_mode = "straight"  # can be 'straight', 'turn_left', 'turn_right'
#         self.led_level = 0.0

#     def turn_classifier(self):
#         if not self.inlet:
#             return None

#         self.get_logger().info("⏳ Collecting EEG samples...")
#         self.eeg_buffer.clear()
#         while len(self.eeg_buffer) < self.window_size:
#             sample, timestamp = self.inlet.pull_sample()
#             if sample:
#                 self.eeg_buffer.append(sample)

#         eeg_window = np.array(self.eeg_buffer)
#         band_feats = []

#         for ch_idx in self.channel_indices:
#             signal = eeg_window[:, ch_idx]
#             f, psd = welch(signal, fs=self.fs, nperseg=128)
#             for band_range in self.bands.values():
#                 band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
#                 band_feats.append(band_power)

#         # === Bias Correction ===
#         correction = 0.0432 / 2
#         for i, ch_idx in enumerate(self.channel_indices):
#             for b in range(len(self.bands)):
#                 idx = i * len(self.bands) + b
#                 if ch_idx in [3, 6]:
#                     band_feats[idx] -= correction
#                 elif ch_idx in [5, 7]:
#                     band_feats[idx] += correction

#         X_input = np.array(band_feats).reshape(1, -1)
#         pred = self.clf.predict(X_input)[0]
#         pred_label = self.le.inverse_transform([pred])[0]
#         self.get_logger().info(f"🧠 EEG Prediction (bias corrected): {pred_label.upper()} (class={pred})")
#         return pred_label

#     def publish_right(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = -0.2
#         self.publisher_.publish(msg)
#         self.get_logger().info("↪️ Turning RIGHT")

#     def publish_left(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.2
#         self.publisher_.publish(msg)
#         self.get_logger().info("↩️ Turning LEFT")

#     def publish_straight(self):
#         msg = Twist()
#         msg.linear.x = 0.2
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info("⬆️ Moving STRAIGHT")

#     def publish_brakes(self):
#         msg = Twist()
#         msg.linear.x = 0.0
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
#         self.get_logger().info("🛑 BRAKES APPLIED")

#     def publish_velocity(self):
#         line = self.ser.readline().decode('utf-8').strip()
#         if line and ',' in line:
#             parts = line.split(',')
#             if len(parts) == 4:
#                 try:
#                     _, _, envelope, led_level = map(float, parts)
#                     self.led_level = led_level
#                 except ValueError:
#                     self.get_logger().warn('❌ Invalid serial data')
#                     return

#         if self.led_level > 1 and self.clench_ready:
#             self.clench_ready = False
#             self.publish_brakes()

#             if self.current_mode == "straight":
#                 self.get_logger().info("🧠 Predicting direction...")
#                 label = self.turn_classifier()
#                 if label == "right":
#                     self.current_mode = "turn_right"
#                     self.publish_right()
#                 elif label == "left":
#                     self.current_mode = "turn_left"
#                     self.publish_left()
#                 else:
#                     self.get_logger().info("❓ EEG data error — staying straight.")
#                     self.current_mode = "straight"
#                     self.publish_straight()
#             else:
#                 self.get_logger().info("🔁 Switching to STRAIGHT")
#                 self.current_mode = "straight"
#                 self.publish_straight()

#         elif self.led_level <= 1:
#             self.clench_ready = True

#         # Maintain current motion if not clenching
#         if self.led_level <= 1:
#             if self.current_mode == "straight":
#                 self.publish_straight()
#             elif self.current_mode == "turn_right":
#                 self.publish_right()
#             elif self.current_mode == "turn_left":
#                 self.publish_left()

#     def destroy_node(self):
#         super().destroy_node()
#         self.ser.close()

# def main(args=None):
#     rclpy.init(args=args)
#     node = EMGCmdVelPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()