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

        self.PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 115200
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=1)

        self.fs = 250
        self.window_size = 750
        self.channel_indices = [3, 5, 6, 7]
        self.bands = {'mu': (8, 12), 'beta': (13, 30)}
        self.eeg_buffer = deque(maxlen=self.window_size)

        self.clf = joblib.load("/home/aditya/Downloads/random_forest_eeg_model.pkl")
        self.le = joblib.load("/home/aditya/Downloads/label_encoder.pkl")

        self.get_logger().info(" Resolving LSL stream...")
        streams = pylsl.resolve_byprop('name', 'BCI_stream2', timeout=5)
        if not streams:
            self.get_logger().error("No EEG stream found.")
            self.inlet = None
        else:
            self.inlet = pylsl.StreamInlet(streams[0])
            self.get_logger().info("Connected to EEG stream.")

        # === State Control ===
        self.clench_ready = True  # allow clench detection
        self.current_mode = "straight"  # can be 'straight', 'turn_left', 'turn_right'
        self.led_level = 0.0

    def turn_classifier(self):
        if not self.inlet:
            return None

        self.get_logger().info("Collecting EEG samples...")
        self.eeg_buffer.clear()
        while len(self.eeg_buffer) < self.window_size:
            sample, timestamp = self.inlet.pull_sample()
            if sample:
                self.eeg_buffer.append(sample)

        eeg_window = np.array(self.eeg_buffer)
        band_feats = []

        for ch_idx in self.channel_indices:
            signal = eeg_window[:, ch_idx]
            f, psd = welch(signal, fs=self.fs, nperseg=128)
            for band_range in self.bands.values():
                band_power = np.mean(psd[(f >= band_range[0]) & (f <= band_range[1])])
                band_feats.append(band_power)

        # === Bias Correction ===
        correction = 0.0432 / 2
        for i, ch_idx in enumerate(self.channel_indices):
            for b in range(len(self.bands)):
                idx = i * len(self.bands) + b
                if ch_idx in [3, 6]:
                    band_feats[idx] -= correction
                elif ch_idx in [5, 7]:
                    band_feats[idx] += correction

        X_input = np.array(band_feats).reshape(1, -1)
        pred = self.clf.predict(X_input)[0]
        pred_label = self.le.inverse_transform([pred])[0]
        self.get_logger().info(f"EEG Prediction (bias corrected): {pred_label.upper()} (class={pred})")
        return pred_label

    def publish_right(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -0.2
        self.publisher_.publish(msg)
        self.get_logger().info(" Turning RIGHT")

    def publish_left(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.2
        self.publisher_.publish(msg)
        self.get_logger().info("Turning LEFT")

    def publish_straight(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(" Moving STRAIGHT")

    def publish_brakes(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(" BRAKES APPLIED")

    def publish_velocity(self):
        line = self.ser.readline().decode('utf-8').strip()
        if line and ',' in line:
            parts = line.split(',')
            if len(parts) == 4:
                try:
                    _, _, envelope, led_level = map(float, parts)
                    self.led_level = led_level
                except ValueError:
                    self.get_logger().warn(' Invalid serial data')
                    return

        if self.led_level > 1 and self.clench_ready:
            self.clench_ready = False
            self.publish_brakes()

            if self.current_mode == "straight":
                self.get_logger().info("Predicting direction...")
                label = self.turn_classifier()
                if label == "right":
                    self.current_mode = "turn_right"
                    self.publish_right()
                elif label == "left":
                    self.current_mode = "turn_left"
                    self.publish_left()
                else:
                    self.get_logger().info(" EEG data error — staying straight.")
                    self.current_mode = "straight"
                    self.publish_straight()
            else:
                self.get_logger().info("Switching to STRAIGHT")
                self.current_mode = "straight"
                self.publish_straight()

        elif self.led_level <= 1:
            self.clench_ready = True

        # Maintain current motion if not clenching
        if self.led_level <= 1:
            if self.current_mode == "straight":
                self.publish_straight()
            elif self.current_mode == "turn_right":
                self.publish_right()
            elif self.current_mode == "turn_left":
                self.publish_left()

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