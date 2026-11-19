import math
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class HandTrackerNode(Node):
    """ROS 2 node for real-time hand tracking and 6-DOF servo angle publishing."""

    # Calibrated 2D landmark distance ranges (min_closed, max_open)
    LIMITS = {
        'thumb_flex': (0.22, 0.35),
        'index':      (0.35, 0.45),
        'middle':     (0.35, 0.45),
        'ring':       (0.25, 0.40),
        'pinky':      (0.20, 0.35),
        'thumb_base': (0.15, 0.25),
    }

    def __init__(self, show_preview: bool = True):
        super().__init__('hand_tracker_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_array', 10)
        self.show_preview = show_preview

        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    @staticmethod
    def map_range(x: float, in_min: float, in_max: float, out_min: int, out_max: int) -> int:
        """Map a distance value to a clamped servo angle (0-180 degrees)."""
        mapped = int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
        return max(min(mapped, 180), 0)

    @staticmethod
    def calculate_distance(p1, p2) -> float:
        """Compute Euclidean 2D distance between two MediaPipe landmarks."""
        return math.hypot(p2.x - p1.x, p2.y - p1.y)

    def run(self):
        cap = cv2.VideoCapture(0)
        self.get_logger().info('Hand Tracker Node started. Publishing to /servo_array')

        while cap.isOpened() and rclpy.ok():
            success, img = cap.read()
            if not success:
                break

            if self.show_preview:
                img = cv2.flip(img, 1)

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = self.hands.process(img_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    if self.show_preview:
                        self.mp_draw.draw_landmarks(
                            img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                        )

                    lm = hand_landmarks.landmark

                    # Compute wrist-to-finger landmark distances
                    thumb_tip  = self.calculate_distance(lm[0], lm[4])
                    index_tip  = self.calculate_distance(lm[0], lm[8])
                    middle_tip = self.calculate_distance(lm[0], lm[12])
                    ring_tip   = self.calculate_distance(lm[0], lm[16])
                    pinky_tip  = self.calculate_distance(lm[0], lm[20])
                    thumb_base = self.calculate_distance(lm[0], lm[2])

                    # Map distances to servo angles (180 = closed, 0 = open)
                    angle_thumb_flex = self.map_range(thumb_tip,  *self.LIMITS['thumb_flex'], 180, 0)
                    angle_index      = self.map_range(index_tip,  *self.LIMITS['index'],      180, 0)
                    angle_middle     = self.map_range(middle_tip, *self.LIMITS['middle'],     180, 0)
                    angle_ring       = self.map_range(ring_tip,   *self.LIMITS['ring'],       180, 0)
                    angle_pinky      = self.map_range(pinky_tip,  *self.LIMITS['pinky'],      180, 0)
                    angle_thumb_base = self.map_range(thumb_base, *self.LIMITS['thumb_base'], 180, 0)

                    msg = Int32MultiArray()
                    msg.data = [
                        angle_thumb_flex,
                        angle_index,
                        angle_middle,
                        angle_ring,
                        angle_pinky,
                        angle_thumb_base,
                    ]
                    self.publisher_.publish(msg)

            if self.show_preview:
                cv2.imshow('ROS 2 Hand Tracker', img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrackerNode(show_preview=True)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()