#!/usr/bin/env python3
import rclpy
from classification_msgs.msg import ClassificationResult

class ClassificationPublisher:
    def __init__(self):
        self.node = rclpy.create_node('classification_publisher')
        self.pub = self.node.create_publisher(ClassificationResult, '/drone/classification_topic', 10)

    def publish_classification(self, label, confidence):
        msg = ClassificationResult()
        msg.label = label
        msg.confidence = confidence
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    classification_publisher = ClassificationPublisher()
    rclpy.spin(classification_publisher.node)

if __name__ == '__main__':
    main()
