#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from classification_msgs.msg import ClassificationResult
from plugins.decision_maker import make_decision
import torch
from torchvision import transforms, models

class ClassificationPublisher:
    def __init__(self):
        self.node = rclpy.create_node('classification_publisher')
        self.pub = self.node.create_publisher(ClassificationResult, '/drone/classification_topic', 10)

    def publish_classification(self, label, confidence):
        msg = ClassificationResult()
        msg.label = label
        msg.confidence = confidence
        self.pub.publish(msg)

def perform_classification(image):
    # Image preprocessing
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])

    input_image = transform(image).unsqueeze(0)  # Add batch dimension

    # Load pre-trained ResNet model
    model = models.resnet18(pretrained=True)
    model.eval()

    # Forward pass to obtain predictions
    with torch.no_grad():
        output = model(input_image)

    # Get the class label with the highest probability
    _, predicted_class = torch.max(output, 1)

    # Return label and confidence
    label = str(predicted_class.item())
    confidence = torch.nn.functional.softmax(output, dim=1)[0][predicted_class].item()

    return label, confidence

def callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    label, confidence = perform_classification(cv_image)
    classification_publisher.publish_classification(label, confidence)
    make_decision(label, confidence)

def main(args=None):
    rclpy.init(args=args)
    global classification_publisher
    classification_publisher = ClassificationPublisher()
    node = rclpy.create_node('image_subscriber')
    qos_profile = QoSProfile(depth=10)
    node.create_subscription(Image, '/drone/camera_topic', callback, qos_profile)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
