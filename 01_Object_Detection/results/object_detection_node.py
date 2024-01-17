# ~/ros2_ws/src/object_detection_package/scripts/object_detection_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Load the TensorFlow model
        self.model = tf.keras.models.load_model('path')

        # Create a publisher for object detection results
        self.object_detection_publisher = self.create_publisher(String, 'object_detection_results', 10)

        # Create a subscriber for the camera image
        self.image_subscriber = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        self.image_subscriber  # prevent unused variable warning

        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess the input image for EfficientNet
        preprocessed_input = preprocess_input(cv_image)

        # Perform inference using the TensorFlow model
        predictions = self.model.predict(np.expand_dims(preprocessed_input, axis=0))

        # Process the predictions as needed
        # For example, you might extract the class labels and confidence scores

        # Publish the results using ROS 2 publisher
        detection_result_msg = String()
        detection_result_msg.data = "Object Detected!"
        self.object_detection_publisher.publish(detection_result_msg)

def preprocess_input(image):
    # Implement any preprocessing steps needed for EfficientNet
    # Resize the image to the required input size
    input_size = (224, 224)  # Adjust based on your model's input size
    image = tf.image.resize(image, input_size)
    # Normalize pixel values to the range [0, 1]
    # image /= 255.0
    # Expand the dimensions to match the model input shape
    image = tf.expand_dims(image, axis=0)
    return image.numpy()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
