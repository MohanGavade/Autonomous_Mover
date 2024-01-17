# ~/catkin_ws/src/object_detection_package/scripts/object_detection_node.py

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node', anonymous=True)

        # Define the custom TensorFlow model architecture
        base_model = tf.keras.applications.efficientnet.EfficientNetB4(include_top=False)
        base_model.trainable = False
        inputs = tf.keras.layers.Input(shape=(224, 224, 3), name="input_layer")
        x = tf.keras.layers.experimental.preprocessing.Rescaling(1./255)(inputs)  # Normalize pixel values
        x = base_model(x, training=False)
        x = tf.keras.layers.GlobalAveragePooling2D(name="global_average_pooling")(x)
        outputs = tf.keras.layers.Dense(2, activation="softmax", name="output_layer")(x)
        self.model = tf.keras.Model(inputs, outputs)

        # Load pre-trained weights for the custom model
        weights_path = 'path/to/your/pretrained_weights.h5'
        self.model.load_weights(weights_path)

        # Create a publisher for object detection results
        self.object_detection_publisher = rospy.Publisher('object_detection_results', String, queue_size=10)

        # Create a subscriber for the camera image
        self.image_subscriber = rospy.Subscriber('camera_image', Image, self.image_callback, queue_size=10)

        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Preprocess the input image for the custom model
        preprocessed_input = preprocess_input(cv_image)

        # Perform inference using the TensorFlow model
        predictions = self.model.predict(np.expand_dims(preprocessed_input, axis=0))

        # Process the predictions as needed
        # For example, you might extract the class labels and confidence scores

        # Publish the results using ROS publisher
        detection_result_msg = String()
        detection_result_msg.data = "Object Detected!"
        self.object_detection_publisher.publish(detection_result_msg)

def preprocess_input(image):
    # Implement any additional preprocessing steps needed for your custom model
    return image

if __name__ == '__main__':
    try:
        ObjectDetectionNode()
        rospy.spin()
    except KeyboardInterrupt:
        pass
