""" 
vision node listens for /image topic messages and 
runs the data through cv2's hog model for human detection
overlaying a bounding box on the /image and publishing 
this bounded box image onto the /image_classified topic. 
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_system_default
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class VisionNode(Node):
    """
    Vision node produces bounding box for humans in the world
    and outputs an image overlay
    """
    def __init__(self):
        super().__init__('vision_node')
        
        # Initialize CV Bridge for converting ROS Images to OpenCV format
        self.bridge = CvBridge()
        
        # Load pre-trained HOG + SVM human detector from OpenCV
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # Create subscription
        self.image_sub = self.create_subscription(
            Image,
            'image',
            self.image_rcv_cb,
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        # Create publisher for classified image
        self.image_classified_publisher = self.create_publisher(
            Image,
            'image_classified',
            qos_profile=qos_profile_system_default,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
        self.get_logger().info('Vision Node initialized. Listening for images...')
    
    def image_rcv_cb(self, msg: Image):
        """
        Callback function that receives images, detects humans, 
        draws bounding boxes, and publishes the result
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            bounding_boxes, weights = self.hog.detectMultiScale(
                cv_image, 
                winStride=(8, 8),
                padding=(4, 4),
                scale=1.05
            )
            
            # Draw bounding boxes on detected humans
            for i, (x, y, w, h) in enumerate(bounding_boxes):
                # Draw rectangle around detected human
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                confidence = float(weights[i])
                label = f'Human: {confidence:.2f}'
                cv2.putText(
                    cv_image, 
                    label, 
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
            
            detection_text = f'Humans detected: {len(bounding_boxes)}'
            cv2.putText(
                cv_image,
                detection_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
            
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            output_msg.header = msg.header  # Preserve original timestamp and frame
            self.image_classified_publisher.publish(output_msg)
            
            if len(bounding_boxes) > 0:
                self.get_logger().info(f'Detected {len(bounding_boxes)} human(s)')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main():
    rclpy.init()
    node = VisionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    
    try:
        spin_thread.start()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()