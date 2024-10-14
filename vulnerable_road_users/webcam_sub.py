# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV li        import time
import time
from ultralytics import YOLO
import pdb

 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      1)
    
    self.publisher2 = self.create_publisher(Image, 'yolo_output', 10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.model = YOLO("yolov10n.pt")
    self.model.to('cuda')
    self.publisher_ = self.create_publisher(String, 'alerts', 1)


   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    

    # Load a pretrained YOLOv8n model

    # Run inference on the source
    results = self.model(current_frame, stream=True, classes=[0, 1])  # list of Results objects
    print(results)
    for result in results:
      boxes = result.boxes  # Boxes object for bounding box outputs
      # result.save()
      plot = result.plot()
      msg_plot = self.br.cv2_to_imgmsg(plot, encoding='bgr8')
      try:
        if len(boxes.conf) > 0:
          if (boxes.conf[0] > 0.65) and (0 in boxes.cls or 1 in boxes.cls):
            msg = String()
            msg.data = "Person detected"
            self.publisher_.publish(msg)
            self.publisher2.publish(msg_plot)
      except RuntimeError:
        pdb.set_trace()
        
      
      #result.show()   display to screen
      # result.save(filename="result.jpg")  # save to disk


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
