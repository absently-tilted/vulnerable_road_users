# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
import rclpy.logging
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV li        
import time
# import lgpio
import time
import pdb
import lgpio
# import pygame
from playsound import playsound

 
class AlertLight(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('alert_light')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      String, 
      'alerts', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.h = lgpio.gpiochip_open(0)

    self.LED = 17
    lgpio.gpio_claim_output(self.h, self.LED)



   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # pygame.mixer.init()


    lgpio.gpio_write(self.h, self.LED, 1)
    time.sleep(0.1)
            
    lgpio.gpio_write(self.h, self.LED, 0)
    time.sleep(0.1)
    # print("LED flashed")

    #   # Load the WAV file into pygame
    # pygame.mixer.music.load(file)

    #   # Play the audio
    # pygame.mixer.music.play()

    #   # Wait until the audio finishes playing
    # while pygame.mixer.music.get_busy():
    #       pygame.time.Clock().tick(10)

    rclpy.logging("Alert, light flashing")
    # rclpy.sleep(5)

    

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  alert = AlertSub()  
  # Spin the node so the callback function is called.
  rclpy.spin(alert)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  alert.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
