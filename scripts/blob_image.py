#!/usr/bin/env python
import rospy
import cv2
from cmvision.msg import Blobs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class blob_listener():
    def __init__(self):
        rospy.init_node('blob_listener', anonymous=True)
        self.bridge = CvBridge()
        cv2.namedWindow('Object', 1)
        
        
        rospy.Subscriber('/stereo/left/image_raw', Image, self.camera_callback)
        self.image_pub = rospy.Publisher("image_object_recognition_square",Image, queue_size=5)
        
        self.hasCameraFrame = False

        while not self.hasCameraFrame:
            print 'waiting on camera info.'
            rospy.sleep(0.5)
        
        rospy.Subscriber('blobs', Blobs, self.blob_callback)
        rospy.spin()
    
    def camera_callback(self, image):
        self.hasCameraFrame = True
        self.image_cv = self.bridge.imgmsg_to_cv2(image, 'bgr8')
    
    def blob_callback(self, blobs):
        for blob in blobs.blobs:
                if blob.name == 'Yellow_Cube':
                    print 'Yellow_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
                    self.draw_rect(blob.x, blob.y, (blob.top-blob.bottom), (blob.right-blob.left))
                elif blob.name == 'Green_Cube':
                    print 'Green_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
                    self.draw_rect(blob.x, blob.y, (blob.top-blob.bottom), (blob.right-blob.left))
                elif blob.name == 'Blue_Cube':
                    print 'Blue_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
                    self.draw_rect(blob.x, blob.y, (blob.top-blob.bottom), (blob.right-blob.left))
                elif blob.name == 'Pink_Cube':
                    print 'Pink_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
                    self.draw_rect(blob.x, blob.y, (blob.top-blob.bottom), (blob.right-blob.left))
    
    def draw_rect(self, x, y, h, w):
        cv2.rectangle(self.image_cv, (x, y), (x+w, y+h), (0, 255, 0), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image_cv, "bgr8"))

if __name__ == '__main__':
    blob_listener()