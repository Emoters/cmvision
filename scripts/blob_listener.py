#!/usr/bin/env python
import rospy
from cmvision.msg import Blobs

def blob_callback(blobs):
    for blob in blobs.blobs:
            if blob.name == 'Yellow_Cube':
                print 'Yellow_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
            elif blob.name == 'Green_Cube':
                print 'Green_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
            elif blob.name == 'Blue_Cube':
                print 'Blue_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)
            elif blob.name == 'Pink_Cube':
                print 'Pink_Cube: x = ' + str(blob.x) + ' y = ' + str(blob.y)

# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
def listener():
    rospy.init_node('blob_listener', anonymous=True)
    rospy.Subscriber('blobs', Blobs, blob_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()