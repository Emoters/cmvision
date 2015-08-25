#!/usr/bin/env python
import rospy
from cmvision.msg import Blobs

def blob_callback(blobs):
    for blob in blobs.blobs:
            if blob.name == 'Pink':
                print 'Pink: x = ' + str(blob.x)

# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
def listener():
    rospy.init_node('blob_listener', anonymous=True)
    rospy.Subscriber('blobs', Blobs, blob_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()