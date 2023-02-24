#!/usr/bin/env python
#from auvsi2019.msg import ImageMsg
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool



class ImgProc:
    def __init__(self):
        print "masuk init"
        self.state = False
        self.isSending = False
        self.isCapturing = False
        self.camSub = rospy.Subscriber('statecam', Bool, self.callbackState)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_ori", Image, queue_size=1)
        self.imagebin_pub = rospy.Publisher("image_bin", Image, queue_size=1)
        while not rospy.is_shutdown():
            if self.state==True:
                self.sendImage()
            elif self.isCapturing and not self.isSending:
                self.stopCam()
            rospy.sleep(0.01)

    def startCam(self):
        self.cap = cv2.VideoCapture(0)
        self.isCapturing = True

    def sendImage(self):
        self.isSending = True
        print self.state;
        ret, frame = self.cap.read()
        frame = cv2.resize(frame, (480,360))
        bin_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img)
            print("image ori published")
        except CvBridgeError as err:
            print(err)
#            try:
#                img = bridge.cv2_to_imgmsg(bin_frame, "mono8")
#                self.imagebin_pub.publish(img)
#                print("image bin published")
#            except CvBridgeError as err:
#                print(err)
        self.isSending = False

    def stopCam(self):
        self.isCapturing = False
        self.cap.release()

    def callbackState(self, data):
        if (data.data is True):
            self.startCam();
        print "camera", data.data
        self.state = data.data


if __name__ == '__main__':
    rospy.init_node('ImgProc', anonymous=False)
    imgpro = ImgProc()
