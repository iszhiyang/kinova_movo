import sys
rospth1='/opt/ros/kinetic/lib/python2.7/dist-packages'
# rospth2='/home/radoe-1/movo_ws/devel/lib/python2.7/dist-packages'
# # rospth='/opt/ros/kinetic/lib/python2.7/dist-packages'
# if rospth in sys.path:
sys.path.remove(rospth1)
# sys.path.remove(rospth2)
import threading
import cv2
import time
sys.path.append(rospth1)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class eyeinhand_camera_server(object):
    def __init__(self):
        rospy.init_node("eyeinhand_camera_publisher")
        self.bridge = CvBridge()

        series_ids=[0,1] # right hand

        cams=[]
        for id in series_ids:
            cam=cv2.VideoCapture(id)
            cam.set(4, 320)
            cam.set(3, 240)
            cams.append(cam)
        self.imgs=[None for i in cams]
        self.img_pubs=[]
        for i in range(len(self.imgs)):
            self.img_pubs.append(rospy.Publisher("bri_img"+str(i), Image, queue_size=10))
        # self.image_pub =


        for id,cam in enumerate(cams):
            get_thr=threading.Thread(target=self._get,args=(cam,id,))
            get_thr.start()

        pub_thr=threading.Thread(target=self._pub,args=())
        pub_thr.start()


    def _pub(self):

        print("=> publisher running")
        while True:
            for i,img in enumerate(self.imgs):
                if img is None:
                    continue
                # self.img_pubs[i].publish(img)#self.bridge.cv2_to_imgmsg(img, "bgr8"))
                self.img_pubs[i].publish(self.bridge.cv2_to_imgmsg(img))
                # time.sleep(1)
                rospy.Rate(10)
    def _get(self,cam,camid):
        print("=> getting img ",camid," running")
        while True:
            try:
                ret, frame = cam.read()
                # print(frame.shape,camid)
                self.imgs[camid]=frame
                if not ret:
                    print("failed to grab frame")
                    # break

                rospy.Rate(10)
            except Exception as err:
                print (err)
                pass

if __name__=="__main__":
    camera_server=eyeinhand_camera_server()