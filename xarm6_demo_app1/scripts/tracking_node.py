# coding: utf-8
import numpy as np
import cv2
import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class testNode():
    def __init__(self):
        self.bridge = CvBridge()
        # Subscriberの作成
        sub_rgb = message_filters.Subscriber('/camera/color/image_raw', Image)
        sub_depth = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 0.5)
        self.mf.registerCallback(self.callback)
        # Publisherの作成
        self.pub_target = rospy.Publisher('/camera/target', PoseStamped, queue_size=5)
        self.pub_dbg_1 = rospy.Publisher('/camera/debug', Image, queue_size=5)
        self.pub_dbg_2 = rospy.Publisher('/camera/debug2', Image, queue_size=5)

    def callback(self, rgb_data, depth_data):
        #print('rgb   width = {}, height = {}'.format(rgb_data.width, rgb_data.height))
        #print('depth width = {}, height = {}'.format(depth_data.width, depth_data.height))


        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, rgb_data.encoding)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, depth_data.encoding)
        except CvBridgeError, e:
            rospy.logerr(e)

        gray_image = cv2.cvtColor(color_image , cv2.COLOR_BGR2GRAY)     # グレースケール化
        ret, img_binary = cv2.threshold(gray_image,                     # 二値化
                40, 255,                                                # 二値化のための閾値60(調整要)
                cv2.THRESH_BINARY)

        img_binary_inv = cv2.bitwise_not(img_binary)
        dummy_image, contours, hierarchy = cv2.findContours(img_binary_inv, # 輪郭検出
                cv2.RETR_LIST,
                cv2.CHAIN_APPROX_SIMPLE)
        img_con = img_binary_inv
        num_cont = list()
        for i in range(len(contours)):
            num_cont.append(len(np.array(contours[i])))
            #print('i={}, len={}, num_const={}'.format(i, len(np.array(contours[i])), num_cont[i]))
        i_max = num_cont.index(max(num_cont))  # ノイズ除去のため、輪郭検出点数の多いcontoursを採用する
        cv2.drawContours(img_con, contours[i_max], -1, 100, 2)

        # 輪郭検出から物体中心を算出
        x = np.mean(contours[i_max].T[0, 0])                                # 輪郭のx方向平均値を算出
        y = np.mean(contours[i_max].T[1, 0])                                # 輪郭のy方向平均値を算出

        img_circle = cv2.circle(color_image, (int(x), int(y)), 30, 155, 3)  # 検出した位置にサークル描画

        send_img_circle = self.bridge.cv2_to_imgmsg(img_circle, rgb_data.encoding)  # ROS msgに変換
        ofs = 320
        x = x * float(depth_data.width - ofs) / float(rgb_data.width) + ofs/2
        y = y * float(depth_data.height) / float(rgb_data.height)

        depth_image_4 = depth_image * 4
        d_img_circle = cv2.circle(depth_image_4, (int(x), int(y)), 30, 255*256, 3)  # 検出した位置にサークル描画
        send_d_img = self.bridge.cv2_to_imgmsg(d_img_circle, depth_data.encoding)   # ROS msgに変換
        d_img_array = np.array(depth_image)
        z_int16 = d_img_array[int(y),int(x)]
        ps = PoseStamped()
        ps.header = depth_data.header
        ps.pose.position = Point(y/1000,x/1000,float(z_int16)/1000)
        ps.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        self.pub_target.publish(ps)
        self.pub_dbg_1.publish(send_img_circle)
        self.pub_dbg_2.publish(send_d_img)


if __name__ == '__main__':
    rospy.init_node('target_notifier')

    node = testNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

