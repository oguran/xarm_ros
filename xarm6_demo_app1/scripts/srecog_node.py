# coding: utf-8
import numpy as np
import cv2
import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from srecog_msgs.msg import ObjPose
from srecog_msgs.msg import ObjPoseList

class testNode():
    def __init__(self):
        self.bridge = CvBridge()
        # Subscriberの作成
        sub_rgb = message_filters.Subscriber('/camera/color/image_raw', Image)
        sub_depth = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        sub_rgb_cinfo = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
        sub_rgb_dinfo = message_filters.Subscriber('/camera/depth/camera_info', CameraInfo)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth, sub_rgb_cinfo, sub_rgb_dinfo], 100, 0.5)
        self.mf.registerCallback(self.callback)
        # Publisherの作成
        #self.pub_target = rospy.Publisher('/xarm/camera/target', PoseStamped, queue_size=5)
        self.pub_target = rospy.Publisher('/srecog/obj_pose_list', ObjPoseList, queue_size=5)
        self.pub_d_image = rospy.Publisher('/xarm/camera/depth/image', Image, queue_size=5)
        self.pub_d_cinfo = rospy.Publisher('/xarm/camera/depth/camera_info', CameraInfo, queue_size=5)
        self.pub_dbg_1 = rospy.Publisher('/camera/debug', Image, queue_size=5)
        self.pub_dbg_2 = rospy.Publisher('/camera/debug2', Image, queue_size=5)

    def callback(self, rgb_data, depth_data, c_camera_info, d_camera_info):
        #print('rgb   width = {}, height = {}'.format(rgb_data.width, rgb_data.height))
        #print('depth width = {}, height = {}'.format(depth_data.width, depth_data.height))
        #print('c_info width = {}, height = {}'.format(c_camera_info.width, c_camera_info.height))
        #print('d_info width = {}, height = {}'.format(d_camera_info.width, d_camera_info.height))


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

        if num_cont:    # リストが空ではない
            i_max = num_cont.index(max(num_cont))  # 輪郭検出点数の一番多いcontoursを採用する
            cv2.drawContours(img_con, contours[i_max], -1, 100, 2)

            # 輪郭検出から物体中心を算出
            x = np.mean(contours[i_max].T[0, 0])   # 輪郭のx方向平均値を算出
            y = np.mean(contours[i_max].T[1, 0])   # 輪郭のy方向平均値を算出
        else:
            x = 0
            y = 0

        img_circle = cv2.circle(color_image, (int(x), int(y)), 20, 155, 3)  # 検出した位置にサークル描画

        send_img_circle = self.bridge.cv2_to_imgmsg(img_circle, rgb_data.encoding)  # ROS msgに変換
        # coloer/image_raw -> /depth/image_raw のx,y座標変換。
        if depth_data.width > depth_data.height:    # アスペクト比が横が長い場合
            # rgb image とdepteh imagaの縦の比率分引き延ばす（圧縮する）.
            # その後、横を上記比率分引き伸ばし（圧縮し）、足りない分/2 をオフセットとする
            gain = float(depth_data.height) / float(rgb_data.height)
            ofs_x = (float(rgb_data.width) - float(depth_data.width) / gain) / 2
            ofs_y = 0
        else:
            gain = float(depth_data.width) / float(rgb_data.width)
            ofs_x = 0
            ofs_y = (float(rgb_data.height) - float(depth_data.height) / gain) / 2
        x = (x - ofs_x) * gain
        y = (y - ofs_y) * gain

        d_img_array = np.array(depth_image)
        if np.amax(d_img_array) > 255*256/4:
            depth_image_mod = depth_image
        else:
            depth_image_mod = depth_image * 4
        d_img_circle = cv2.circle(depth_image_mod, (int(x), int(y)), 10, 255*256, 3)    # 検出した位置にサークル描画
        send_d_img = self.bridge.cv2_to_imgmsg(d_img_circle, depth_data.encoding)       # ROS msgに変換

        if x > depth_data.width:
            x = 0
        if y > depth_data.height:
            y = 0
        z_int16 = d_img_array[int(y),int(x)]

        pose = Pose()
        pose.position = Point(x,y, (float)(z_int16))
        pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        op = ObjPose()
        op.class_name="sports ball"
        op.poses = np.array([pose])

        opl = ObjPoseList()
        opl.header = depth_data.header
        opl.obj_poses = np.array([op])

        self.pub_target.publish(opl)
        self.pub_d_image.publish(depth_data)
        self.pub_d_cinfo.publish(d_camera_info)
        self.pub_dbg_1.publish(send_img_circle)
        self.pub_dbg_2.publish(send_d_img)


if __name__ == '__main__':
    rospy.init_node('target_notifier')

    node = testNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

