# System Lib
import threading
# 3rd-party lib
import numpy as np
import cv2
# pytorch
import torch
# ros
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


def loop():
    while not rospy.is_shutdown():
        rospy.spin()


def marker_gen(frame_id, class_name, idx, score, pos):
    marker = Marker()

    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame_id
    marker.ns = class_name
    marker.id = idx
    marker.type = marker.SPHERE
    marker.action = marker.ADD

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]

    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    hsv = np.uint8([[[score * 180, 255, 255]]])
    color = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0][0]

    marker.color.a = 0.5
    marker.color.r = color[0] / 255.0
    marker.color.g = color[1] / 255.0
    marker.color.b = color[2] / 255.0

    return marker


class ObjectDetection:
    def __init__(self, opencv_vis=False):
        # === Class parameters ===
        self.opencv_vis = opencv_vis
        subs_img_topic = "/camera/color/image_raw"
        subs_img_d_topic = "/camera/aligned_depth_to_color/image_raw"
        subs_cam_info_topic = "/camera/aligned_depth_to_color/camera_info"

        # === Vision model init ===
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        # load a model pre-trained pre-trained on COCO
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5l')
        self.model.to(self.device)
        # Only for evaluation
        self.model.eval()

        # === ROS init ===
        self.lock = threading.RLock()
        with self.lock:
            self.depth_img = Image()
            self.camera_info = CameraInfo()
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber(subs_img_topic, Image, self.callback, queue_size=1, buff_size=2**22)
        self.img_d_sub = rospy.Subscriber(subs_img_d_topic, Image, self.depth_trans, queue_size=1, buff_size=2**21)
        self.cam_info_sub = rospy.Subscriber(subs_cam_info_topic, CameraInfo, self.intrinsic_trans, queue_size=1)
        self.img_pub = rospy.Publisher("/object_detection/image_anchor", Image, queue_size=1)
        self.markers_pub = rospy.Publisher("/object_detection/markers", MarkerArray, queue_size=1)

        # === Main Loop ===
        loop()

    def callback(self, img_msg):
        with self.lock:
            img_d_msg = self.depth_img
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            cv_image_d = self.bridge.imgmsg_to_cv2(img_d_msg)
        except CvBridgeError as e:
            print(e)
            return

        # Clip the image to improve the efficiency for ROI
        bias_y = 342
        bias_x = 374
        # Note the x and y are inverse
        image_rgb = cv2.cvtColor(cv_image[bias_y:, bias_x:, :], cv2.COLOR_BGR2RGB)

        predictions = self.model([image_rgb], size=640)

        # Parameters
        pred = predictions.xyxy[0].cpu()
        boxes = pred[:, 0:4].detach().numpy().astype(np.int64)
        boxes[:, 0] += bias_x
        boxes[:, 1] += bias_y
        boxes[:, 2] += bias_x
        boxes[:, 3] += bias_y
        scores = pred[:, 4].detach().numpy()
        labels = pred[:, 5].detach().numpy().astype(np.int64)

        markers = MarkerArray()
        # Visualization
        for idx, box in enumerate(boxes):
            class_name = predictions.names[labels[idx]]
            cv2.rectangle(cv_image, box[0:2], box[2:], (255, 0, 0), 1)
            cv2.rectangle(cv_image, box[0:2], (box[0] + 80, box[1] + 20), (255, 0, 0), -1)
            cv2.putText(cv_image, class_name, (box[0] + 6, box[1] + 16), cv2.FONT_HERSHEY_PLAIN, 1,
                        (255, 255, 255), 1, cv2.LINE_AA)

            frame_id = img_msg.header.frame_id
            pos = [(box[0]+box[2])//2, (box[1]+box[3])//2]
            depth = cv_image_d[pos[1], pos[0]]*1e-3
            # print(class_name, self.de_projection(pos, depth))
            markers.markers.append(marker_gen(frame_id, class_name, idx, scores[idx], self.de_projection(pos, depth)))

        if self.opencv_vis:
            cv2.imshow("object_detection", cv_image)
            cv2.waitKey(1)

        img_msg_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        img_msg_pub.header.frame_id = img_msg.header.frame_id
        self.img_pub.publish(img_msg_pub)
        self.markers_pub.publish(markers)

    def depth_trans(self, msg):
        with self.lock:
            self.depth_img = msg

    def intrinsic_trans(self, msg):
        with self.lock:
            self.camera_info = msg

    def de_projection(self, pixel, depth):
        with self.lock:
            cam_info = self.camera_info
        coeffs = cam_info.D
        ppx = cam_info.K[2]
        ppy = cam_info.K[5]
        fx = cam_info.K[0]
        fy = cam_info.K[4]

        x = (pixel[0] - ppx) / fx
        y = (pixel[1] - ppy) / fy

        r2 = x * x + y * y
        f = 1 + coeffs[0] * r2 + coeffs[1] * r2 * r2 + coeffs[4] * r2 * r2 * r2
        ux = x * f + 2 * coeffs[2] * x * y + coeffs[3] * (r2 + 2 * x * x)
        uy = y * f + 2 * coeffs[3] * x * y + coeffs[2] * (r2 + 2 * y * y)
        x = ux
        y = uy
        point = [depth * x, depth * y, depth]
        return point


if __name__ == "__main__":
    rospy.init_node("object_detection")
    ObjectDetection()






