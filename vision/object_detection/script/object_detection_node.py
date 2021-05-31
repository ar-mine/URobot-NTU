# 3rd-party lib
import numpy as np
import cv2
# pytorch
import torch
import torchvision.transforms as transforms
# ros
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ObjectDetection:
    def __init__(self):
        # === Vision model init ===
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        # load a model pre-trained pre-trained on COCO
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5l')
        self.model.to(self.device)
        # Only for evaluation
        self.model.eval()
        self.transf = transforms.ToTensor()

        # Publisher and Subscriber
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1, buff_size=2**22)
        self.img_pub = rospy.Publisher("/object_detection/image_anchor", Image, queue_size=1)

        # === Main Loop ===
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def callback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv_image = cv_image[342:, 374:, :]
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        predictions = self.model([image_rgb], size=640)

        # Parameters
        pred = predictions.xyxy[0].cpu()
        boxes = pred[:, 0:4].detach().numpy().astype(np.int64)
        scores = pred[:, 4].detach().numpy()
        labels = pred[:, 5].detach().numpy().astype(np.int64)

        # Visualization
        labels_name = []
        for idx, box in enumerate(boxes):
            if scores[idx] >= 0.0:
                cv2.rectangle(cv_image, box[0:2], box[2:], (255, 0, 0))
                labels_name.append(predictions.names[labels[idx]])

        cv2.imshow("object_detection", cv_image)
        cv2.waitKey(1)

        img_msg_pub = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.img_pub.publish(img_msg_pub)


if __name__ == "__main__":
    rospy.init_node("object_detection")
    ObjectDetection()






