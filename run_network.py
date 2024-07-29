#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import torch
import os
import matplotlib.cm as cm
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import Marker

from models.matching import Matching
from models.utils import (AverageTimer, VideoStreamer,
                          make_matching_plot_fast, frame2tensor, read_image, process_resize)

class SuperglueNetworkNode:
    def __init__(self):
        rospy.init_node('superglue_network_node', anonymous=True)
        rospy.loginfo("Starting superglue network!")
        # Create a CvBridge instance
        self.bridge = CvBridge()

        #self.camera_k = np.array([[fx1, 0, cx1], [0, fy1, cy1], [0, 0, 1]], dtype=np.float32)

        # Setting up Pytorch for Superglue
        torch.set_grad_enabled(False)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # Using all the default values
        self.config = {
            'superpoint': {
                'nms_radius': 4,
                'keypoint_threshold': 0.005,
                'max_keypoints': -1
            },
            'superglue': {
                'weights': 'indoor',
                'sinkhorn_iterations': 20,
                'match_threshold': 0.2,
            }
        }
        
        self.resize = [256, 256]
        # self.resize = [640, 480]
        self.matching = Matching(self.config).eval().to(self.device)
        self.keys = ['keypoints', 'scores', 'descriptors']

        # a path to the photo of groundtruth + process ground truth for comparision
        
        # padding the groundtrtuh


        self.groundtruth = "/home/karlym/data/chair_segment.png" # 88 by 130

        groundtruth_image = cv2.imread(str(self.groundtruth)) 
        groundtruth_image = cv2.copyMakeBorder(groundtruth_image, 63, 63, 84, 84, cv2.BORDER_CONSTANT) # for segmented chair

        if groundtruth_image is None:
            rospy.logerr("Error loading groundtruth")
            return

        w, h = groundtruth_image.shape[1], groundtruth_image.shape[0]
        w_new, h_new = process_resize(w, h, self.resize) # rescale
        scales = (float(w) / float(w_new), float(h) / float(h_new))
        rescaled_gt_image = cv2.resize(groundtruth_image, (w_new, h_new), interpolation=cv2.INTER_AREA)
        rescaled_gt_image = cv2.cvtColor(rescaled_gt_image, cv2.COLOR_RGB2GRAY)
        gt_inp = frame2tensor(rescaled_gt_image, self.device)

        self.gt_data = self.matching.superpoint({'image': gt_inp})
        self.gt_data = {k+'0': self.gt_data[k] for k in self.keys}
        self.gt_data['image0'] = gt_inp
        self.gt_frame = rescaled_gt_image
        self.gt_image_id = 0
    
        # Set up a subscriber to the image topic
        self.image_subscriber = Subscriber("/gibson_ros/camera/rgb/image", Image)
        self.point_publisher = rospy.Publisher('matched_points_camera', PoseArray, queue_size=10)
        self.timer_sub = Subscriber('/sim_clock', Marker)
        self.sync = ApproximateTimeSynchronizer([self.image_subscriber, self.timer_sub], queue_size = 1, slop = 0.3)
        self.sync.registerCallback(self.image_callback)

    def image_callback(self, msg, marker):
        torch.set_grad_enabled(False)
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV image: %s" % str(e))
            return
    
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        if grey_image is None:
            rospy.logerr("Cannot open image")
            return

        w, h = grey_image.shape[1], grey_image.shape[0]
        w_new, h_new = process_resize(w, h, self.resize)
        scales = (float(w) / float(w_new), float(h) / float(h_new))
        image = cv2.resize(grey_image, (w_new, h_new), interpolation=cv2.INTER_AREA)
        inp = frame2tensor(image, self.device)
        
        pred = self.matching({**self.gt_data, 'image1':inp})
        kpts0 = self.gt_data['keypoints0'][0].cpu().numpy()
        kpts1 = pred['keypoints1'][0].cpu().numpy()
        matches = pred['matches0'][0].cpu().numpy()
        confidence = pred['matching_scores0'][0].cpu().numpy()

        valid = matches > -1
        mkpts0 = kpts0[valid]
        mkpts1 = kpts1[matches[valid]]
        valid_confidence = confidence[valid]

        # send transform
        if len(mkpts1 > 10):
            self.publish_matched_points(mkpts1, valid_confidence)

        color = cm.jet(confidence[valid])
        text = [
            'SuperGlue',
            'Keypoints: {}:{}'.format(len(kpts0), len(kpts1)),
            'Matches: {}'.format(len(mkpts0))
        ]
        k_thresh = self.matching.superpoint.config['keypoint_threshold']
        m_thresh = self.matching.superglue.config['match_threshold']
        small_text = [
            'Keypoint Threshold: {:.4f}'.format(k_thresh),
            'Match Threshold: {:.2f}'.format(m_thresh),
        ]
        out = make_matching_plot_fast(
            self.gt_frame, image, kpts0, kpts1, mkpts0, mkpts1, color, text,
            path="/home/karlym/test.jpg", show_keypoints=False, small_text=small_text)

        cv2.imshow('SuperGlue matches', out)
        cv2.waitKey(1)

    def publish_matched_points(self, matched, confidence):
        assert len(matched) == len(confidence)

        # Assuming the image size hasn't changed..
        frame_id = "camera_rgb_optical_frame"

        poses = []
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        for i, (x,y) in enumerate(matched):
            if confidence[i] > 0.5:
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = 0.0
                poses.append(pose)
        
        poseArray = PoseArray()
        poseArray.header = header
        poseArray.poses = poses

        if len(poses) == 0:
            rospy.logwarn("Some matches but no confidence..")
            return
        
        self.point_publisher.publish(poseArray)

    

    
        

def main():
    try:
        superglue_network_node = SuperglueNetworkNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

