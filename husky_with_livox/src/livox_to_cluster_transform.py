#!/usr/bin/env python3
"""
Bounding Boxes are published from the hbdscan_clusterizer node, but the position of the bounding
boxes aren't with respect to the livox sensor, hence the tf tree must be updated, i.e.:
a transform should be published between livox frame_id and cluster frame_id, which is exactly
what this node does:

1. Subscribes to the bounding boxes published by the hbdscan_clusterizer.py
2. Creates the transform between livox and cluster frame_ids with the updated
position of the bounding box.

Author: Henrique Poleselo
November 25th 2022.
"""

import rospy
import tf
from jsk_recognition_msgs.msg import BoundingBoxArray

LIVOX_FRAME_ID = 'livox'
BOUNDING_BOX_FRAME_ID = 'cluster'

rospy.init_node('livox_to_cluster_tf', anonymous=True)

def transform_pose_from_livox_to_cluster(bounding_box_msg):
    br = tf.TransformBroadcaster()
    #rospy.loginfo(f"Sending transform between {LIVOX_FRAME_ID} and cluster...")
    for bounding_box in bounding_box_msg.boxes:
        br.sendTransform((bounding_box.pose.position.x, bounding_box.pose.position.y, bounding_box.pose.position.z),
                        (0.0, 0.0, 0.0, 1),
                        rospy.Time.now(),
                        BOUNDING_BOX_FRAME_ID,
                        LIVOX_FRAME_ID)

if __name__ == '__main__':
    # Sets up subscriber for transforming incoming bounding boxes frame_ids to the livox frame_id
    
    # ! RViz should be launched ONLY after this package is running properly (Adjust that in the launch file)
    rospy.Subscriber("cluster_bounding_boxes", BoundingBoxArray, transform_pose_from_livox_to_cluster)
    rospy.spin()