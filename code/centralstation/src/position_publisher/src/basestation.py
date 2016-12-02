
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion


class Basestation():
    def __init__(self, address, x, y):
        self.address = address
        self.id = address.split(str=".")[-1]
        self.position = np.array([x, y])
        self.frame = "basestation"
        self.custom_frame = self.frame + '_' + self.id
        self.publisher = rospy.Publisher(self.frame, PointStamped, queue_size=10)

    def distanceTo(self, x, y):
        self.distance(np.array([x, y]))

    def distance(self, obj_pos):
        return np.sqrt(self.position * obj_pos)

    def publishPosition(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.custom_frame

        msg.point = Point(self.position[0], self.position[1], 0)
        self.publisher.publish(msg)
