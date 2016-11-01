#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

from model import Poly3
from basestation_selector import BasestationSelector
from ekf import EKF
from basestation import Basestation
from json_extractor import NoMeasurementException

# model in meters
COEFFS = [-0.1416, 2.311, -14.76, 36.79]

# model in centimeters
# COEFFS = [-1.416e-07, 0.0002311, -0.1476, 36.79]

EVERY_MINUTE = 0.1667
EVERY_SECOND = 1

def createBasestationList():
    return [ Basestation("192.168.1.19", 2, 2),
                Basestation("192.168.1.17", 3, 3)
            ]


def publishPosition(position, publisher):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = '/odom'
    msg.child_frame_id = '/base_link'

    msg.pose.pose.position = Point(position[0], position[1], 0)

    publisher.publish(msg)


def runNode():
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    rate = rospy.Rate(EVERY_SECOND)

    basestations = createBasestationList()
    model = Poly3(COEFFS)
    selector = BasestationSelector(basestations)
    ekf = EKF("2022", 1, 0.1, basestations, model, selector)
    miss = 2

    while not rospy.is_shutdown():
        try:
            if miss >= 2:            # after two misses it is assumed the target is not in range
                rate = rospy.Rate(EVERY_MINUTE)  # once in a minute
                position = ekf.setInitialPositionToCloserBasestation()
            else:
                position = ekf.ekfIteration()

            publishPosition(position, pub)
            rate = rospy.Rate(EVERY_SECOND)  # normal rate
            miss = 0

        except NoMeasurementException:      # no measurements means the target is not in range
            miss += 1

        rate.sleep()

if __name__ == '__main__':
    try:
        runNode()
    except rospy.ROSInterruptException:
        pass
