#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

from model import Poly3
from beacon_selector import BeaconSelector
from ekf import EKF
from basestation import Basestation
from json_extractor import NoMeasurementException

# (-0.1416)  2.311  -14.76  36.79;
COEFFS = [-0.1416, 2.311, -14.76, 36.79]

# (-1.416e-07)*(sqrt(x.^2 + y.^2)).^3 + 0.0002311*(x.^2 + y.^2) + -0.1476*sqrt(x.^2 + y.^2) + 36.79
#COEFFS = [-1.416e-07, 0.0002311, -0.1476, 36.79]

EVERY_MINUTE = 0.1667
EVERY_SECOND = 1


def createBeaconList():
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

    beacons = createBeaconList()
    model = Poly3(COEFFS)
    selector = BeaconSelector(beacons)
    ekf = EKF("2022", 1, 0.1, beacons, model, selector)
    miss = 2

    while not rospy.is_shutdown():
        try:
            if miss >= 2:            # after two misses it is assumed the target is not in range
                rate = rospy.Rate(EVERY_MINUTE)  # once in a minute
                position = ekf.setInitialPositionToCloserBeacon()
            else:
                print "ekf"
                position = ekf.ekf()
                print "end ekf", position

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
