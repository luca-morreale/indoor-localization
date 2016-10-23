import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

from ekf import EKF
from json_extractor import NoMeasurementException

coeffs = []


def createBeaconList():
    pass


def publishPosition(position, publisher):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = '/odom'
    msg.child_frame_id = '/base_link'

    msg.pose.pose.position = Point(position[0], position[1], 0)

    publisher.publish(msg)
    rospy.loginfo(msg)


def runNode():
    pub = rospy.Publisher('odom', String, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    # tag, dt, sensor_size, var_z, beacons, coeffs
    ekf = EKF("2022", 1, 1, 0.1, createBeaconList(), coeffs)
    miss = 0

    while not rospy.is_shutdown():

        try:
            position = ekf.ekf()

            publishPosition(position, pub)

            if miss >= 2:       # target is back in range so restore previous rate
                miss = 0
                rate = rospy.Rate(1)  # normal rate
        except NoMeasurementException:      # no measurements means the target is not in range
            miss += 1

        if miss == 2:       # if the target is not in range reduce the frequency of polling
            rate = rospy.Rate(0.0166)  # once in a minute

        rate.sleep()

if __name__ == '__main__':
    try:
        runNode()
    except rospy.ROSInterruptException:
        pass
