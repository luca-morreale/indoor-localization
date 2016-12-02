
import rospy
from visualization_msgs.msg import Marker


class CircumferencePublisher(object):

    def __init__(self, model, tag_color, alpha):
        self.model = model
        self.alpha = alpha
        self.tag_color = tag_color
        self.shape = Marker.SPHERE
        self.publisher = rospy.Publisher('/measurement_debug', Marker, queue_size=10)

    def setPosition(self, marker, position):
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0

    def setRadius(self, marker, measurement):
        radius = self.model.value(measurement)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = 0.0

    def setColor(self, marker, tag):
        marker.color.r = self.tag_color[tag][0]
        marker.color.g = self.tag_color[tag][1]
        marker.color.b = self.tag_color[tag][2]
        marker.color.a = self.alpha

    def publish_shape(self, position, measurement, tag, namespace):

        marker = Marker()
        marker.header.frame_id = '/measurement_debug'
        marker.header.stamp = rospy.Time.now()

        marker.ns = namespace
        marker.id = int(tag)
        marker.action = Marker.ADD

        self.setPosition(marker, position)
        self.setRadius(marker, measurement)
        self.setColor(marker, tag)

        marker.lifetime = rospy.Duration()

        self.publisher.publish(marker)
