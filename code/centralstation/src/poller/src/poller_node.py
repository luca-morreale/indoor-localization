#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from poller.msg import Measurement, MeasurementList

import socket

from client import Client
from orderedset import OrderedSet
from json_handler import containsMeasurements, extractJson


class Poller(object):
    def __init__(self):
        rospy.init_node('poller_node')
        self.rate = rospy.Rate(3)   # 3hz
        self.extractParams()
        self.createCommunicators()
        self.request_list = OrderedSet([])

    def createCommunicators(self):
        self.client = Client(10019)
        self.measurements_publisher = rospy.Publisher('measurements', MeasurementList, queue_size=10)
        self.request_subscriber = rospy.Subscriber('measurements_request', String, self.pushbackRequest)

    def extractParams(self):
        self.debug = rospy.get_param("/poller_node/debug")
        self.extractBasestationFromParams()

    def extractBasestationFromParams(self):
        stations = rospy.get_param("/poller_node/basestations")
        self.storeBasestation(stations)

    def storeBasestation(self, stations):
        self.basestations = set()
        for station in stations:
            self.basestations.add(station)

    def debug_msg(self, msg):
        if self.debug:
            print msg

    def pushbackRequest(self, msg):
        self.debug_msg('Arrived message: ' + str(msg))
        if str(msg.data) in self.basestations:
            self.request_list.add(msg.data)
            self.debug_msg('Added to request ' + str(msg))

    def measurementsLoop(self):
        while not rospy.is_shutdown():
            while not self.request_list.isEmpty():
                station_address = self.request_list.pop()
                self.debug_msg('Serving request: ' + str(station_address))
                self.serveRequest(station_address)
            self.debug_msg('Empty list')
            self.rate.sleep()

    def serveRequest(self, station_address):
        try:
            data = self.pollStation(station_address)
            if containsMeasurements(data):
                self.publishMeasuements(extractJson(data), station_address)
        except socket.error:
            pass

    def pollStation(self, station_address):
        return self.client.pollBasestation(station_address)

    def publishMeasuements(self, measurs, station):
        msg = MeasurementList()
        for el in measurs:
            msg.data.append(self.generateMeasurement(el))
        msg.basestation = station
        msg.header.stamp = rospy.Time.now()
        self.debug_msg('Publishing measurement: ' + str(msg))
        self.measurements_publisher.publish(msg)

    def generateMeasurement(self, element):
        tmp = Measurement()
        tmp.tag = element['id_tag'].encode('utf-8')
        tmp.measurement = int(element['rssid'])
        return tmp


if __name__ == '__main__':
    try:
        node = Poller()
        node.measurementsLoop()
    except rospy.ROSInterruptException:
        pass
