#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from poller.msg import Measurement, MeasurementList

import socket

from client import Client
from orderedset import OrderedSet
from basestation import Basestation
from json_handler import containsMeasurements, extractJson


class Poller(object):
    def __init__(self):
        rospy.init_node('poller_node')
        self.rate = rospy.Rate(3)   # 3hz
        self.extractBasestationFromParams()
        self.createCommunicators()
        self.request_list = OrderedSet([])

    def createCommunicators(self):
        self.client = Client(10019)
        self.measurements_publisher = rospy.Publisher('measurements', MeasurementList, queue_size=10)
        self.request_subscriber = rospy.Subscriber('measurements_request', String, self.pushbackRequest)

    def extractBasestationFromParams(self):
        stations = rospy.get_param("/poller_node/basestations")
        self.storeBasestation(stations)

    def storeBasestation(self, stations):
        self.basestations = []
        for station in stations:
            self.basestations.append(Basestation(station[0], float(station[1]), float(station[2])))

    def pushbackRequest(self, msg):
        self.request_list.add(msg.data)

    def measurementsLoop(self):
        while not rospy.is_shutdown():
            while not self.request_list.isEmpty():
                station_address = self.request_list.pop()
                self.serveRequest(station_address)
            self.rate.sleep()

    def pollStation(self, station_address):
        return self.client.pollBasestation(station_address)

    def serveRequest(self, station_address):
        try:
            data = self.pollStation(station_address)
            if containsMeasurements(data):
                self.publishMeasuements(extractJson(data), station_address)
        except socket.error:
            pass

    def publishMeasuements(self, measurs, station):
        msg = MeasurementList()
        for el in measurs:
            msg.data.append(self.generateMeasurement(el))
        msg.basestation = station
        msg.header.stamp = rospy.Time.now()
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
