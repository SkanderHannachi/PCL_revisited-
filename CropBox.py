#!/usr/bin/env python

import math
import rospy
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

MARKER_LIFETIME = 2
BUFFSIZE = 1 

TOPIC_CLOUD_IN = '/velodyne_points/lidar'
TOPIC_CLOUD_FILTER = '/cropBox_displayed'
TOPIC_MARKER = '/display_new_cropBox'

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

class Crop : 
	"""Crop is the class to compute and display cropboxes in python.
		instance takes : 
			@min_vertice     :  minPoint to define cropBox : type =  L{tuple}
			@max_vertice     :  maxPoint to define cropBox : type =  L{tuple}
			@cloud_in_topic  :  topic's name               
			@topic_pub       :  topic's name
	"""
	def __init__ (self, min_vertice=None, max_vertice=None, cloud_in_topic=None, topic_pub_cloud=None, topic_pub_marker= None, marker_type = None): 
		self.min_vertice = min_vertice
		self.max_vertice = max_vertice
		self.cloud_in_topic   = cloud_in_topic
		self.topic_pub_cloud  = topic_pub_cloud
		self.topic_pub_marker = topic_pub_marker 
		self.publisher_cloud_in    = rospy.Publisher(self.topic_pub_cloud, PointCloud2, queue_size=BUFFSIZE)
		self.pubisher_marker_box   = rospy.Publisher(self.topic_pub_marker, Marker, queue_size=BUFFSIZE) 
		self.cloud_in         = PointCloud2()
		self.cloud_in_ls      = list(pc2.read_points(self.cloud_in, field_names = ("x", "y", "z"), skip_nans=True))
		self.marker           = Marker() 
		self.marker_type      = marker_type
		rospy.Subscriber(TOPIC_CLOUD_IN, PointCloud2, self.callback_get_cloud)
		self.init_marker()
	
	def init_marker(self): 
		self.marker.lifetime = rospy.Duration(MARKER_LIFETIME)
		self.marker.type = Marker.CUBE 
		self.marker.action = Marker.ADD 
		self.marker.header.frame_id = "robot"
		self.marker.pose.position.x = self.min_vertice[0]
		self.marker.pose.position.y = 0
		self.marker.pose.position.z = self.min_vertice[2]
		self.marker.scale.x = self.max_vertice[0] - self.min_vertice[0]
		self.marker.scale.y = self.max_vertice[1] - self.min_vertice[1]
		self.marker.scale.z = self.max_vertice[2] - self.min_vertice[2]
		self.marker.color.r = 0.5
		self.marker.color.g = 1.0
		self.marker.color.b = 0
		self.marker.color.a = 0.3
	
	def display_marker(self): 
		self.pubisher_marker_box.publish(self.marker) 

	def sort_cloud(self, axis): 
		"""Depending on which axis, this method return sorted PointCloud.
			@axis : which field (could be x, y or z) type : L{type(field_names)}
		"""
		return  sorted(self.cloud_in_ls, key=lambda tup: tup[axis])

	def callback_get_cloud(self, data):
		self.cloud_in = data
		self.cloud_in_ls = list(pc2.read_points(self.cloud_in, field_names = ("x", "y", "z"), skip_nans=True))
		header = data.header
		self.filter_crop(header, self.cloud_in_ls)
		self.display_marker()

	def filter_crop(self, header, points_list): 
		"""core function, takes 2 arguments : 
			@header      : type = L{std_msgs.msg.Header}
			@points_list : type = iterable     
		"""
		cloud_filtered=[]
		for point in points_list: 
			if ((self.min_vertice[0] < point[0] < self.max_vertice[0]) and (self.min_vertice[1] < point[1] < self.max_vertice[1]) and (self.min_vertice[2] < point[2] < self.max_vertice[2])):
				cloud_filtered.append(point)
		self.publisher_cloud_in.publish(pc2.create_cloud_xyz32(header = header, points = cloud_filtered))
	
	
if __name__ == "__main__": 
	rospy.init_node('test_for_crop', log_level=rospy.DEBUG)
	Crop(min_vertice=(2, -0.5, 1), max_vertice=(5, 0.5, 2),cloud_in_topic=TOPIC_CLOUD_IN, topic_pub_cloud=TOPIC_CLOUD_FILTER, topic_pub_marker=TOPIC_MARKER)
	rospy.spin()