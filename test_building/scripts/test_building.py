#!/usr/bin/env python
import roslib
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from tf.transformations import euler_from_quaternion
import rospkg

import yaml


class Robot:
	speed = 4.0
	def __init__(self, topic, start_pos):
		# positions are [x, y, angle]
		self.start_pos = start_pos
		self.topic     = topic

		self.setup()

	def setup(self):
		# move to start pos if necessary
		self.active = True;
		self.goal   = self.start_pos

		self.odom_sub  = rospy.Subscriber(self.topic + "/base_pose_ground_truth", Odometry, self.odom_callback)
		self.twist_pub = rospy.Publisher(self.topic + "/cmd_vel", Twist)

	def __repr__(self):
		return "%s(topic=%r, start_pos=%r)" % (
			self.__class__.__name__, self.topic, self.start_pos)		

	def odom_callback(self, odom):
		if self.active:
			self.cur_pos = odom.pose.pose.position

			q     = odom.pose.pose.orientation
			rpy   = euler_from_quaternion([q.x, q.y, q.z, q.w])
			angle = math.degrees(rpy[2])

			twist, self.active = self.twist_to_goal(self.cur_pos.x, self.cur_pos.y, angle)
			self.twist_pub.publish(twist)

	def twist_to_goal(self, x, y, angle):
		active = False

		twist = Twist()

		if abs(self.goal[0] - x) > 0.1 or abs(self.goal[1] - y) > 0.1:
			coeff          = self.speed / ((self.goal[0] - x) ** 2 + (self.goal[1] - y) ** 2)
			twist.linear.x = (self.goal[0] - x) * math.sqrt(coeff)
			twist.linear.y = (self.goal[1] - y) * math.sqrt(coeff)
			active         = True

		if abs(self.goal[2] - angle) > 3.0:
			twist.angular.z = 5 * math.tanh(self.goal[2] - angle)
			active          = True

		return twist, active

	def is_active(self):
		return self.active


class Door(Robot):
	def __init__(self, topic, start_pos, open_pos, closed_pos):
		# positions are [x, y, angle]
		self.start_pos  = start_pos
		self.closed_pos = closed_pos
		self.open_pos   = open_pos
		self.topic      = topic

		self.setup()

	def close_door(self):
		self.goal   = self.closed_pos
		self.active = True

	def open_door(self):
		self.goal   = self.open_pos
		self.active = True

class Person(Robot):
	def __init__(self, topic, start_waypoint, waypoints):
		self.start_waypoint = start_waypoint
		self.waypoints      = waypoints
		self.topic          = topic

		self.setup()

	def setup(self, doors):
		self.active        = True;
		self.goal          = self.waypoints[self.start_waypoint][:]
		self.next_waypoint = self.start_waypoint
		self.repeat        = False;
		self.doors         = doors
		self.too_close     = False;

		self.odom_sub  = rospy.Subscriber(self.topic + "/base_pose_ground_truth", Odometry, self.odom_callback)
		self.twist_pub = rospy.Publisher(self.topic + "/cmd_vel", Twist)

	def odom_callback(self, odom):
		pos = odom.pose.pose.position
		# self._open_doors(pos)


		if self.active:
			q     = odom.pose.pose.orientation
			rpy   = euler_from_quaternion([q.x, q.y, q.z, q.w])
			angle = math.degrees(rpy[2])
			twist, self.active = self.twist_to_goal(pos.x, pos.y, angle)

			self.twist_pub.publish(twist)

			# if we haven't already reached the goal
			if not self.active and self.next_waypoint != self.end:
				self.active = True
				self._next_waypoint()


	def _open_doors(self, pos):
		for door in self.doors:
			if math.sqrt((door.cur_pos.x - pos.x) ** 2 + (door.cur_pos.y - pos.y) ** 2) < 2.0:
				door.open_door()

	def _next_waypoint(self):
		if self.repeat == "loop":
			self.next_waypoint = (self.next_waypoint + self.direction) % len(self.waypoints)
		elif self.repeat == "oneway":
			self.next_waypoint += self.direction

			if self.next_waypoint == self.end:
				self.direction *= -1
				self.end = (len(self.waypoints) - 1) - self.end
		else:
			self.next_waypoint = (self.next_waypoint + self.direction) % len(self.waypoints)
		self.goal = self.waypoints[self.next_waypoint][:]


	def loop(self, direction=1):
		# will loop indefinitely
		self.move_to(-1, direction, "loop")

	def loop_outandback(self):
		self.move_to(len(self.waypoints) - 1, 1, "oneway")

	def move_to_end(self):
		self.move_to(len(self.waypoints) - 1)

	def move_to_start(self):
		self.move_to(0)

	def move_to(self, index, direction=None, repeat=False):
		# if in doubt, don't loop
		if direction == None:
			if index >= self.next_waypoint:
				self.direction = 1
			else:
				self.direction = -1
		else:
			self.direction = direction

		self.end    = index
		self.repeat = repeat
		self.active = True

		self._next_waypoint()


class EventHandler:
	def __init__(self):
		self.events = []
		self.elapsed = 0

	def add_event(self, time, func):
		self.events.append((time, func))
		self.events.sort(key=lambda x: x[0], reverse=True)

	def run(self):
		self.start_time = rospy.Time.now()

		r = rospy.Rate(15)
		while not rospy.is_shutdown() and len(self.events) > 0:
			if (rospy.Time.now() - self.start_time) >= self.events[-1][0]:
				self.events[-1][1]()
				print "calling function:",(self.start_time + self.events[-1][0]).to_sec()
				self.events.pop()
			r.sleep()
			print rospy.Time.now().to_sec()





if __name__=="__main__":
	rospy.init_node('test_building')

	r = rospkg.RosPack()

	f = open(r.get_path("test_building") + "/config/test_building.yaml")

	y = yaml.load(f)

	f.close()

	for door in y['doors']:
		door.setup()

	for person in y['people']:
		person.setup(y['doors'])

	e = EventHandler()

	e.add_event(rospy.Duration(0),  lambda :y['people'][0].move_to_end())

	# e.add_event(rospy.Duration(0),  lambda :y['doors'][10].open_door())
	# e.add_event(rospy.Duration(2),  lambda :y['people'][1].move_to_end())
	# e.add_event(rospy.Duration(2),  lambda :y['people'][2].move_to_end())
	# e.add_event(rospy.Duration(2),  lambda :y['people'][3].move_to_end())
	# e.add_event(rospy.Duration(5),  lambda :y['people'][4].move_to_end())
	# e.add_event(rospy.Duration(5),  lambda :y['people'][5].move_to_end())
	# e.add_event(rospy.Duration(5),  lambda :y['people'][6].move_to_end())
	# e.add_event(rospy.Duration(9),  lambda :y['people'][7].move_to_end())
	# e.add_event(rospy.Duration(9),  lambda :y['people'][8].move_to_end())
	# e.add_event(rospy.Duration(9),  lambda :y['people'][9].move_to_end())
	# e.add_event(rospy.Duration(13), lambda :y['doors'][10].close_door())

	e.run()


	# y['doors'][8].open_door()

	# while door.is_active() and not rospy.is_shutdown():
	# 	pass

	# for person in y['people']:
	# 	person.loop_outandback()

	# close all the doors
	# for door in y['doors']:
	# 	door.setup()
	# 	door.close_door()

	# for door in y['doors']:
	# 	while door.is_active() and not rospy.is_shutdown():
	# 		pass


	print "done\n"

	rospy.spin()