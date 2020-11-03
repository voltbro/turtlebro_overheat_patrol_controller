#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
voltbro 2020
"""

import rospy
from turtlebro_overheat_sensor.msg import HeatAlert
from std_msgs.msg import Float32MultiArray, String, Bool
from datetime import datetime


class HeatController(object):
    """

    """
    def __init__(self):

        # start node
        rospy.init_node('heat_controller', log_level=rospy.DEBUG)
        rospy.loginfo('HeatController: start heat detector node')

        self._output_topic = 'heat_controller_output'
        self._input_topic = 'heat_controller_input'
        self._sensor_topic = 'heat_sensor_output'
        self._patrol_topic = 'patrol_control'
        self._led_topic = 'alarm_led'

        self._rate = rospy.Rate(10)

        self._output_pub = rospy.Publisher(self._output_topic, String, queue_size=10)
        self._heat_sub = rospy.Subscriber(self._sensor_topic, HeatAlert, self._heat_callback)
        self._patrol_pub = rospy.Publisher(self._patrol_topic, String, queue_size=10)
        self._led_pub = rospy.Publisher(self._led_topic, Bool, queue_size=10)
        # self._alarm_led_pub = rospy.Publisher(self._alarm_led_topic, Bool, queue_size=10)
        self._input_sub = rospy.Subscriber(self._input_topic, String, self._input_callback)

        # When you create a subscriber, a new process is created in which messages are processed.
        # A new process is created for each subscriber.
        # That is, if your node is subscribed to two topics, then its message handlers will work
        # in the parallel threads. And in one more thread, the main loop of the program runs.
        # When the main loop ends, the node also stops message processing threads.

        # start work loop
        self._run()

    def bye(self):
        # In this method, you can add actions that you would like
        # to take after the node is finished, such as stopping the motors or turning off the alarm LED.

        # for now it is only a stub
        print("bye")

    def _heat_callback(self, msg):
        # when a message is received in a topic,
        # correspond function is called in Subscriber`s thread.
        # the new message will not be processed
        # until the previous message has been processed

        # that function is only a stub. add your own logic here.
        rospy.loginfo("HeatController: heat report received")
        for i in msg.__slots__:
            rospy.loginfo("{} : {}".format(i, getattr(msg, i)))

    def _send_message_to_led(self, state):
        # state must be True or False

        # this method sends msg to arduino`s led topic

        # look here for details
        # https://github.com/voltbro/turtlebro_overheat_sensor/blob/main/readme.md#launch
        self._led_pub.publish(state)

    def _input_callback(self, msg):
        # when a message is received in a topic,
        # correspond function is called in Subscriber`s thread.
        # the new message will not be processed
        # until the previous message has been processed
        if msg.data == 'start':
            self._start_patrol()

        elif msg.data == 'skip':
            self._skip_current_point()

        elif msg.data == 'stop':
            self._stop_patrol()

    def _run(self):
        while not rospy.is_shutdown():
            # That is the main loop
            #
            # This loop runs in its own thread. Processing incoming messages from other nodes works in other threads.
            # The work in these threads runs in parallel.
            # For example, to pass information to the main loop about what data was in the incoming message,
            # you need to create a class variable. In the function for processing incoming messages,
            # you can put data into this variable,
            # and in the main loop you can constantly check if its value has changed.
            # Similarly you can transfer data between message processing threads from different topics.

            # Add your work logic here
            self._rate.sleep()

    def _start_patrol(self):
        # that function is only a stub. add your own logic here.
        rospy.loginfo("HeatController: start command received")

    def _stop_patrol(self):
        # that function is only a stub. add your own logic here.
        rospy.loginfo("HeatController: stop command received")

    def _skip_current_point(self):
        # that function is only a stub. add your own logic here.
        rospy.loginfo("HeatController: skip command received")


if __name__ == '__main__':
    try:
        hd = HeatController()

    except rospy.ROSInterruptException:
        hd.bye()
