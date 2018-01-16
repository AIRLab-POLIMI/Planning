#!/usr/bin/env python
import os
import sys
import rospy
import roslaunch

from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg  import PoseWithCovarianceStamped

def launch(i):

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    args = ['rrt_planning', 'nh.launch']
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(args)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()
    rospy.sleep(2)
    f = open(os.getcwd() + '/data/map.exp')
    row = f.readlines()[int(i)]
    f.close()
    publish(row)
    rospy.spin()

def publish(row):
    #start
    rospy.logfatal('starting')
    rospy.logfatal(row)
    data = row.split('_', 7)

    start = PoseWithCovarianceStamped()

    start.header.stamp = rospy.Time.now()
    start.header.frame_id = 'map'
    start.pose.pose.position.x = float(data[0])
    start.pose.pose.position.y = float(data[1])
    start.pose.pose.position.z = 0
    start.pose.pose.orientation.x = 0
    start.pose.pose.orientation.y = 0
    start.pose.pose.orientation.z = float(data[2])
    start.pose.pose.orientation.w = float(data[3])

    global s_pub
    s_pub.publish(start)
    rospy.logfatal('published start')
    rospy.sleep(1)

    #goal
    rospy.logfatal('starting goal')
    goal = PoseStamped()

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose.position.x = float(data[4]);
    goal.pose.position.y = float(data[5]);
    goal.pose.position.z = 0
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = float(data[6]);
    goal.pose.orientation.w = float(data[7]);

    global g_pub
    g_pub.publish(goal)
    rospy.logfatal('published goal')

if __name__ == '__main__':
    i = sys.argv[1]
    global s_pub, g_pub, sub
    s_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    g_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('test', anonymous=True)
    launch(i)
