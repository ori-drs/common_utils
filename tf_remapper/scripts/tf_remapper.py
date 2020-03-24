#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg


def get_param(name):
    # Get the parameter, print an error if not available.
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        reason = 'Parameter ' + name + ' not found on server.'
        rospy.logerr(reason)
        rospy.signal_shutdown(reason)


if __name__ == '__main__':
    # Initialize the ROS node.
    rospy.init_node('tf_remapper')

    # Read the ROS parameters.
    original_child_frame_id = get_param('~original_child_frame_id')
    original_parent_frame_id = get_param('~original_parent_frame_id')

    new_child_frame_id = get_param('~new_child_frame_id')
    new_parent_frame_id = get_param('~new_parent_frame_id')

    frequency = get_param('~frequency')

    # Set up tf buffer, publisher and rate.
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(frequency)

    # Repeat until the node is shut down.
    while not rospy.is_shutdown():
        try:

            # Get the latest transformation from child to parent.
            transform_message = tf_buffer.lookup_transform(original_parent_frame_id, original_child_frame_id, rospy.Time(0))

            transform_message.header.frame_id = new_parent_frame_id
            transform_message.child_frame_id = new_child_frame_id

            broadcaster.sendTransform(transform_message)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Skip this update.
            rospy.logdebug(
                'Could not get transformation from %s to %s'%(original_parent_frame_id,original_child_frame_id))
            rate.sleep()
            continue


        rate.sleep()