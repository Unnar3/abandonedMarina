#!/usr/bin/env python
 
import rosbag
 
with rosbag.Bag('new.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('sample.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
			print msg.header.stamp
			outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)