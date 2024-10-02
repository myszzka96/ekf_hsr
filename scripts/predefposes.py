#!/usr/bin/env python3

import json
import os
import rospy
import subprocess
import sys

from std_msgs.msg import String

from visualization_msgs.msg import Marker, MarkerArray

from rospkg import RosPack; rp = RosPack()
package_name = os.path.dirname(os.path.dirname(__file__))
repo_path = rp.get_path(package_name)


sys.path.append(repo_path)

from scripts.hsr_simple_mover import hsr_simple_mover
from scripts.general_helpers.general_helper import GeneralHelper
from scripts.general_helpers.rviz_helper import RvizHelper

class hsr_lab_mover:
    def read_locations_json(self, locations_json_file: str) -> dict:
        locations_json_path = os.path.join(repo_path, 'config', 'robot_move', locations_json_file)

        locations_json = json.load(open(locations_json_path))

        return locations_json
    
    def configure_node_rate(self, hz):
        self.node_rate = hz
        self.node_time_interval = 1/self.node_rate

    def establish_rviz_pubs(self):
        self.locations_marker_array_pub = rospy.Publisher(f'{self.class_name}/locations/markers',
                                                          MarkerArray,
                                                          queue_size=10)
        
        self.locations_label_markers_array_pub = rospy.Publisher(f'{self.class_name}/locations_label/markers',
                                                                 MarkerArray,
                                                                 queue_size=10)
            
    def __init__(self):
        rospy.init_node('hsr_lab_mover', anonymous=True)

        self.general_helper = GeneralHelper()
        self.rviz_helper = RvizHelper()

        self.class_name = self.__class__.__name__

        self.locations_json = self.read_locations_json(locations_json_file='rc_lab_pos.json')
        print(f'locations_json:')
        self.general_helper.pprint_dict(dict=self.locations_json)

        self.locations_list = self.locations_json['routes']['lap'].items()

        self.configure_node_rate(hz=10)

        self.establish_rviz_pubs()

        print(f'Listening for start signal...')

        rospy.Timer(rospy.Duration(self.node_time_interval), self.timed_cb)
        rospy.Subscriber('/isaac_sim/robot_behavior_start', String, self.move_behavior_cb)
        rospy.on_shutdown(self.shutdown_cb)

        rospy.spin()

    def timed_cb(self, event):
        self.publish_move_locations()

    def publish_move_locations(self):
        location_markers, locations_label_markers = [], []
        label_position_counts = {}

        for i, kv in enumerate(self.locations_list):
            k, v = kv
            x, y = v[0], v[1]

            # Track how many times the same position has been encountered for label markers
            if (x, y) in label_position_counts:
                label_position_counts[(x, y)] += 1
            else:
                label_position_counts[(x, y)] = 1

            z_offset = 0.1 * label_position_counts[(x, y)]  # Adjust height only for label markers

            # Location marker (unchanged)
            location_marker = self.rviz_helper.generate_marker(ns='locations',
                                                            frame_id='map',
                                                            marker_type=Marker.CUBE,
                                                            marker_action=Marker.ADD,
                                                            marker_id=i,
                                                            marker_scale=(0.02, 0.02, 0.02),
                                                            marker_color=(0, 0, 0, 1),
                                                            marker_position=[x, y, 0])

            # Location label marker (with z_offset to prevent overlap)
            location_label_marker = self.rviz_helper.generate_marker(ns='locations_labels',
                                                                    frame_id='map',
                                                                    marker_type=Marker.TEXT_VIEW_FACING,
                                                                    marker_action=Marker.ADD,
                                                                    marker_id=i,
                                                                    marker_text=str(k),
                                                                    marker_scale=(0.1, 0.1, 0.1),
                                                                    marker_color=(1, 1, 1, 1),
                                                                    marker_position=[x, y, 0 + z_offset])

            location_markers.append(location_marker)
            locations_label_markers.append(location_label_marker)

        location_markers = MarkerArray(location_markers)
        locations_label_markers = MarkerArray(locations_label_markers)

        self.locations_marker_array_pub.publish(location_markers)
        self.locations_label_markers_array_pub.publish(locations_label_markers)

    def move_behavior(self):
        self.simple_mover = hsr_simple_mover()

        self.simple_mover.move_robot_to_go()
        # simple_mover.move_base_to_origin() # If localization is good, this will work correctly. 

        for k, v in self.locations_list:
            print(f'k: {k}, v: {v}')

            self.simple_mover.move_robot_to_go()

            x,y,theta = v

            self.simple_mover.move_base_abs_goal(x=x, y=y, theta=theta)

            rospy.sleep(1)

    def delete_all_markers(self, ns):
        print(f'Deleting all markers: {ns}')

        markers = []

        marker = Marker()
        marker.ns = ns
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL

        markers.append(marker)

        marker_array = MarkerArray(markers)

        if ns == 'locations':
            self.locations_marker_array_pub.publish(marker_array)

        if ns == 'locations_label':
            self.locations_label_markers_array_pub.publish(marker_array)

    def shutdown_cb(self):
        self.delete_all_markers(ns='locations')
        self.delete_all_markers(ns='locations_label')
    
    def move_behavior_cb(self, data):
        data_str = data.data
        print(f'received data: {data_str}')

        if data_str == 'start':
            print('Starting move behavior!!!')

            self.publish_move_locations()
            
            self.move_behavior()

            print("End of move behavior!!!")

if __name__ == '__main__':
    lab_mover = hsr_lab_mover()