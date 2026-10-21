#!/usr/bin/env python3
"""Lightweight obstacle-detection/perception node.

Nav2's own costmap already consumes /scan_filtered directly for planning -- this node
does NOT sit in that critical path (duplicating it would just be two places
that can disagree). Its job is the "Perception" box in the autonomy pipeline
diagram: turn raw LiDAR ranges into human-readable "here are N obstacle
clusters" markers for RViz, and log detection events, so we can SEE what the
robot currently believes is out there while debugging. All detection is from
/scan_filtered (LiDAR with arena-wall returns removed, see wall_filter.py) only --
no forbidden sensors, no pre-supplied obstacle list, no wall returns.


Basic Logic: LiDAR sensor -> convert ranges into (x,y) points->cluster consecutive points->publish MarkerArray

Eg: Imagine the LiDAR sees a boulder, several adjacent laser beams hit different parts of it.
This code converts those hits into points, groups nearby points, calculates each group's center 
and draws a cylinder there 
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray #marker describes one visual object, such as a cylinder, MarkerArray packs multiple Markers into one single message  


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('cluster_distance_threshold', 0.25)#start a new cluster if consecutive points are more than 25cm apart
        self.declare_parameter('min_cluster_points', 3)#ignore groups containing fewer than 3 points
        self.declare_parameter('log_period_sec', 2.0)#Limit detection messages to one every 2 seconds

        self._marker_pub = self.create_publisher(MarkerArray, 'detected_obstacles', 10) #Whenever the node detects clusters, it publishes markers on this topic for RViz
        self._scan_sub = self.create_subscription(LaserScan, 'scan_filtered', self._on_scan, 10)
        self._last_log_time = self.get_clock().now() #Get current ROS clock time

    def _on_scan(self, msg: LaserScan): #Called whenever a new LiDAR scan arrives, convert LiDAR readings into points 
        points = [] #list of valid points 
        angle = msg.angle_min #initialize angle to the first beam's angle
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                points.append((r * math.cos(angle), r * math.sin(angle))) #converting polar to cartesian coordinates ( x = rcos(theta), y = rsin(theta))
            angle += msg.angle_increment #moves to the next beam's angle

        clusters = self._cluster(points)

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, cluster in enumerate(clusters):
            #calculate center of each cluster (cx,cy)
            cx = sum(p[0] for p in cluster) / len(cluster) 
            cy = sum(p[1] for p in cluster) / len(cluster)
            marker = Marker() #turn each cluster into an RviZ marker
            marker.header.frame_id = msg.header.frame_id #tell RviZ which coordinate frame the marker's position belongs to 
            marker.header.stamp = now #set the marker's timestamp
            marker.ns = 'detected_obstacles' #namespace 
            marker.id = i
            #tells RViz to display a cylinder and add or update it 
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            #place the cylinder at estimated cluster center and do appropriate rotation
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3 #length
            marker.scale.y = 0.3 #breadth
            marker.scale.z = 0.4 #height
            marker.color.r = 1.0 #red
            marker.color.g = 0.2 #green
            marker.color.b = 0.1 #blue
            marker.color.a = 0.8 #opacity
            marker.lifetime.sec = 1 #set lifetime of a marker to 1 sec
            marker_array.markers.append(marker) #add the cylinder to the collection of markers 
        self._marker_pub.publish(marker_array)

        # Rate-limited logging -- this callback runs at LiDAR scan rate (10Hz),
        # logging every scan would flood the console for no debugging benefit.
        now_time = self.get_clock().now()
        period = self.get_parameter('log_period_sec').value
        if clusters and (now_time - self._last_log_time).nanoseconds / 1e9 >= period:
            self.get_logger().info(f'Obstacle detected: {len(clusters)} cluster(s) in current scan.')
            self._last_log_time = now_time

    def _cluster(self, points):
        """Group consecutive scan points into obstacle blobs by simple distance gating.

        This is intentionally the simplest clustering that works: walk the scan
        in angular order and start a new cluster whenever the gap to the next
        point exceeds a threshold. It's O(n) and good enough for debugging
        visualization -- a full DBSCAN/Euclidean-cluster-extraction pipeline
        would be overkill for what is essentially a "what does RViz show me" tool.
        """
        threshold = self.get_parameter('cluster_distance_threshold').value
        min_points = self.get_parameter('min_cluster_points').value

        clusters = [] #store completed groups 
        current = [] #store group being built
        for p in points:
            if current and math.dist(current[-1], p) > threshold: #calculate eucledian distance between previous and new point with the formula: d=sqrt{(x_2-x_1)^2+(y_2-y_1)^2}

                if len(current) >= min_points: #check if valid cluster ( more than min number of points)
                    clusters.append(current)
                current = [] #reset current group si that a new one can begin
            current.append(p) #the point that caused the split becomes the first point in the new group
        if len(current) >= min_points:
            clusters.append(current)
        return clusters


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
