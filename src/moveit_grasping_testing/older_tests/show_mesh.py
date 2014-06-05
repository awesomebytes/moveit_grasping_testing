#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'test_mesh_publishing'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('test_mesh_node')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 2

while not rospy.is_shutdown():

    marker = Marker()
    marker.header.frame_id = "/head_mount_xtion_link"
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
#     marker.color.a = 1.0
#     marker.color.r = 1.0
#     marker.color.g = 1.0
#     marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
#     marker.pose.position.x = math.cos(count / 50.0)
#     marker.pose.position.y = math.cos(count / 40.0) 
#     marker.pose.position.z = math.cos(count / 30.0) 
    marker.pose.position.x = 0.1
    marker.pose.position.y = -0.5
    marker.pose.position.z = 0.5
    
    marker.mesh_resource = "package://moveit_grasping_testing/meshes/cloud.ply"
    marker.mesh_use_embedded_materials = True
    #marker.mesh_resource = "package://moveit_grasping_testing/meshes/mesh.stl"
    
    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
    if(count > MARKERS_MAX):
        markerArray.markers.pop(0)
    
    markerArray.markers.append(marker)
    
    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    
    # Publish the MarkerArray
    publisher.publish(markerArray)
    print "publishing..."
    
    count += 1
    
    rospy.sleep(0.01)