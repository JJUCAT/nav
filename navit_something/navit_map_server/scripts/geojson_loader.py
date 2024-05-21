#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from navit_msgs.msg import GeoJsonTask, GeoJsonArea, GeoJsonPath
from nav_msgs.msg import Path
from foxglove_msgs.msg import GeoJSON
import geodesy.utm as utm
import unique_id
from uuid_msgs.msg import UniqueID

import geopandas as gpd
import sys


if __name__ == '__main__':
    rospy.init_node('geojson_loader')

    if len(sys.argv) < 2:
        rospy.logerr('Please specify the path to the GeoJSON file.')
        sys.exit(1)

    file_path = sys.argv[1]
    rospy.loginfo('Loading GeoJSON file from: {}'.format(file_path))
    
    geojson_map = rospy.Publisher('/geojson_map', GeoJsonTask, queue_size=1, latch=True)
    foxglove_map = rospy.Publisher('/foxglove_map', GeoJSON, queue_size=1, latch=True)
    foxglove_msg = GeoJSON()
    foxglove_msg.geojson= open(file_path, 'r').read()

    geojson_task = GeoJsonTask()

    # 加载 GeoJSON 文件
    gdf = gpd.read_file(file_path)

    # map_origin
    map_origin = gdf.geometry[0].centroid
    map_origin = utm.fromLatLong(map_origin.y, map_origin.x)

    geojson_task.map_origin.header.frame_id = 'map'
    geojson_task.map_origin.pose.position.x = map_origin.easting
    geojson_task.map_origin.pose.position.y = map_origin.northing
    geojson_task.map_origin.pose.position.z = 0.0
    geojson_task.map_origin.pose.orientation.w = 1.0


    # 遍历每个几何对象
    for geometry in gdf['geometry']:
        if geometry is not None:
            # 当前的行数
            row = gdf[gdf['geometry'] == geometry]

            # 确保几何对象是多边形
            if geometry.geom_type == 'Polygon':
                # 创建一个新的任务区域
                area = GeoJsonArea()
                area_id = UniqueID()
                area_id = unique_id.toMsg(unique_id.fromRandom())
                area.id = area_id

                polygon = PolygonStamped()
                polygon.header.frame_id = 'map'
                polygon.header.stamp = rospy.Time.now()

                # 遍历多边形的外部线环的每个点
                for coord in geometry.exterior.coords:
                    utm_coord = utm.fromLatLong(coord[1], coord[0])
                    point = Point32()
                    point.x = utm_coord.easting - map_origin.easting
                    point.y = utm_coord.northing - map_origin.northing
                    point.z = 0.0
                    polygon.polygon.points.append(point)

                area.area = polygon

                try:
                    has_no_go_zone = row['no_go_zone'].values[0] > 0
                except KeyError:
                    has_no_go_zone = False
                    rospy.logwarn('No no_go_zone column found in GeoJSON file. Assuming no no_go_zone.')

                # 检查当前行no_go_zone是否为空
                if has_no_go_zone:
                    rospy.loginfo('Found no_go_zone with {} points.'.format(len(geometry.exterior.coords)))
                    geojson_task.no_go_zones.append(area)
                else:
                    rospy.loginfo('Found work_area with {} points.'.format(len(geometry.exterior.coords)))
                    geojson_task.work_areas.append(area)

            elif geometry.geom_type == 'LineString':
                task_path = GeoJsonPath()
                task_id = UniqueID()
                task_id = unique_id.toMsg(unique_id.fromRandom())
                task_path.id = task_id
                channel = Path()
                # 遍历线的每个点
                for coord in geometry.coords:
                    utm_coord = utm.fromLatLong(coord[1], coord[0])
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = utm_coord.easting - map_origin.easting
                    pose.pose.position.y = utm_coord.northing - map_origin.northing
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    channel.poses.append(pose)
                task_path.path = channel

                
                # 检查当前row是否有work_path，如果有再判断是否为空
                try:
                    has_work_path = row['work_path'].values[0] > 0
                except:
                    rospy.logwarn('No work_path column found in GeoJSON file.')
                    has_work_path = False
                
                if has_work_path:
                    rospy.loginfo('Found work_path with {} points.'.format(len(geometry.coords)))
                    geojson_task.work_paths.append(task_path)
                else:
                    rospy.loginfo('Found channel with {} points.'.format(len(geometry.coords)))
                    geojson_task.channels.append(task_path)

            elif geometry.geom_type == 'Point':

                for coord in geometry.coords:
                    utm_coord = utm.fromLatLong(coord[1], coord[0])
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = utm_coord.easting - map_origin.easting
                    pose.pose.position.y = utm_coord.northing - map_origin.northing
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0

                try:
                    has_dock_pose = row['dock_pose'].values[0]
                except KeyError:
                    has_dock_pose = False
                    rospy.logwarn('No dock_pose column found in GeoJSON file. Assuming no dock_pose.')
                if has_dock_pose:
                    rospy.loginfo('Found dock at {}, {}.'.format(geometry.x, geometry.y))
                    geojson_task.dock_pose = pose

                try:
                    has_map_origin = row['map_origin'].values[0]
                except KeyError:
                    has_map_origin = False
                    rospy.logwarn('No map_origin column found in GeoJSON file. Assuming no map_origin.')

                if has_map_origin:
                    rospy.loginfo('Found map_origin at {}, {}.'.format(geometry.x, geometry.y))
                    geojson_task.map_origin = pose
    # 发布任务
    geojson_map.publish(geojson_task)

    # geojson_map visualization with markers
    geojson_map_marker = rospy.Publisher('/geojson_map_marker', MarkerArray, queue_size=1, latch=True)
    marker_array = MarkerArray()
    
    for polygon in geojson_task.work_areas:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = 'geojson_map'
        marker.id = len(marker_array.markers)
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        marker.text = 'work_area'
        marker.pose.orientation.w = 1.0
        marker.points = polygon.area.polygon.points
        marker_array.markers.append(marker)

        marker_text = Marker()
        marker_text.header.frame_id = 'map'
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.ns = 'geojson_map'
        marker_text.id = len(marker_array.markers)
        marker_text.scale.x = 10.0
        marker_text.scale.y = 10.0
        marker_text.scale.z = 10.0
        marker_text.color.a = 1.0
        marker_text.color.r = 0.0
        marker_text.color.g = 0.0
        marker_text.color.b = 1.0
        marker_text.lifetime = rospy.Duration(0)
        marker_text.frame_locked = True
        marker_text.text = 'work_area'
        marker_text.pose.position.x = polygon.area.polygon.points[0].x
        marker_text.pose.position.y = polygon.area.polygon.points[0].y
        marker_text.pose.position.z = 2.0
        marker_text.pose.orientation.w = 1.0
        marker_array.markers.append(marker_text)


    for channel in geojson_task.channels:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = 'geojson_map'
        marker.id = len(marker_array.markers)
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        marker.text = 'channel'
        marker.pose.orientation.w = 1.0
        marker.points = [pose.pose.position for pose in channel.path.poses]
        marker_array.markers.append(marker)

        marker_text = Marker()
        marker_text.header.frame_id = 'map'
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.ns = 'geojson_map'
        marker_text.id = len(marker_array.markers)
        marker_text.scale.x = 10.0
        marker_text.scale.y = 10.0
        marker_text.scale.z = 10.0
        marker_text.color.a = 1.0
        marker_text.color.r = 1.0
        marker_text.color.g = 0.0
        marker_text.color.b = 0.0
        marker_text.lifetime = rospy.Duration(0)
        marker_text.frame_locked = True
        marker_text.text = 'channel'
        marker_text.pose.position.x = marker.points[0].x
        marker_text.pose.position.y = marker.points[0].y
        marker_text.pose.position.z = 2.0
        marker_text.pose.orientation.w = 1.0
        marker_array.markers.append(marker_text)

    for no_go_zone in geojson_task.no_go_zones:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = 'geojson_map'
        marker.id = len(marker_array.markers)
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        marker.text = 'no_go_zone'
        marker.pose.orientation.w = 1.0
        marker.points = no_go_zone.area.polygon.points
        marker_array.markers.append(marker)

        marker_text = Marker()
        marker_text.header.frame_id = 'map'
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.ns = 'geojson_map'
        marker_text.id = len(marker_array.markers)
        marker_text.scale.x = 10.0
        marker_text.scale.y = 10.0
        marker_text.scale.z = 10.0
        marker_text.color.a = 1.0
        marker_text.color.r = 1.0
        marker_text.color.g = 0.0
        marker_text.color.b = 1.0
        marker_text.lifetime = rospy.Duration(0)
        marker_text.frame_locked = True
        marker_text.text = 'no_go_zone'
        marker_text.pose.position.x = marker.points[0].x
        marker_text.pose.position.y = marker.points[0].y
        marker_text.pose.position.z = 2.0
        marker_text.pose.orientation.w = 1.0
        marker_array.markers.append(marker_text)

    for work_path in geojson_task.work_paths:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = 'geojson_map'
        marker.id = len(marker_array.markers)
        marker.scale.x = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        marker.text = 'work_path'
        marker.pose.orientation.w = 1.0
        marker.points = [pose.pose.position for pose in work_path.path.poses]
        marker_array.markers.append(marker)

        marker_text = Marker()
        marker_text.header.frame_id = 'map'
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.action = Marker.ADD
        marker_text.ns = 'geojson_map'
        marker_text.id = len(marker_array.markers)
        marker_text.scale.x = 10.0
        marker_text.scale.y = 10.0
        marker_text.scale.z = 10.0
        marker_text.color.a = 1.0
        marker_text.color.r = 0.0
        marker_text.color.g = 1.0
        marker_text.color.b = 0.0
        marker_text.lifetime = rospy.Duration(0)
        marker_text.frame_locked = True
        marker_text.text = 'work_path'
        marker_text.pose.position.x = marker.points[0].x
        marker_text.pose.position.y = marker.points[0].y
        marker_text.pose.position.z = 2.0
        marker_text.pose.orientation.w = 1.0
        marker_array.markers.append(marker_text)

    # visualize the map_origin
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.ns = 'geojson_map'
    marker.id = len(marker_array.markers)
    marker.scale.x = 2.5
    marker.scale.y = 2.5
    marker.scale.z = 2.5
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration(0)
    marker.frame_locked = True
    marker.text = 'map_origin'
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker_array.markers.append(marker)

    marker_text = Marker()
    marker_text.header.frame_id = 'map'
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.action = Marker.ADD
    marker_text.ns = 'geojson_map'
    marker_text.id = len(marker_array.markers)
    marker_text.scale.x = 10.0
    marker_text.scale.y = 10.0
    marker_text.scale.z = 10.0
    marker_text.color.a = 1.0
    marker_text.color.r = 1.0
    marker_text.color.g = 1.0
    marker_text.color.b = 0.0
    marker_text.lifetime = rospy.Duration(0)
    marker_text.frame_locked = True
    marker_text.text = 'map_origin'
    marker_text.pose.position.x = 0
    marker_text.pose.position.y = 0
    marker_text.pose.position.z = 2.0
    marker_text.pose.orientation.w = 1.0
    marker_array.markers.append(marker_text)

    # visualize the dock pose
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.ns = 'geojson_map'
    marker.id = len(marker_array.markers)
    marker.scale.x = 2.5
    marker.scale.y = 2.5
    marker.scale.z = 2.5
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.lifetime = rospy.Duration(0)
    marker.frame_locked = True
    marker.text = 'dock_pose'
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = geojson_task.dock_pose.pose.position.x
    marker.pose.position.y = geojson_task.dock_pose.pose.position.y
    marker.pose.position.z = 0.0
    marker_array.markers.append(marker)

    marker_text = Marker()
    marker_text.header.frame_id = 'map'
    marker_text.header.stamp = rospy.Time.now()
    marker_text.type = Marker.TEXT_VIEW_FACING
    marker_text.action = Marker.ADD
    marker_text.ns = 'geojson_map'
    marker_text.id = len(marker_array.markers)
    marker_text.scale.x = 10.0
    marker_text.scale.y = 10.0
    marker_text.scale.z = 10.0
    marker_text.color.a = 1.0
    marker_text.color.r = 0.0
    marker_text.color.g = 1.0
    marker_text.color.b = 1.0
    marker_text.lifetime = rospy.Duration(0)
    marker_text.frame_locked = True
    marker_text.text = 'dock_pose'
    marker_text.pose.position.x = geojson_task.dock_pose.pose.position.x
    marker_text.pose.position.y = geojson_task.dock_pose.pose.position.y
    marker_text.pose.position.z = 2.0
    marker_text.pose.orientation.w = 1.0
    marker_array.markers.append(marker_text)


    geojson_map_marker.publish(marker_array)
    foxglove_map.publish(foxglove_msg)

    rospy.spin()
    

