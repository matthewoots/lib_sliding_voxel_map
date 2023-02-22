/*
* sliding_map.cpp
*
* ---------------------------------------------------------------------
* Copyright (C) 2023 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include "sliding_map.h"

sliding_map::slidingMap::slidingMap(float resolution, 
    Eigen::Vector3d map_size, double time_sync_threshold, 
    size_t queue_size) : 
    _resolution(resolution), _map_size(map_size), 
    _time_sync_threshold(time_sync_threshold),
    _queue_size(queue_size)
{
    _voxel_map = pcl::VoxelGrid<pcl::PointXYZ>::Ptr(
        new pcl::VoxelGrid<pcl::PointXYZ>);
    _voxel_map->setLeafSize(
        _resolution, _resolution, _resolution);

    _module_start_time = std::chrono::system_clock::now();

    _parameters_set = true;
}

sliding_map::slidingMap::slidingMap()
{
    _voxel_map = pcl::VoxelGrid<pcl::PointXYZ>::Ptr(
        new pcl::VoxelGrid<pcl::PointXYZ>);

    _module_start_time = std::chrono::system_clock::now();
}

void sliding_map::slidingMap::set_parameters(
    float resolution, Eigen::Vector3d map_size, 
    double time_sync_threshold, size_t queue_size)
{
    _resolution = resolution;
    _voxel_map->setLeafSize(
        _resolution, _resolution, _resolution);
    _map_size = map_size;
    _time_sync_threshold = time_sync_threshold;
    _queue_size = queue_size;

    _parameters_set = true;
}

void sliding_map::slidingMap::add_input_cloud(t_p_sc time,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double time_from_start = 
        std::chrono::duration<double>(time - _module_start_time).count();

    cloud_mutex.lock();

    _cloud_queue.push(
        std::make_pair(time_from_start, cloud));

    while (_cloud_queue.size() > _queue_size)
        _cloud_queue.pop();
    
    cloud_mutex.unlock();
}

void sliding_map::slidingMap::set_pose_stamped(
    t_p_sc time, Eigen::Affine3d transform)
{   
    double time_from_start = 
        std::chrono::duration<double>(time - _module_start_time).count();

    pose_mutex.lock();
    _transform_stamped = std::make_pair(time_from_start, transform);
    pose_mutex.unlock();
}

void sliding_map::slidingMap::set_pose_stamped(
    t_p_sc time, Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
    double time_from_start = 
        std::chrono::duration<double>(time - _module_start_time).count();
    Eigen::Matrix3d R(quat.toRotationMatrix());
    Eigen::Affine3d t;
    t.translation() = pos;
    t.linear() = R;

    pose_mutex.lock();
    _transform_stamped = std::make_pair(time_from_start, t);
    pose_mutex.unlock();
}

void sliding_map::slidingMap::update_sliding_map()
{
    assert(_parameters_set);

    cloud_mutex.lock();
    // combine the point clouds
    while (!_cloud_queue.empty())
    {
        *_filtered_xyz_map += *(_cloud_queue.front().second);
        _cloud_queue.pop();
    }

    _voxel_map->setInputCloud(_filtered_xyz_map);
    _voxel_map->filter(*_filtered_xyz_map);
    cloud_mutex.unlock();

    pose_mutex.lock();
    Eigen::Vector3d center = _transform_stamped.second.translation();
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setMin(
        Eigen::Vector4f(center.x() - _map_size.x(), 
        center.y() - _map_size.y(), 
        center.z() - _map_size.z(), 1.0));
    crop_filter.setMax(
        Eigen::Vector4f(center.x() + _map_size.x(), 
        center.y() + _map_size.y(), 
        center.z() + _map_size.z(), 1.0));

    crop_filter.setInputCloud(_filtered_xyz_map);
    crop_filter.filter(*_filtered_xyz_map);

    pose_mutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr sliding_map::slidingMap::get_sliding_map()
{
    return _filtered_xyz_map;
}