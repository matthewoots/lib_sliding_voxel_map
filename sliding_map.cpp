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
    Eigen::Vector3d map_size,
    size_t queue_size) : 
    _resolution(resolution), _map_size(map_size),
    _queue_size(queue_size)
{
    _voxel_map = pcl::VoxelGrid<pcl::PointXYZ>::Ptr(
        new pcl::VoxelGrid<pcl::PointXYZ>);
    _voxel_map->setLeafSize(
        _resolution, _resolution, _resolution);

    _module_start_time = std::chrono::system_clock::now();
}

void sliding_map::slidingMap::add_input_cloud(t_p_sc time,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double time_difference = 
        std::chrono::duration<double>(time - _module_start_time).count();
    auto cloud_transform_pair = 
        std::make_pair(cloud, Eigen::Affine3d::Identity());
    
    _cloud_map.insert({time_difference, cloud_transform_pair});

    if (_cloud_map.size() > _queue_size)
        _cloud_map.erase(_cloud_map.begin());
}

void sliding_map::slidingMap::set_pose(t_p_sc time, Eigen::Affine3d transform)
{
    _transform = transform;
}

void sliding_map::slidingMap::set_pose(t_p_sc time, Eigen::Vector3d pos, Eigen::Quaterniond quat)
{
    Eigen::Matrix3d R(quat.toRotationMatrix());
    _transform.translation() = pos;
    _transform.linear() = R;
}

void sliding_map::slidingMap::update_sliding_map()
{
    // did not receive latest pose message
}