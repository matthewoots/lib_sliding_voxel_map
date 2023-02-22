/*
* sliding_map.h
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


#ifndef SLIDING_MAP_H
#define SLIDING_MAP_H

#include <math.h>
#include <map>
#include <mutex>
#include <Eigen/Dense>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

typedef std::chrono::time_point<std::chrono::system_clock> t_p_sc;

namespace sliding_map
{
    class slidingMap
    {
        public:

            slidingMap(float resolution, Eigen::Vector3d map_size, size_t queue_size = 1);

            void add_input_cloud(t_p_sc time,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

            void set_pose_stamped(t_p_sc time, Eigen::Affine3d transform);
            void set_pose_stamped(t_p_sc time, Eigen::Vector3d pos, Eigen::Quaterniond quat);

            void update_sliding_map();

            ~slidingMap()
            {
                _cloud_map.clear();
            }
        
        private:

            t_p_sc _module_start_time;

            size_t _queue_size;

            float _resolution;

            Eigen::Vector3d _map_size;

            pcl::VoxelGrid<pcl::PointXYZ>::Ptr _voxel_map;

            std::queue<double, Eigen::Affine3d> _transform;

            /**
             * @brief 
             * 1st pair: (bool) validator (whether this cloud time stamp has a time sync)
             *           (double) relative time from start of module
             * 2nd pair: (pcl) pointcloud pointer
             *           (transform) transform that correspond to the pointcloud
             */
            std::map<std::pair<bool, double>, 
                std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Affine3d>> _cloud_map;

    };
}

#endif