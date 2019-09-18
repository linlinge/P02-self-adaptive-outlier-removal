#pragma once
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

typedef pcl::PointXYZRGBA PointType;
double ComputeMeanDistance(const pcl::PointCloud<PointType>::ConstPtr cloud);