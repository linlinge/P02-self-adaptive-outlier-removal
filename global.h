#pragma onece
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <ceres/ceres.h>
#include <chrono>
#include <math.h>
#include<thread>
#include <opencv2/opencv.hpp>

#include "PCLExtend.h"
#include "BasicGeometry.h"
using namespace cv;
using namespace std;
using namespace ceres;