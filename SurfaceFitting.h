#pragma once
#include "global.h"
class SurfaceFitting
{
	public:
		Mat P_;
		double x_min_,x_max_;
		double y_min_,y_max_;
		double z_min_,z_max_;
		
		
		Mat Poly33(Mat& x, Mat& y,Mat& z);
		Mat Poly33(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
		double Calculate(double datx,double daty);
		double AlgebraicDistacne(double datx,double daty, double datz);
		void DrawSurface(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
};
