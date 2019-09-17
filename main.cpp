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
#include "surface_fitting.h"
#include "pcl_expansion.h"
#include<thread>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
using namespace ceres;

#define RADIUS 0.01f
#define THREASHOLD_DISTANCE 0.005

struct CostFunctor
{
    CostFunctor (double p[10],double test_point[3])
	{
		for(int i=0;i<10;i++) p_[i]=p[i];
		for(int i=0;i<3;i++) test_point_[i]=test_point[i];		
	}
	
    // 残差的计算
    template <typename T>
    bool operator() (const T* const x, const T* const y, const T* const z, const T* const lamda,T* residual ) const     // 残差
    {
        residual[0] = pow(x[0]-test_point_[0],2) + pow(y[0]-test_point_[1],2) + pow(z[0]-test_point_[2],2) 
					  + lamda[0]*(p_[0]+p_[1]*x[0]+p_[2]*y[0]+ p_[3]*pow(x[0],2) + p_[4]*x[0]*y[0]+p_[5]*pow(y[0],2)+p_[6]*pow(x[0],3)+p_[7]*pow(x[0],2)*y[0]+p_[8]*x[0]*pow(y[0],2)+p_[9]*pow(y[0],3) - z[0] );
        return true;
    }
	
    double p_[10], test_point_[3];    // x,y数据
};


int main(int argc,char** argv)
{
	// Definition, load file
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_ft(new pcl::PointCloud<PointType>);
	vector<int> outlier_idx;
	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1) 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	float mean_distance=ComputeMeanDistance(cloud);
	
	
	// establish kdtree
	pcl::search::KdTree<PointType>::Ptr kdtree (new pcl::search::KdTree<PointType>());
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	kdtree->setInputCloud(cloud);
	
	for(int i=0;i<cloud->points.size();i++)
	{
		// step 1: find neighbour within radius
		kdtree->radiusSearch (cloud->points[i], RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
		for(int j=0;j<pointIdxRadiusSearch.size();j++)
		{
			cloud_tmp->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
		}
		
		// step 2: convert neighbours data into Mat
		Mat x=Mat_<double>(cloud_tmp->points.size(),1);
		Mat y=Mat_<double>(cloud_tmp->points.size(),1);
		Mat z=Mat_<double>(cloud_tmp->points.size(),1);		
		for(int j=0;j<cloud_tmp->points.size();j++)
		{
			x.at<double>(j,0)=cloud_tmp->points[j].x;
			y.at<double>(j,0)=cloud_tmp->points[j].y;
			z.at<double>(j,0)=cloud_tmp->points[j].z;
		}
		
		// step 3: calculate the polynomial
		Mat P=Poly33(x,y,z);
		//cout<<P<<endl;
		
		// step 4: calculate the distances between points and surface 
		double array_p[10]={P.at<double>(0,0), P.at<double>(1,0), P.at<double>(2,0), P.at<double>(3,0), P.at<double>(4,0), P.at<double>(5,0), P.at<double>(6,0), P.at<double>(7,0), P.at<double>(8,0), P.at<double>(9,0)};
		vector<double> dist;
		
		for(int j=0;j<x.rows;j++)
		{
			// define cost function
			double test_point[3]={x.at<double>(j,0),y.at<double>(j,0),z.at<double>(j,0)};
			double foot_point_x,foot_point_y,foot_point_z,lamda;
			foot_point_x=test_point[0];
			foot_point_y=test_point[1];
			foot_point_z=array_p[0]+array_p[1]*foot_point_x+array_p[2]*foot_point_y+ array_p[3]*pow(foot_point_x,2) 
							+ array_p[4]*foot_point_x*foot_point_y+array_p[5]*pow(foot_point_y,2)
							+array_p[6]*pow(foot_point_x,3)+array_p[7]*pow(foot_point_x,2)*foot_point_y+array_p[8]*foot_point_x*pow(foot_point_y,2)+array_p[9]*pow(foot_point_y,3);
			lamda=1;
			
			
			// problem
			ceres::Problem problem;
			CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1,1,1,1>(new CostFunctor(array_p, test_point));
			problem.AddResidualBlock (cost_function, nullptr, &foot_point_x,&foot_point_y,&foot_point_z,&lamda);
			
			// configurate solver
			ceres::Solver::Options options;     // 这里有很多配置项可以填
			options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
			options.minimizer_progress_to_stdout = false;   // 输出到cout
			
			ceres::Solver::Summary summary;                // 优化信息
			chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
			ceres::Solve ( options, &problem, &summary );
			
			
			float dist_tmp=sqrt(pow(x.at<double>(j,0) - foot_point_x,2) + pow( y.at<double>(j,0) - foot_point_y ,2) + pow( z.at<double>(j,0) - foot_point_z,2));						
			dist.push_back(dist_tmp);
			//cout<<"("<<test_point[0]<<","<<test_point[1]<<","<<test_point[2]<<" -> ("<<foot_point_x<<","<<foot_point_y<<","<<foot_point_z<<")"<<endl;
		}
		
		for(int j=0;j<dist.size();j++)
		{
			if(dist[j]>sqrt(2)*mean_distance)
			{
				outlier_idx.push_back(pointIdxRadiusSearch[j]);
			}
		}
	}
	
	sort(outlier_idx.begin(),outlier_idx.end());
    unique(outlier_idx.begin(),outlier_idx.end());
	
	for(int i=outlier_idx.size()-1;i>=0;i--)
	{
		//cloud->points.erase(cloud->points.begin()+outlier_idx[i]);
		cloud->points[outlier_idx[i]].r=255;
	}
	
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> 
		viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	// Set background
	// viewer->setBackgroundColor (0.33, 0.97, 0.59); 
	viewer->setBackgroundColor (1.0f, 1.0f, 1.0f);

	//Set multi-color for point cloud
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  	
	
	//Add the demostration point cloud data
	viewer->addPointCloud<PointType> (cloud, multi_color, "cloud1");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");

	while(!viewer->wasStopped()){	
		viewer->spin();
		 boost::this_thread::sleep (boost::posix_time::microseconds (10));
	}
	
	return 0;
}