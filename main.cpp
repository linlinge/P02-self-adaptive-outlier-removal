#include "global.h"
#include "PCLExtend.h"
#include "BasicGeometry.h"
#include "SurfaceFitting.h"
#include "Others.h"
#include "euclidean_distance.h"

using namespace cv;
using namespace std;
using namespace ceres;


/* void example()
{
	SurfaceFitting sf;
	Mat P=sf.Poly33(cloud);
	
	vector<double> dist;
	Mat var=Mat_<double>(10,1);
	
	
 	for(int i=0;i<cloud->points.size();i++)
	{	
		//cout<<"--------------------------------"<<endl;
		double datx=cloud->points[i].x;
		double daty=cloud->points[i].y;
		double datz=cloud->points[i].z;
		double dist_tmp=abs(datz- sf.Calculate(datx,daty));
		//double dist_tmp=sf.AlgebraicDistacne(datx,daty,datz);
		dist.push_back(dist_tmp);		
	}  
	
	ColorBrush cb(dist);
	cout<<cb.min_<<","<<cb.max_<<endl;
	for(int i=0;i<cloud->points.size();i++)
	{ 
		V3 ctmp=cb.GetColor(dist[i]);
		cloud->points[i].r=ctmp.r;
		cloud->points[i].g=ctmp.g;
		cloud->points[i].b=ctmp.b;
	}
}
 */
int main(int argc,char** argv)
{	
	/**************************************Definition, Load File*************************************************/
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	//pcl::PointCloud<PointType>::Ptr cloud_ft(new pcl::PointCloud<PointType>);

	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1) 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}	
	
	// Definition
	vector<int> outlier_idx;
	
	// calculate mean distance
	double mean_dist=ComputeMeanDistance(cloud);
	
	// establish kd-tree
	pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>());
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	kdtree->setInputCloud(cloud);
	
	for(int i=0;i<cloud->points.size();i++){		
		if ( kdtree->radiusSearch (cloud->points[i], 0.01, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
			
			// Get the neighbours
			pcl::PointCloud<PointType>::Ptr cloud_tmp(new pcl::PointCloud<PointType>);
			for (int j = 0; j < pointIdxRadiusSearch.size(); ++j){
				cloud_tmp->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
			}
			
			// Fitting
			SurfaceFitting sf;
			sf.Poly33(cloud_tmp);
			for(int j=0;j<pointIdxRadiusSearch.size();j++){
				int idx=pointIdxRadiusSearch[j];
				double dist_tmp=sf.AlgebraicDistacne(cloud->points[idx].x,cloud->points[idx].y,cloud->points[idx].z);
				//printf("(%f,%f)",dist_tmp,mean_dist);
				if(dist_tmp > 2*mean_dist){
					outlier_idx.push_back(idx);
				}
			}
		}
	}
 	
	for(int i=0;i<cloud->points.size();i++){
		cloud->points[i].r=0;
		cloud->points[i].g=255;
		cloud->points[i].b=0;
	}
		
	
	// mark outlier
	sort(outlier_idx.begin(),outlier_idx.end());
	vector<int>::iterator it=unique(outlier_idx.begin(),outlier_idx.end());
	outlier_idx.erase(it,outlier_idx.end());
	
	cout<<outlier_idx.size()<<endl;
	for(int i=0;i<outlier_idx.size();i++){
		cloud->points[outlier_idx[i]].r=255;
		cloud->points[outlier_idx[i]].g=0;
		cloud->points[outlier_idx[i]].b=0;
	}

	/**************************************Demonstrate Result ***************************************************/
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> 
		viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	//sf.DrawSurface(viewer);
	
	// Set background
	// viewer->setBackgroundColor (0.33, 0.97, 0.59); 
	viewer->setBackgroundColor (1.0f, 1.0f, 1.0f);

	//Set multi-color for point cloud
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  	
	
	//Add the demostration point cloud data
	viewer->addPointCloud<PointType> (cloud, multi_color, "cloud1");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");

	while(!viewer->wasStopped()){	
		viewer->spin();
		 boost::this_thread::sleep (boost::posix_time::microseconds (10));
	}
	
	return 0;
}