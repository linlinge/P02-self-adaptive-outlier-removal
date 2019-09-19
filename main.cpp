#include "global.h"
#include "PCLExtend.h"
#include "BasicGeometry.h"
#include "SurfaceFitting.h"
#include "Others.h"
#include "euclidean_distance.h"

using namespace cv;
using namespace std;
using namespace ceres;


/* Mat Poly33(pcl::PointCloud<PointType>::Ptr cloud)
{
	int n=cloud->points.size();
	Mat x=Mat_<float>(n,1);
	Mat y=Mat_<float>(n,1);
	Mat z=Mat_<float>(n,1);
	
	for(int i=0;i<n;i++)
	{
		x.at<float>(i,0)=cloud->points[i].x;
		y.at<float>(i,0)=cloud->points[i].y;
		z.at<float>(i,0)=cloud->points[i].z;
	}	
	
	return Poly33(x,y,z);
} */

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
	
	
 	SurfaceFitting sf;
	Mat P=sf.Poly33(cloud);
	cout<<P<<endl;
	vector<double> dist;
	Mat var=Mat_<double>(10,1);
	
	
 	for(int i=0;i<cloud->points.size();i++)
	{	
		//cout<<"--------------------------------"<<endl;
		double datx=cloud->points[i].x;
		double daty=cloud->points[i].y;
		double datz=cloud->points[i].z;
		double dist_tmp=abs(datz- sf.Calculate(datx,daty));
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

	/**************************************Demonstrate Result ***************************************************/
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> 
		viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	sf.DrawSurface(viewer);
	
	// Set background
	// viewer->setBackgroundColor (0.33, 0.97, 0.59); 
	viewer->setBackgroundColor (1.0f, 1.0f, 1.0f);

	//Set multi-color for point cloud
	pcl::visualization::PointCloudColorHandlerRGBField<PointType> multi_color(cloud);  	
	
	//Add the demostration point cloud data
	viewer->addPointCloud<PointType> (cloud, multi_color, "cloud1");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud1");

	while(!viewer->wasStopped()){	
		viewer->spin();
		 boost::this_thread::sleep (boost::posix_time::microseconds (10));
	}
	
	return 0;
}