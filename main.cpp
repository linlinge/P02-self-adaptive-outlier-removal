#include "global.h"
#include "euclidean_distance.h"

int main(int argc,char** argv)
{	
	/**************************************Definition, Load File*************************************************/
	
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloud_ft(new pcl::PointCloud<PointType>);

	if (pcl::io::loadPLYFile<PointType>(argv[1], *cloud) == -1) 
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}






	


	/**************************************Demonstrate Result ***************************************************/
	
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