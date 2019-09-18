#include "euclidean_distance.h"


void numerial_approach(pcl::PointCloud<PointType>::Ptr cloud)
{
	// Definition
	vector<int> outlier_idx;
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
		
		// step 5: store outlier
		for(int j=0;j<dist.size();j++)
		{
			if(dist[j]>sqrt(2)*mean_distance)
			{
				outlier_idx.push_back(pointIdxRadiusSearch[j]);
			}
		}
	}
	
	// step 6: set outlier points with red color
	sort(outlier_idx.begin(),outlier_idx.end());
    unique(outlier_idx.begin(),outlier_idx.end());	
	for(int i=outlier_idx.size()-1;i>=0;i--)
	{
		//cloud->points.erase(cloud->points.begin()+outlier_idx[i]);
		cloud->points[outlier_idx[i]].r=255;
	}
}