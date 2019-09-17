#include "surface_fitting.h"
Mat Poly33(Mat& x, Mat& y,Mat& z)
{	
	Mat A=Mat_<double>(x.rows,10);
	Mat rst=Mat_<double>(x.rows,10);
	
	for(int i=0;i<x.rows;i++)
	{
		A.at<double>(i,0)=1;
		A.at<double>(i,1)=x.at<double>(i,0);
		A.at<double>(i,2)=y.at<double>(i,0);
		A.at<double>(i,3)=pow(x.at<double>(i,0),2);
		A.at<double>(i,4)=x.at<double>(i,0)*y.at<double>(i,0);
		A.at<double>(i,5)=pow(y.at<double>(i,0),2);
		A.at<double>(i,6)=pow(x.at<double>(i,0),3);
		A.at<double>(i,7)=pow(x.at<double>(i,0),2)*y.at<double>(i,0);
		A.at<double>(i,8)=x.at<double>(i,0)*pow(y.at<double>(i,0),2);
		A.at<double>(i,9)=pow(y.at<double>(i,0),3);
	}
	/* cout<<"A"<<A<<endl;
	cout<<"(A'*A)^-1:"<<(A.t()*A).inv(CV_SVD_SYM)<<endl; */
	rst=(A.t()*A).inv(CV_SVD_SYM)*A.t()*z;
	//cout<<rst<<endl;
	return rst;
}