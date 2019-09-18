#pragma once
#include "V3.hpp"
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

enum LineInitType{PD,PP};
enum ProjectionType{XY,XZ,YZ};
class Plane;

/* 
	Pointwise Equation:
 		(x-x1)/d.x=(y-y1)/d.y=(z-z1)/d.z
*/
class Line
{
public:
	// represent by parameter	
	V3 Point_,Direction_;
	
	// convert point-direction equation to parameter equation
	Line(V3 dat1, V3 dat2, LineInitType mode);
	V3 IsIntersect(Plane& dat);
	V3 GetProjectionVector(ProjectionType mode);
	float GetProjectionArc(ProjectionType mode);
	V3 TransformTo(ProjectionType mode);
};

class Plane
{
public:
	float A_, B_, C_, D_;
	Plane(V3 P1, V3 P2, V3 P3);
	V3 IsIntersect(Line& dat);
};

class Angle
{
public:
	float arc_, angle_;	
	Angle(V3& mid,V3& left,V3& right);
};


// 
Mat VectorToRotation(V3 orientation_and_arc);
Mat GetRotationMatrixToAxis(V3 vec, int axis);
Eigen::MatrixXf MatToMatrixXf(Mat dat);



/******************************************************************************************************/
//											Surface Fitting
/******************************************************************************************************/
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


Mat Poly33(Mat& x, Mat& y,Mat& z);