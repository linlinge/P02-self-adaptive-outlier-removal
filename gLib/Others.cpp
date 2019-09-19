#include "Others.h"
ColorBrush::ColorBrush(vector<double> dat)
{
	double min_tmp=INT_MAX;
	double max_tmp=-INT_MAX;
	for(int i=0;i<dat.size();i++)
	{
		if(min_tmp>dat[i])
			min_tmp=dat[i];
		
		if(max_tmp<dat[i])
			max_tmp=dat[i];
	}
	min_=min_tmp;
	max_=max_tmp;
}

V3 ColorBrush::GetColor(double confidence)
{
	V3 color_temp;
	double n=4.0; // 
	double step=(max_-min_)/n;
	double c=confidence-min_;
	if(c<step)
	{
		color_temp.r=0;
		color_temp.g=confidence/step;
		color_temp.b=255;
	}
	else if(c<2*step)
	{
		color_temp.r=0;
		color_temp.g=255;
		color_temp.b=255-(confidence-step)/step;
	}
	else if(c<3*step)
	{
		color_temp.r=(confidence-2*step)/step;
		color_temp.g=255;
		color_temp.b=0;
	}
	else if(c<4*step)
	{
		color_temp.r=255;
		color_temp.g=255-(confidence-3*step)/step;
		color_temp.b=0;
	}
	
	return color_temp;
}