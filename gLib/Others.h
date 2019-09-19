#pragma once
#include <vector>
#include "V3.hpp"
using namespace std;
class ColorBrush
{
	public:
		double min_,max_;
		
		ColorBrush(vector<double> dat);
		V3 GetColor(double confidence);
};