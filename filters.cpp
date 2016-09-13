#include "filters.hpp"
#include <cmath>

bool convolve(const std::vector<double> & f, const std::vector<double> & kernel, std::vector<double> & out)
{
	if( &f == &out ) {
		return false;
	}
	if( f.size() < kernel.size() ) {
		return false;
	}

	out.resize(f.size());

	size_t l_half = floor(kernel.size()/2); // amount of elements on the left
	size_t r_half = kernel.size() - 1 - l_half; // amount of elements on the right

	for(size_t i = l_half; i < f.size()-r_half; ++i) {
		double sum = 0;
		for( size_t j=0; j<kernel.size(); ++j) {
			sum += f[i + j-l_half]*kernel[j];
		}
		out[i] = sum;
	}
	return true;
}
void gaussian_template(size_t win_size, double sigma, std::vector<double> & temp)
{
	temp.resize(win_size);

	int centre = floor(win_size/2);
	double sum = 0;
	for(size_t i=0; i < win_size; ++i) {
		temp[i] = exp(-pow(i-centre,2)/(2*pow(sigma,2)));
		sum += temp[i];
	}
	for(double& i : temp) {
		i /= sum;
	}
}
// Took form T.Brox et.al CFilter.cpp
bool derivative_template(size_t win_size, std::vector<double> & temp)
{
	switch(win_size) {
		case 2: 
			temp.resize(win_size);
			temp[0] = -1;
			temp[1] = 1;
			return true;
		case 3: 
			temp.resize(win_size);
			temp[0] = -0.5;
			temp[1] = 0;
			temp[2] = 0.5;
			return true;
		case 4: 
			temp.resize(win_size);
			temp[0] = 0.041666666666666666666666666666667;
			temp[1] = -1.125;
			temp[2] = 1.125;
			temp[3] = -0.041666666666666666666666666666667;
			return true;
		case 5: 
			temp.resize(win_size);
			temp[0] = 0.083333333333;
			temp[1] = -0.66666666666;
			temp[2] = 0;
			temp[3] = 0.66666666666;
			temp[4] = -0.083333333333;
			return true;
		default: 
			// TODO exception
			return false;
	}
}
