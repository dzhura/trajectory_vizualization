#pragma once
#include <vector>
#include <cstddef>

bool convolve(const std::vector<double> & f, const std::vector<double> & kernel, std::vector<double> & out);
void gaussian_template(size_t win_size, double sigma, std::vector<double> & temp);
bool derivative_template(size_t win_size, std::vector<double> & temp);
