#pragma once

#include <vector>
#include <list>
#include <fstream>
#include <opencv2/core/core.hpp> // cv::Point

struct trajectory_t
{
	//typedef float component_t;
	typedef double component_t;
	typedef cv::Point_<component_t> point_t;

	typedef std::vector<point_t>::iterator iterator;
	typedef std::vector<point_t>::const_iterator const_iterator;

	trajectory_t() { }
	trajectory_t(size_t size, unsigned int start_frame);

	void recreate(size_t size, unsigned int start_frame);

	size_t size() const;

	point_t & operator[](size_t i);
	const point_t & operator[](size_t i) const;

	void get_x_components(std::vector<component_t> & components) const;
	void get_y_components(std::vector<component_t> & components) const;

	iterator begin();
	const_iterator begin() const;
	iterator end();
	const_iterator end() const;
		
	std::vector<point_t> _points;
	unsigned int _start_frame; // frames start from 0
}; //trajectory_t

typedef std::list<size_t> partition_t;

void read_dat_header(int & video_length, int & amount_of_elements, std::ifstream & in);
void write_dat_header(int video_length, int amount_of_elements, std::ofstream & out);

// Not safe
void read(trajectory_t & tr, std::ifstream & in);
void write(const trajectory_t & tr, std::ofstream & out);

// Not safe
void read(partition_t & pr, std::ifstream & in);
void write(const partition_t & pr, std::ofstream & out);
