#include "trajectory_t.hpp"
#include <cassert>
#include <cmath> // lroundf

trajectory_t::trajectory_t(size_t size, unsigned int start_frame):
		_points(size), _start_frame(start_frame) { }

void trajectory_t::recreate(size_t size, unsigned int start_frame)
{
	_points.resize(size);
	_start_frame = start_frame;
}

size_t trajectory_t::size() const
{
	return _points.size();
}

trajectory_t::point_t & trajectory_t::operator[](size_t i)
{
	return _points[i];
}

const trajectory_t::point_t & trajectory_t::operator[](size_t i) const
{
	return _points[i];
}

void trajectory_t::get_x_components(std::vector<component_t> & components) const
{
	components.resize(_points.size());
	for(size_t i=0; i<_points.size(); ++i) {
		components[i]=_points[i].x;
	}
}
void trajectory_t::get_y_components(std::vector<component_t> & components) const
{
	components.resize(_points.size());
	for(size_t i=0; i<_points.size(); ++i) {
		components[i]=_points[i].y;
	}
}

trajectory_t::iterator trajectory_t::begin()
{
	return _points.begin();
}

trajectory_t::const_iterator trajectory_t::begin() const
{
	return _points.begin();
}

trajectory_t::iterator trajectory_t::end()
{
	return _points.end();
}

trajectory_t::const_iterator trajectory_t::end() const
{
	return _points.end();
}

void read_dat_header(int & video_length, int & amount_of_elements, std::ifstream & in)
{
	assert(in.is_open());
	in >> video_length >> amount_of_elements;
}
void write_dat_header(int video_length, int amount_of_elements, std::ofstream & out)
{
	assert(out.is_open());
	out << video_length << std::endl;
	out << amount_of_elements << std::endl;
}

void read(trajectory_t & tr, std::ifstream & in)
{
	assert(in.is_open());

	int label;
	in >> label;
	size_t size;
	in >> size;

	tr._points.resize(size);

	trajectory_t::point_t p;
	int frame;
	in >> p.x >> p.y >> frame;

	tr._points[0] = p;
	tr._start_frame = frame;

	for(trajectory_t::iterator it=1+tr.begin(); it!=tr.end(); ++it) {
		in >> it->x >> it->y >> frame;
	}
}
void write(const trajectory_t & tr, std::ofstream & out)
{
	assert(tr.size() > 0);
	assert(out.is_open());

	out << (int)0/*label*/ << ' ' << tr.size() << std::endl;

	unsigned int frame = tr._start_frame;
	out << lroundf(tr[0].x) << ' ' << lroundf(tr[0].y) << ' ' << frame++ << std::endl;
	for(trajectory_t::const_iterator it=1+tr.begin(); it!=tr.end(); ++it) {
		out << it->x << ' ' << it->y << ' ' << frame++ << std::endl;
	}
}

void read(partition_t & pr, std::ifstream & in)
{
	assert(in.is_open());

	size_t size;
	in >> size;

	for(size_t i=0; i<size; ++i) {
		partition_t::value_type cut_point;
		in >> cut_point;
		pr.push_back(cut_point);
	}
}
void write(const partition_t & pr, std::ofstream & out)
{
	assert(out.is_open());

	out << pr.size() << std::endl;
	for(partition_t::const_iterator cit=pr.begin(); cit!=pr.end(); ++cit) {
		out << *cit << ' ';
	}
	out << std::endl;
}
