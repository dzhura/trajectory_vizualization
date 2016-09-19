// (c) Maxim Dzhura Sep 2016
//
// Reads trajectories and their partition from two .dat files and corresponding frames from a .bmf file
// Show trajectories in the frames and provides x,y projections for each trajectory as well as acceleration
#include <vector>
#include <cassert>
#include <algorithm> // min_element max_element fill_n

#include <fstream>
#include <iostream>
#include <string>

#include <cstdio> // sprintf

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "filters.hpp"

extern "C" {
#include "gnuplot_i.h"
}

struct trajectory_t
{
        trajectory_t() { } 
	trajectory_t(size_t size):
       		_x(size), _y(size), _t(size) { } 

	void resize(size_t size)
       	{ 
		_x.resize(size);
	       	_y.resize(size);
	       	_t.resize(size); 
	} 
	
	size_t size() const 
	{
		return _t.size();
	} 
	
	std::vector<double> _x, _y; 
	std::vector<int> _t; 
}; //trajectory_t

typedef std::vector<unsigned int> partition_t;

// maps a voxel (x,y,t) to an index of trajectory, which it belongs to
class voxel_map_t
{
	public:
	voxel_map_t():
       		_widht(0), _height(0), _video_length(0), _data(0) { }
	voxel_map_t(int width, int height, int video_length):
		_widht(width), _height(height), _video_length(video_length),
	       	_data(new int[_widht*_height*_video_length])
	{
		std::fill_n(_data, _widht*_height*_video_length, -1);
	}

	~voxel_map_t()
	{
		delete [] _data;
	}

	void resize(int width, int height, int video_length)
	{
		if( _data!=0) {
			delete [] _data;
		}

		_widht = width; _height = height; _video_length = video_length;
		_data = new int[_widht*_height*_video_length];
		std::fill_n(_data, _widht*_height*_video_length, -1);
	}

	int & operator() (int x, int y, int t) const
	{
		assert( -1 < x && x < _widht );
		assert( -1 < y && y < _height );
		assert( -1 < t && t < _video_length );

		return _data[_shift(x, y, t)];
	}


	private:
	int _widht;
	int _height;
	int _video_length;

	int * _data;

	private:

	size_t _shift(int x, int y, int t) const {
		return x + _widht*(y + t*_height);
	}

	voxel_map_t(const voxel_map_t &);
	voxel_map_t(voxel_map_t &);
	voxel_map_t & operator=(const voxel_map_t &);
	
}; // voxel_map_t

// A struct for passing arguments to SetMouseCallback
class mouse_callback_input_t
{
	public:
	const unsigned int & _current_frame_number;
	const voxel_map_t & _trajectory_id;
	const std::vector<trajectory_t> & _trajectories;
	const std::vector<partition_t> & _partitions;
	const std::vector<cv::Scalar> & _color_scheme;

	cv::Mat _plot_xy_cords;
	std::string _plot_xy_cords_name;

	gnuplot_ctrl * _plot_xt[2];
	gnuplot_ctrl * _plot_yt[2];
	unsigned int _num_drawn_trajectories;

	public:
	mouse_callback_input_t( const unsigned int & current_frame_number, const voxel_map_t & trajectory_id,
	       			const std::vector<trajectory_t> & trajectories, const std::vector<partition_t> & partitions,
				const std::vector<cv::Scalar> & color_scheme):
			       	_current_frame_number(current_frame_number), _trajectory_id(trajectory_id),
			       	_trajectories(trajectories), _partitions(partitions),
				_color_scheme(color_scheme), _num_drawn_trajectories(0)
	{
		for(int i=0; i<2; ++i) {
			_plot_xt[i] = gnuplot_init();
			_plot_yt[i] = gnuplot_init();
			gnuplot_set_xlabel(_plot_xt[i], (char*)"t");
			gnuplot_set_ylabel(_plot_xt[i], (char*)"x");
			gnuplot_set_xlabel(_plot_yt[i], (char*)"t");
			gnuplot_set_ylabel(_plot_yt[i], (char*)"y");
		}
		gnuplot_cmd(_plot_xt[1], (char*)"set xzeroaxis");
		gnuplot_cmd(_plot_yt[1], (char*)"set xzeroaxis");
	}

	~mouse_callback_input_t() {
		for(int i=0; i<2; ++i) {
			gnuplot_close(_plot_xt[i]);
			gnuplot_close(_plot_yt[i]);
		}
	}
}; // mouse_callback_input_t

static void show_graphs( int event, int x, int y, int dummy, void * args);

void draw_curve( std::vector<int>::const_iterator x_begin, std::vector<int>::const_iterator x_end,
		std::vector<int>::const_iterator y_begin, std::vector<int>::const_iterator y_end, const cv::Scalar & color, cv::Mat & out);
void draw_points( std::vector<int>::const_iterator x_begin, std::vector<int>::const_iterator x_end,
		std::vector<int>::const_iterator y_begin, std::vector<int>::const_iterator y_end, const cv::Scalar & color, cv::Mat & out);

void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<cv::Scalar> & color_scheme, std::vector<cv::Mat> & video, voxel_map_t & pos_2_trajectory);

int main(int argc, char * argv[]) 
{
	const cv::Size boundary_size(5,5); // distance btw end of image to drawing area

	if(argc!=1+3) {
		std::cout << "Usage: " << argv[0] << " <path_to_trajectories> <path_to_partition> <path_to_frames>" << std::endl;
		return 1;
	}

	std::string path_to_trajectories(argv[1]);
	if( path_to_trajectories.compare(path_to_trajectories.find_last_of("."), std::string::npos, ".dat") != 0 ) {
		std::cout << path_to_trajectories << " must be a .dat file" << std::endl;
		return 1;
	}

	std::string path_to_partition(argv[2]);
	if( path_to_partition.compare(path_to_partition.find_last_of("."), std::string::npos, ".dat") != 0 ) {
		std::cout << path_to_partition << " must be a .dat file" << std::endl;
		return 1;
	}

	std::string path_to_frames(argv[3]);
	if( path_to_frames.compare(path_to_frames.find_last_of("."), std::string::npos, ".bmf") != 0 ) {
		std::cout << path_to_frames << " must be a .bmf file" << std::endl;
		return 1;
	}

	//// read input
	// read trajectories
	std::ifstream in_trajectoires(path_to_trajectories);
	if( !in_trajectoires.is_open() ) {
		std::cout << "Canot " << path_to_trajectories << std::endl;
		return 1;
	}

	unsigned int video_length;
	unsigned int trajectory_amount;
	in_trajectoires >> video_length >> trajectory_amount;

	std::vector<trajectory_t> trajectories(trajectory_amount);
	for(size_t i=0; i<trajectories.size(); ++i) {
		int label;
		unsigned int trajectory_size;
		in_trajectoires >> label >> trajectory_size;

		trajectories[i].resize(trajectory_size);
		for(size_t j=0; j < trajectories[i].size(); ++j) {
			in_trajectoires >> trajectories[i]._x[j] >> trajectories[i]._y[j] >> trajectories[i]._t[j];
			trajectories[i]._x[j] = lroundf(trajectories[i]._x[j]);
			trajectories[i]._y[j] = lroundf(trajectories[i]._y[j]);
		}
	}
	in_trajectoires.close();

	// read partition of trajectories
	std::ifstream in_partition(path_to_partition);
	if( !in_partition.is_open() ) {
		std::cout << "Canot " << path_to_partition << std::endl;
		return 1;
	}

	unsigned int video_length_2;
	unsigned int trajectory_amount_2;
	in_partition >> video_length_2 >> trajectory_amount_2;

	std::vector<partition_t> partitions(trajectory_amount_2);
	for(size_t i=0; i<partitions.size(); ++i) {
		unsigned int partition_amount;
		in_partition >> partition_amount;

		partitions[i].resize(partition_amount);
		for(size_t j=0; j < partitions[i].size(); ++j) {
			in_partition >> partitions[i][j];
		}
	}
	in_partition.close();

	// read frames
	std::string root_dir = path_to_frames.substr(0, path_to_frames.find_last_of("/")+1);
	if(root_dir.compare(path_to_frames) == 0) { // if frames are in current folder
		root_dir = "./";
	}

	std::ifstream in_frames(path_to_frames);
	if( !in_frames.is_open() ) {
		std::cout << "Canot " << path_to_frames << std::endl;
		return 1;
	}

	unsigned int video_length_3;
	int dummy;
	in_frames >> video_length_3 >> dummy;

	std::vector<cv::Mat> frames(video_length_3);
	for(size_t i=0; i<frames.size(); ++i) {
		std::string frame_name;
		in_frames >> frame_name;

		frames[i] = cv::imread(root_dir + frame_name);
		if(frames[i].data == 0) {
			std::cout << "Cannot read " << (root_dir + frame_name) << std::endl;
			return 1;
		}
	}

	// check input
	if(video_length_3 != video_length) {
		std::cout << "Trajectory file is wrong"	<< std::endl;
		return 1;
	}
	if(video_length_3 != video_length_2) {
		std::cout << "Partition file is wrong" << std::endl;
		return 1;
	}

	//// prepare for vizualization
	// generate a color scheme for drawing of trajectories
	// color scheme idea was taken from T. Brox dense trajectories
	std::vector<cv::Scalar> color_scheme(1024);
	for(int i=0; i<256; ++i) {
		color_scheme[i] = cv::Scalar(255, i, 0);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[256+i] = cv::Scalar(255-i, 255, 0);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[512+i] = cv::Scalar(0, 255, i);
	}
	for(int i=0; i<256; ++i) {
		color_scheme[768+i] = cv::Scalar(255, 255, 255);
	}

	// generate Max`s colors for drawing of projections
	std::vector<cv::Scalar> max_colors(11);
	max_colors[0] = cv::Scalar(255, 0, 0, 1);
	max_colors[1] = cv::Scalar(0, 255, 0);
	max_colors[2] = cv::Scalar(255, 255, 0);
	max_colors[3] = cv::Scalar(255, 0, 255);
	max_colors[4] = cv::Scalar(0, 255, 255);
	max_colors[5] = cv::Scalar(255, 255, 255);
	max_colors[6] = cv::Scalar(0, 125, 0);
	max_colors[7] = cv::Scalar(125, 125, 0);
	max_colors[8] = cv::Scalar(125, 0, 125);
	max_colors[9] = cv::Scalar(0, 125, 125);
	max_colors[10] = cv::Scalar(125, 125, 125);

	// draw trajectories on the frames
	voxel_map_t pos_2_trajectory;
	draw_trajectories(trajectories, color_scheme, frames, pos_2_trajectory);

	// prepare mouse call handler
	cv::Scalar background_color(0,0,0);
	unsigned int current_frame_number = 0;
	mouse_callback_input_t mouse_callback_input(current_frame_number, pos_2_trajectory, trajectories, partitions, max_colors);
	mouse_callback_input._plot_xy_cords = cv::Mat(frames[0].size(), CV_8UC3, background_color);
	mouse_callback_input._plot_xy_cords_name = std::string("xy projection"); 

	//// do vizualization
	std::string current_frame_name("Current frame");
	cv::imshow(current_frame_name, frames[current_frame_number]);
	cv::setMouseCallback(current_frame_name, show_graphs, &mouse_callback_input);
	cv::imshow(mouse_callback_input._plot_xy_cords_name, mouse_callback_input._plot_xy_cords);
	cv::moveWindow(mouse_callback_input._plot_xy_cords_name, frames[0].size().width, 0);

	for(;;) {
		int c = cv::waitKey(0);
		if( (c & 255) == 27 ) { // if ESC
			return 0;
		}
		switch( (char)c) {
			case 'f':
				// Go to the next frame
				if(current_frame_number < video_length-1) {
					++current_frame_number;
					cv::imshow(current_frame_name, frames[current_frame_number]);
				}
				break;
			case 'b':
				// Go to the previous frame
				if(current_frame_number > 0) {
					--current_frame_number;
					cv::imshow(current_frame_name, frames[current_frame_number]);
				}
				break;
			case 'r':
				// Refresh
				mouse_callback_input._num_drawn_trajectories = 0;
				mouse_callback_input._plot_xy_cords = background_color;
				cv::imshow(mouse_callback_input._plot_xy_cords_name, mouse_callback_input._plot_xy_cords);
				//cv::moveWindow(mouse_callback_input._plot_xy_cords_name, frames[0].size().width, 0);
				for(int i=0; i<2; ++i) {
					gnuplot_resetplot(mouse_callback_input._plot_xt[i]);
					gnuplot_resetplot(mouse_callback_input._plot_yt[i]);
				}
				break;
		}
	}
	return 0;
}

static void show_graphs( int event, int x, int y, int, void * args)
{
	if(event == cv::EVENT_MOUSEMOVE) {
		return;
	}

	switch(event) {
		case cv::EVENT_LBUTTONDOWN: {
			mouse_callback_input_t * callback_input = (mouse_callback_input_t*)args;

			unsigned int current_frame = callback_input->_current_frame_number;

			int seleceted_traj_id = callback_input->_trajectory_id(x, y, current_frame);
			if(seleceted_traj_id == -1) { // trajectory is not selected
				return;
			}
			
			const partition_t & partition = callback_input->_partitions[seleceted_traj_id];
			const trajectory_t & trajectory = callback_input->_trajectories[seleceted_traj_id];

			// Select a colors
			const std::vector<cv::Scalar> & color_scheme = callback_input->_color_scheme;
			if( callback_input->_num_drawn_trajectories >= color_scheme.size() ) {
				std::cout << "Not enough colors. Press 'r' to refresh" << std::endl;
				break;
			}

			cv::Scalar color_for_projections = color_scheme[callback_input->_num_drawn_trajectories];
			cv::Scalar color_for_partition = cv::Scalar(0,0,255);

			// round trajectory
			std::vector<int> rounded_x(trajectory.size()), rounded_y(trajectory.size());
			for(size_t i=0; i<trajectory.size(); ++i) {
				rounded_x[i] = lroundf(trajectory._x[i]);
				rounded_y[i] = lroundf(trajectory._y[i]);
			}

			// get trajectory partition points
			std::vector<int> x_partition(partition.size()), y_partition(partition.size());
			for(unsigned int i=0; i<partition.size(); ++i) {
				x_partition[i] = rounded_x[partition[i]];
				y_partition[i] = rounded_y[partition[i]];
			}

			// Draw projections and partitions of trajectory
			// xy
			draw_curve(rounded_x.cbegin(), rounded_x.cend(), rounded_y.cbegin(), rounded_y.cend(), color_for_projections, callback_input->_plot_xy_cords);
			draw_points(x_partition.cbegin(), x_partition.cend(), y_partition.cbegin(), y_partition.cend(), color_for_partition, callback_input->_plot_xy_cords);

			// Show them
			cv::imshow(callback_input->_plot_xy_cords_name, callback_input->_plot_xy_cords);

			// plot speed and accelearation
			if(trajectory.size() < 5) {
				break; // trajectory is too short
			}

			// compute speed and acceleration
			std::vector<double> gaussian, derivative;
			gaussian_template(3, 3.0, gaussian);
			derivative_template(3, derivative);

			std::vector<double> smooth_x, smooth_y;
			convolve(trajectory._x, gaussian, smooth_x);
			smooth_x.front() = trajectory._x.front();
			smooth_x.back() = trajectory._x.back();
			convolve(trajectory._y, gaussian, smooth_y);
			smooth_y.front() = trajectory._y.front();
			smooth_y.back() = trajectory._y.back();

			std::vector<double> x_speed, y_speed;
			convolve(smooth_x, derivative, x_speed);
			convolve(smooth_y, derivative, y_speed);

			std::vector<double> x_acceleration, y_acceleration;
			convolve(x_speed, derivative, x_acceleration);
			convolve(y_speed, derivative, y_acceleration);

			// Prepare to plot
			for(int i=0; i<2; ++i) {
				gnuplot_resetplot(callback_input->_plot_xt[i]);
				gnuplot_resetplot(callback_input->_plot_yt[i]);
				gnuplot_setstyle(callback_input->_plot_xt[i], (char*)"lines");
				gnuplot_setstyle(callback_input->_plot_yt[i], (char*)"lines");
			}
			char trajectory_title[50];
			sprintf(trajectory_title, "trajectory %d", callback_input->_num_drawn_trajectories);
			char speed_title[50];
			sprintf(speed_title, "speed %d", callback_input->_num_drawn_trajectories);
			char acceleration_title[50];
			sprintf(acceleration_title, "acceleration %d", callback_input->_num_drawn_trajectories);

			// Plot projections, speed and acceleration
			// xt
			gnuplot_plot_x(callback_input->_plot_xt[0], &trajectory._x[0], trajectory.size(), trajectory_title);
			gnuplot_plot_x(callback_input->_plot_xt[0], &smooth_x[0], smooth_x.size(), (char*)"smooth");

			gnuplot_plot_x(callback_input->_plot_xt[1], &x_speed[0], x_speed.size(), speed_title);
			gnuplot_plot_x(callback_input->_plot_xt[1], &x_acceleration[0], x_acceleration.size(), acceleration_title);
			// yt
			gnuplot_plot_x(callback_input->_plot_yt[0], &trajectory._y[0], trajectory.size(), trajectory_title);
			gnuplot_plot_x(callback_input->_plot_yt[0], &smooth_y[0], smooth_y.size(), (char*)"smooth");

			gnuplot_plot_x(callback_input->_plot_yt[1], &y_speed[0], y_speed.size(), speed_title);
			gnuplot_plot_x(callback_input->_plot_yt[1], &y_acceleration[0], y_acceleration.size(), acceleration_title);

			// get partitions
			std::vector<double> 	x_speed_partition(partition.size()), y_speed_partition(partition.size()),
						x_acceleration_partition(partition.size()), y_acceleration_partition(partition.size()),
						t_partition(partition.begin(), partition.end());
			for(unsigned int i=0; i<partition.size(); ++i) {
				x_speed_partition[i] = x_speed[partition[i]];
				y_speed_partition[i] = y_speed[partition[i]];
				x_acceleration_partition[i] = x_acceleration[partition[i]];
				y_acceleration_partition[i] = y_acceleration[partition[i]];
			}

			// perepare to plot
			for(int i=0; i<2; ++i) {
				gnuplot_setstyle(callback_input->_plot_yt[i], (char*)"points");
				gnuplot_setstyle(callback_input->_plot_xt[i], (char*)"points");
			}

			// plot pratition points
			// xt
			gnuplot_plot_xy(callback_input->_plot_xt[1], &t_partition[0], &x_speed_partition[0], x_speed_partition.size(), (char*)"partition");
			gnuplot_plot_xy(callback_input->_plot_xt[1], &t_partition[0], &x_acceleration_partition[0], x_acceleration_partition.size(), (char*)"partition");
			// ty
			gnuplot_plot_xy(callback_input->_plot_yt[1], &t_partition[0], &y_speed_partition[0], y_speed_partition.size(), (char*)"partition");
			gnuplot_plot_xy(callback_input->_plot_yt[1], &t_partition[0], &y_acceleration_partition[0], y_acceleration_partition.size(), (char*)"partition");


			callback_input->_num_drawn_trajectories++;
			break;
		}
		case cv::EVENT_RBUTTONDOWN: {
		// TODO show 3D trajectory
			break;
		}
	}
}

void draw_curve( std::vector<int>::const_iterator x_begin, std::vector<int>::const_iterator x_end,
		std::vector<int>::const_iterator y_begin, std::vector<int>::const_iterator y_end, const cv::Scalar & color, cv::Mat & out)
{
	assert( std::distance(x_begin, x_end) == std::distance(y_begin, y_end) );
	for(auto x = x_begin+1, y = y_begin+1; x!=x_end && y!=y_end; ++x, ++y) {
		cv::Point from = cv::Point(*(x-1), *(y-1));
		cv::Point to = cv::Point(*x, *y);
		cv::line(out, from, to, color);
	}
}
void draw_points( std::vector<int>::const_iterator x_begin, std::vector<int>::const_iterator x_end,
		std::vector<int>::const_iterator y_begin, std::vector<int>::const_iterator y_end, const cv::Scalar & color, cv::Mat & out)
{
	assert( std::distance(x_begin, x_end) == std::distance(y_begin, y_end) );
	for(auto x = x_begin, y = y_begin; x!=x_end && y!=y_end; ++x, ++y) {
		cv::circle(out, cv::Point(*x, *y), 1, color, -1);
	}
}

void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<cv::Scalar> & color_scheme, std::vector<cv::Mat> & video, voxel_map_t & pos_2_trajectory)
{
	int video_length = video.size();
	int width = video[0].size().width; 
	int height = video[0].size().height;
			
	pos_2_trajectory.resize(width, height, video_length);

	for(unsigned int j=0; j<trajectories.size(); ++j) {
		for(size_t i=0; i<trajectories[j].size(); ++i) {
			int t = trajectories[j]._t[i]; 

			int ix = lroundf(trajectories[j]._x[i]); 
			int iy = lroundf(trajectories[j]._y[i]); 
			cv::Point p1, p2; 
			p1.x = (ix-1<0)? 0: ix-1; 
			p1.y = (iy-1<0)? 0: iy-1; 
			p2.x = (ix+1>=width)? width-1: ix+1; 
			p2.y = (iy+1>=height)? height-1: iy+1; 
			cv::rectangle(video[t], p1, p2, color_scheme[i*10], CV_FILLED); 

			for(short y = std::max(p1.y-2, 0); y <= std::min(p2.y+2, height-1); ++y) {
				for(short x = std::max(p1.x-2, 0); x <= std::min(p2.x+2, width-1); ++x) {
					pos_2_trajectory(x,y,t) = j;
				}
			}
		}
	}
}
