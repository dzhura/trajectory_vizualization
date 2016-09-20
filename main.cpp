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

const int not_trajectory_index = -1;
const int indent = 1; // indent from a point to left, right, top and bottom

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

// A struct for passing arguments to SetMouseCallback
class mouse_callback_input_t
{
	public:
	const int & _current_frame_number;
	const cv::Mat & _pos_2_trajectory_id;
	const std::vector<trajectory_t> & _trajectories;
	const std::vector<partition_t> & _partitions;
	const std::vector<cv::Scalar> & _color_scheme;

	cv::Mat _plot_xy;
	std::string _plot_xy_name;

	gnuplot_ctrl * _plot_xt[2];
	gnuplot_ctrl * _plot_yt[2];
	unsigned int _num_drawn_trajectories;

	public:
	mouse_callback_input_t( const int & current_frame_number, const cv::Mat & trajectory_id,
	       			const std::vector<trajectory_t> & trajectories, const std::vector<partition_t> & partitions,
				const std::vector<cv::Scalar> & color_scheme):
			       	_current_frame_number(current_frame_number), _pos_2_trajectory_id(trajectory_id),
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

// FIXME what to do if color_scheme has less colors than required?
void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<cv::Scalar> & color_scheme, std::vector<cv::Mat> & video);

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

	std::string path_to_list_of_frames(argv[3]);
	if( path_to_list_of_frames.compare(path_to_list_of_frames.find_last_of("."), std::string::npos, ".bmf") != 0 ) {
		std::cout << path_to_list_of_frames << " must be a .bmf file" << std::endl;
		return 1;
	}

	//// read input
	// read frames
	std::string root_dir = path_to_list_of_frames.substr(0, path_to_list_of_frames.find_last_of("/")+1);
	if(root_dir.compare(path_to_list_of_frames) == 0) { // if frames are in current folder
		root_dir = "./";
	}

	std::ifstream in_frames(path_to_list_of_frames);
	if( !in_frames.is_open() ) {
		std::cout << "Cannot " << path_to_list_of_frames << std::endl;
		return 1;
	}

	int video_length;
	int dummy;
	in_frames >> video_length >> dummy;

	std::vector<cv::Mat> frames(video_length);
	for(size_t i=0; i<frames.size(); ++i) {
		std::string frame_name;
		in_frames >> frame_name;

		frames[i] = cv::imread(root_dir + frame_name);
		if(frames[i].data == 0) {
			std::cout << "Cannot read " << (root_dir + frame_name) << std::endl;
			return 1;
		}
		if(frames[i].size() != frames[0].size()) {
			std::cout << "Size of " << i+1 << "-th frame differ from sizes of previous frames" << std::endl;
			return 1;
		}
	}
	in_frames.close();

	// read trajectories
	std::ifstream in_trajectoires(path_to_trajectories);
	if( !in_trajectoires.is_open() ) {
		std::cout << "Cannot open " << path_to_trajectories << std::endl;
		return 1;
	}

	int trajectories_video_length;
	int trajectory_amount;
	in_trajectoires >> trajectories_video_length >> trajectory_amount;
	if(trajectories_video_length != video_length) {
		std::cout << "Trajectories are extracted from a video of another length" << std::endl;
		return 1;
	}

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
		std::cout << "Cannot open " << path_to_partition << std::endl;
		return 1;
	}

	int partitions_video_length;
	int partitions_trajectory_amount;
	in_partition >> partitions_video_length >> partitions_trajectory_amount;
	if(partitions_video_length != video_length) {
		std::cout << "Partitions were extracted from a video of another length" << std::endl;
		return 1;
	}
	if(partitions_trajectory_amount != trajectory_amount) {
		std::cout << "There is no 1-to-1 correspondence btw trajectories and their partitions" << std::endl;
		return 1;
	}

	std::vector<partition_t> partitions(trajectory_amount);
	for(size_t i=0; i<partitions.size(); ++i) {
		unsigned int partition_amount;
		in_partition >> partition_amount;

		partitions[i].resize(partition_amount);
		for(size_t j=0; j < partitions[i].size(); ++j) {
			in_partition >> partitions[i][j];
		}
	}
	in_partition.close();

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
	draw_trajectories(trajectories, color_scheme, frames);

	// create a map: position of a trajectory element to the trajectory index
	int video_size[] = {frames[0].size().width, frames[0].size().height, video_length}; // sizes of all frames are the same
	cv::Mat pos_2_trajectory_id(3, video_size, CV_32SC1, not_trajectory_index);
	for(size_t i = 0; i < trajectories.size(); ++i) {
		for( size_t point_id=0; point_id < trajectories[i].size(); ++point_id) {

			int floor_x = floor(trajectories[i]._x[point_id]); 
			int floor_y = floor(trajectories[i]._y[point_id]); 
			int ceil_x = floor_x + 1;
			int ceil_y = floor_y + 1;

			cv::Point p1, p2; 
			p1.x = (floor_x-indent<0)? 0: floor_x-indent; 
			p1.y = (floor_y-indent<0)? 0: floor_y-indent; 
			p2.x = (ceil_x+indent>=frames[0].size().width-1)? frames[0].size().width-1: ceil_x+indent;
			p2.y = (ceil_y+indent>=frames[0].size().height-1)? frames[0].size().height-1: ceil_y+indent; 

			int t = trajectories[i]._t[point_id];
			for(int y=p1.y; y<=p2.y; ++y)
			for(int x=p1.x; x<=p2.x; ++x) {
				pos_2_trajectory_id.at<int>(x, y, t) = i;
			}
		}
	}

	// prepare mouse call handler
	cv::Scalar background_color(0,0,0);
	int current_frame_number = 0;

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

	mouse_callback_input_t mouse_callback_input(current_frame_number, pos_2_trajectory_id, trajectories, partitions, max_colors);
	mouse_callback_input._plot_xy = cv::Mat(frames[0].size(), CV_8UC3, background_color);
	mouse_callback_input._plot_xy_name = std::string("xy projection"); 

	//// do vizualization
	std::string current_frame_name("Current frame");
	cv::imshow(current_frame_name, frames[current_frame_number]);
	cv::setMouseCallback(current_frame_name, show_graphs, &mouse_callback_input);
	cv::imshow(mouse_callback_input._plot_xy_name, mouse_callback_input._plot_xy);
	cv::moveWindow(mouse_callback_input._plot_xy_name, frames[0].size().width, 0);

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
				mouse_callback_input._plot_xy = background_color;
				cv::imshow(mouse_callback_input._plot_xy_name, mouse_callback_input._plot_xy);
				//cv::moveWindow(mouse_callback_input._plot_xy_name, frames[0].size().width, 0);
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

			int seleceted_traj_id = callback_input->_pos_2_trajectory_id.at<int>(x, y, current_frame);
			if(seleceted_traj_id == not_trajectory_index) { // trajectory is not selected
				return;
			}
			
			const partition_t & partition = callback_input->_partitions[seleceted_traj_id];
			const trajectory_t & trajectory = callback_input->_trajectories[seleceted_traj_id];

			// Select a color
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
			draw_curve(rounded_x.cbegin(), rounded_x.cend(), rounded_y.cbegin(), rounded_y.cend(), color_for_projections, callback_input->_plot_xy);
			draw_points(x_partition.cbegin(), x_partition.cend(), y_partition.cbegin(), y_partition.cend(), color_for_partition, callback_input->_plot_xy);

			// Show them
			cv::imshow(callback_input->_plot_xy_name, callback_input->_plot_xy);

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

void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<cv::Scalar> & color_scheme, std::vector<cv::Mat> & video)
{
	int width = video[0].size().width; 
	int height = video[0].size().height;
			
	for(unsigned int j=0; j<trajectories.size(); ++j) {
		for(size_t i=0; i<trajectories[j].size(); ++i) {
			int floor_x = floor(trajectories[j]._x[i]); 
			int floor_y = floor(trajectories[j]._y[i]); 
			int ceil_x = floor_x + 1;
			int ceil_y = floor_y + 1;
			int t = trajectories[j]._t[i]; 

			cv::Point p1, p2; 
			p1.x = (floor_x-indent<0)? 0: floor_x-indent; 
			p1.y = (floor_y-indent<0)? 0: floor_y-indent; 
			p2.x = (ceil_x+indent>=width-1)? width-1: ceil_x+indent;
			p2.y = (ceil_y+indent>=height-1)? height-1: ceil_y+indent; 
			cv::rectangle(video[t], p1, p2, color_scheme[i*10], CV_FILLED); 
		}
	}
}
