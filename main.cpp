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
#include <utility> // pair

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "filters.hpp"
#include "trajectory_t.hpp"

extern "C" {
#include "gnuplot_i.h"
}

const int not_trajectory_index = -1;
const int indent = 1; // indent from a point to left, right, top and bottom

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
			assert(_plot_xt[i] != NULL || !" the input video is too big"); // TODO allow resizing of video
			assert(_plot_yt[i] != NULL || !" the input video is too big");
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

//// Functions for drawing trajectories in a video sequence with specific color
// Color correspond to age of trajectory. FIXME what to do if color_scheme has less colors than required?
void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<cv::Scalar> & color_scheme, std::vector<cv::Mat> & video);
// Color correspond to partition of trajectory, color of a partition of trajectory differs form colors of neightbour partitions from the same trajectory
void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<partition_t> & partitions, std::vector<cv::Mat> & video);

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

	std::string root_dir = path_to_list_of_frames.substr(0, path_to_list_of_frames.find_last_of("/")+1);
	if(root_dir.compare(path_to_list_of_frames) == 0) { // if frames are in current folder
		root_dir = "./";
	}

	//// read input
	// read frames
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
			std::cout << "Size of " << i+1 << "-th frame differs from sizes of previous frames" << std::endl;
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
	read_dat_header(trajectories_video_length, trajectory_amount, in_trajectoires);
	if(trajectories_video_length != video_length) {
		std::cout << "Trajectories are extracted from a video of another length" << std::endl;
		return 1;
	}

	std::vector<trajectory_t> trajectories(trajectory_amount);
	for(trajectory_t & trajectory : trajectories) {
		read(trajectory, in_trajectoires);
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
	read_dat_header(partitions_video_length, partitions_trajectory_amount, in_partition);
	if(partitions_video_length != video_length) {
		std::cout << "Partitions were extracted from a video of another length" << std::endl;
		return 1;
	}
	if(partitions_trajectory_amount != trajectory_amount) {
		std::cout << "There is no 1-to-1 correspondence btw trajectories and their partitions" << std::endl;
		return 1;
	}


	// Note: it is supposed trajectory and its partition have the same index
	std::vector<partition_t> partitions(trajectory_amount);
	for(partition_t & partition : partitions) {
		read(partition, in_partition);
	}
	in_partition.close();

	//// prepare for vizualization
	draw_trajectories(trajectories, partitions, frames);

	// create a map: a trajectory point to the index of the trajectory
	int video_size[] = {frames[0].size().width, frames[0].size().height, video_length}; // sizes of all frames are the same
	cv::Mat pos_2_trajectory_id(3/*amount of dims*/, video_size, CV_32SC1, not_trajectory_index);
	int trajectory_id=0;
	for(const trajectory_t & trajectory : trajectories) {
		int frame_id = trajectory._start_frame;
		for( const trajectory_t::point_t & point : trajectory._points ) {

			// TODO the same piece of code is in draw_trajectories. Make a separate function
			int floor_x = floor(point.x); 
			int floor_y = floor(point.y); 
			int ceil_x = floor_x + 1;
			int ceil_y = floor_y + 1;

			cv::Point p1, p2; 
			p1.x = (floor_x-indent<0)? 0: floor_x-indent; 
			p1.y = (floor_y-indent<0)? 0: floor_y-indent; 
			p2.x = (ceil_x+indent>=frames[0].size().width-1)? frames[0].size().width-1: ceil_x+indent;
			p2.y = (ceil_y+indent>=frames[0].size().height-1)? frames[0].size().height-1: ceil_y+indent; 

			for(int y=p1.y; y<=p2.y; ++y)
			for(int x=p1.x; x<=p2.x; ++x) {
				pos_2_trajectory_id.at<int>(x, y, frame_id) = trajectory_id;
			}
			frame_id++;
		}
		trajectory_id++;
	}

	// prepare mouse call handler
	cv::Scalar background_color(0,0,0);
	int current_frame_number = 0;

	std::vector<cv::Scalar> colors(11);
	colors[0] = cv::Scalar(255, 0, 0, 1);
	colors[1] = cv::Scalar(0, 255, 0);
	colors[2] = cv::Scalar(255, 255, 0);
	colors[3] = cv::Scalar(255, 0, 255);
	colors[4] = cv::Scalar(0, 255, 255);
	colors[5] = cv::Scalar(255, 255, 255);
	colors[6] = cv::Scalar(0, 125, 0);
	colors[7] = cv::Scalar(125, 125, 0);
	colors[8] = cv::Scalar(125, 0, 125);
	colors[9] = cv::Scalar(0, 125, 125);
	colors[10] = cv::Scalar(125, 125, 125);

	mouse_callback_input_t mouse_callback_input(current_frame_number, pos_2_trajectory_id, trajectories, partitions, colors);
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
				for(int i=0; i<2; ++i) {
					gnuplot_resetplot(mouse_callback_input._plot_xt[i]);
					gnuplot_resetplot(mouse_callback_input._plot_yt[i]);
				}
				break;

			case 'p':
				c = cv::waitKey(0);
				switch( (char)c ) {
					case 'p': // print plot
						cv::imwrite(root_dir + "plot_xy.jpg", mouse_callback_input._plot_xy);
						break;
					case 't': // print trajectories
						int id=0;
						for(const cv::Mat & frame : frames) {
							std::string str_id = std::to_string(id++);
							str_id = std::string(4 - str_id.length(), '0') + str_id;
							cv::imwrite(root_dir + "Trajectories" + str_id + ".jpg", frame);
						}
						break;
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
			if( callback_input->_num_drawn_trajectories >= callback_input->_color_scheme.size() ) {
				std::cout << "Not enough colors. Press 'r' to refresh" << std::endl;
				break;
			}
			
			const trajectory_t & trajectory = callback_input->_trajectories[seleceted_traj_id];
			const partition_t & partition = callback_input->_partitions[seleceted_traj_id];

			// Select a color
			cv::Scalar color_for_projections = callback_input->_color_scheme[callback_input->_num_drawn_trajectories];
			cv::Scalar color_for_partition = cv::Scalar(0,0,255);

			std::vector<trajectory_t::component_t> x, y;
			trajectory.get_x_components(x);
			trajectory.get_y_components(y);

			// round trajectories for drawing
			std::vector<int> rounded_x(trajectory.size()), rounded_y(trajectory.size());
			for(size_t i=0; i<trajectory.size(); ++i) {
				rounded_x[i] = lround(x[i]);
				rounded_y[i] = lround(y[i]);
			}

			// get trajectory partition points
			std::vector<int> x_partition(partition.size());
			std::vector<int> y_partition(partition.size());
			int i=0;
			for(const partition_t::value_type & p : partition) {
				x_partition[i] = rounded_x[p];
				y_partition[i] = rounded_y[p];
				i++;
			}

			// Draw projections and partitions of trajectory
			// xy
			draw_curve(rounded_x.cbegin(), rounded_x.cend(), rounded_y.cbegin(), rounded_y.cend(), color_for_projections, callback_input->_plot_xy);
			draw_points(x_partition.cbegin(), x_partition.cend(), y_partition.cbegin(), y_partition.cend(), color_for_partition, callback_input->_plot_xy);

			// Show them
			cv::imshow(callback_input->_plot_xy_name, callback_input->_plot_xy);

			// plot speed and accelearation
			const int template_size=3;
			if(trajectory.size() < template_size) {
				break; // trajectory is too short
			}

			// compute speed and acceleration
			std::vector<trajectory_t::component_t> gaussian, derivative;
			gaussian_template(template_size, 3.0/*sigma*/, gaussian);
			derivative_template(template_size, derivative);

			std::vector<trajectory_t::component_t> smooth_x, smooth_y;
			convolve(x, gaussian, smooth_x);
			convolve(y, gaussian, smooth_y);
			smooth_x[0] = smooth_x[1];
			smooth_x[smooth_x.size()-1] = smooth_x[smooth_x.size()-2];
			smooth_y[0] = smooth_y[1];
			smooth_y[smooth_y.size()-1] = smooth_y[smooth_y.size()-2];

			std::vector<trajectory_t::component_t> x_speed, y_speed;
			convolve(smooth_x, derivative, x_speed);
			convolve(smooth_y, derivative, y_speed);
			x_speed[0] = x_speed[1];
			x_speed[x_speed.size()-1] = x_speed[x_speed.size()-2];
			y_speed[0] = y_speed[1];
			y_speed[y_speed.size()-1] = y_speed[y_speed.size()-2];

			std::vector<trajectory_t::component_t> x_acceleration, y_acceleration;
			convolve(x_speed, derivative, x_acceleration);
			convolve(y_speed, derivative, y_acceleration);
			x_acceleration[0] = x_acceleration[1];
			x_acceleration[x_acceleration.size()-1] = x_acceleration[x_acceleration.size()-2];
			y_acceleration[0] = y_acceleration[1];
			y_acceleration[y_acceleration.size()-1] = y_acceleration[y_acceleration.size()-2];

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
			gnuplot_plot_x(callback_input->_plot_xt[0], &x[0], trajectory.size(), trajectory_title);
			gnuplot_plot_x(callback_input->_plot_xt[0], &smooth_x[0], smooth_x.size(), (char*)"smooth");

			gnuplot_plot_x(callback_input->_plot_xt[1], &x_speed[0], x_speed.size(), speed_title);
			gnuplot_plot_x(callback_input->_plot_xt[1], &x_acceleration[0], x_acceleration.size(), acceleration_title);
			// yt
			gnuplot_plot_x(callback_input->_plot_yt[0], &y[0], trajectory.size(), trajectory_title);
			gnuplot_plot_x(callback_input->_plot_yt[0], &smooth_y[0], smooth_y.size(), (char*)"smooth");

			gnuplot_plot_x(callback_input->_plot_yt[1], &y_speed[0], y_speed.size(), speed_title);
			gnuplot_plot_x(callback_input->_plot_yt[1], &y_acceleration[0], y_acceleration.size(), acceleration_title);

			// get partitions
			std::vector<trajectory_t::component_t> x_speed_partition(partition.size());
			std::vector<trajectory_t::component_t> y_speed_partition(partition.size());
			i=0;
			for(const partition_t::value_type & p : partition) {
				x_speed_partition[i] = x_speed[p];
				y_speed_partition[i] = y_speed[p];
				i++;
			}
			std::vector<trajectory_t::component_t> x_acceleration_partition(partition.size());
			std::vector<trajectory_t::component_t> y_acceleration_partition(partition.size());
			i=0;
			for(const partition_t::value_type & p : partition) {
				x_acceleration_partition[i] = x_acceleration[p];
				y_acceleration_partition[i] = y_acceleration[p];
				i++;
			}
			// gnu plot requires the same type for both variables
			std::vector<trajectory_t::component_t> t_partition(partition.begin(), partition.end());

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

	for(const trajectory_t & trajectory : trajectories ) {
		int frame_id = trajectory._start_frame;
		int i=0;
		for(const trajectory_t::point_t & point : trajectory._points ) {
			int floor_x = floor(point.x); 
			int floor_y = floor(point.y); 
			int ceil_x = floor_x + 1;
			int ceil_y = floor_y + 1;

			cv::Point p1, p2; 
			p1.x = (floor_x-indent<0)? 0: floor_x-indent; 
			p1.y = (floor_y-indent<0)? 0: floor_y-indent; 
			p2.x = (ceil_x+indent>=width-1)? width-1: ceil_x+indent;
			p2.y = (ceil_y+indent>=height-1)? height-1: ceil_y+indent; 

			cv::rectangle(video[frame_id], p1, p2, color_scheme[i*10], CV_FILLED); 
			frame_id++;
			i++;
		}
	}
}
void draw_trajectories(const std::vector<trajectory_t> & trajectories, const std::vector<partition_t> & partitions, std::vector<cv::Mat> & video)
{
	assert(trajectories.size() == partitions.size());

	int width = video[0].size().width;
	int height = video[0].size().height;
	// Convert video to HSV
	for(cv::Mat & frame : video) {
		cv::cvtColor(frame, frame, CV_BGR2HSV);
	}

	const int hue_step_deg = 30;
	const int saturation = 255;
	const int value = 255;

	for( size_t i=0; i<trajectories.size(); ++i ) {

		const trajectory_t & trajectory = trajectories[i];
		partition_t::const_iterator p_cut_point = partitions[i].begin();
		int frame_id = trajectory._start_frame;
		int hue = 0;
		cv::Scalar color(hue, 0/*saturation*/, 0/*value*/); // distinguish newly initilalized trajectory by a  unqiue color
		for( size_t j=0; j<trajectory.size(); ++j ) {
			
			int floor_x = floor(trajectory[j].x); 
			int floor_y = floor(trajectory[j].y); 
			int ceil_x = floor_x + 1;
			int ceil_y = floor_y + 1;
			
			cv::Point p1, p2; 
			p1.x = (floor_x-indent<0)? 0: floor_x-indent; 
			p1.y = (floor_y-indent<0)? 0: floor_y-indent; 
			p2.x = (ceil_x+indent>=width-1)? width-1: ceil_x+indent;
			p2.y = (ceil_y+indent>=height-1)? height-1: ceil_y+indent; 

			cv::rectangle(video[frame_id++], p1, p2, color, CV_FILLED); 
			if( j == *p_cut_point ) {
				hue= (hue+hue_step_deg)%360;
				color = cv::Scalar(hue, saturation, value);
				p_cut_point++;
			}
		}
	}
	// Convert video back to BGR
	for(cv::Mat & frame : video) {
		cv::cvtColor(frame, frame, CV_HSV2BGR);
	}
}
